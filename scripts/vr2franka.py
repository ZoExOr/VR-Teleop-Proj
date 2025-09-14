import time
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R
import panda_py
from panda_py import controllers
from panda_py import libfranka
from oculus_reader.reader import OculusReader

oculus_reader = OculusReader()

"""
Boundary constraints to prevent the Franka arm from colliding
with the wall, the table, or with itself.
Defined in absolute position coordinates.
"""
BOUNDARY = {
    'x': (0.1, 1.4),
    'y': (-0.24, 0.35),
    'z': (-0.1, 0.7)
}

def get_transformation(Rm, P):
    T = np.eye(4)
    T[:3, :3] = Rm
    T[:3, 3] = P.reshape(3)
    return T
    
def get_hand_pose_and_buttons():
    """
    Read hand pose (rotation + position) and button states from the Oculus controller,
    then convert the hand pose from the "head frame" to the "robot frame".
    
    Returns:
        R_hand2rbt : np.ndarray, shape (3,3)
            Rotation matrix of the hand relative to the robot frame
        P_hand2rbt : np.ndarray, shape (3,1)
            Position vector of the hand relative to the robot frame
        buttons : dict
            Button states from the controller
    """
    euler_head2rbt = np.array([90, 0, 0])
    R_head2rbt = R.from_euler('xyz', euler_head2rbt, degrees=True).as_matrix()

    transforms, buttons = oculus_reader.get_transformations_and_buttons()
    T_hand2head = transforms['r'].copy()
    R_hand2head = T_hand2head[:3, :3]
    P_hand2head = T_hand2head[:3, 3].reshape(3, 1)
    P_head2rbt = np.zeros((3,1))
    T_head2rbt = get_transformation(R_head2rbt, P_head2rbt)
    T_hand2head[:3,:3] = R_hand2head
    T_hand2head[:3, 3] = P_hand2head.reshape(3)
    T_hand2rbt = T_head2rbt @ T_hand2head
    R_hand2rbt = T_hand2rbt[:3, :3]
    P_hand2rbt = T_hand2rbt[:3, 3].reshape(3, 1)

    return R_hand2rbt, P_hand2rbt, buttons

def wait_for_valid_pose(reader, timeout=3.0):
    start = time.time()
    while time.time() - start < timeout:
        transforms, buttons = reader.get_transformations_and_buttons()
        if transforms and 'r' in transforms:
            T = transforms['r']
            if not np.allclose(T, 0) and np.all(np.isfinite(T)):
                return transforms, buttons
        time.sleep(0.01)
    raise RuntimeError("No valid Oculus pose within timeout")

def exceed_boundary(pos):
    pos = pos.flatten()

    return not (BOUNDARY['x'][0] <= pos[0] <= BOUNDARY['x'][1] and
                BOUNDARY['y'][0] <= pos[1] <= BOUNDARY['y'][1] and
                BOUNDARY['z'][0] <= pos[2] <= BOUNDARY['z'][1])

def clamp_pos(x_d):
    x_d[0] = np.clip(x_d[0], BOUNDARY['x'][0], BOUNDARY['x'][1])
    x_d[1] = np.clip(x_d[1], BOUNDARY['y'][0], BOUNDARY['y'][1])
    x_d[2] = np.clip(x_d[2], BOUNDARY['z'][0], BOUNDARY['z'][1])
    
    return x_d

def grasp(gripper):
    """
    Background thread that listens to the controller's 'RG' button
    and controls the gripper accordingly.

    Logic:
    - When the RG button is pressed (rising edge) and the gripper is not holding,
    attempt to close the gripper and grasp an object.
    - When the RG button is released (falling edge) and the gripper is holding,
    open the gripper to release the object.
    - Tracks 'is_grasping' state to avoid repeated commands.
    """

    is_grasping = False    
    prev_rg     = False     

    while True:
        _, _, buttons = get_hand_pose_and_buttons()
        if buttons is None:
            continue

        rg = buttons['RG']

        if rg and not prev_rg and not is_grasping:
            # print("button detected, closing…")
            try:
                ok = gripper.grasp(0.01, 0.05, 15, 20, 1)
                if ok:
                    is_grasping = True
                    # print("grasp success")
                else:
                    print("grasp failed")
            except RuntimeError as e:
                print("grasp error:", e)

        if (not rg) and prev_rg and is_grasping:
            # print("opening the gripper")
            try:
                gripper.move(0.08, 0.05)
            except RuntimeError as e:
                print("open error:", e)
            is_grasping = False

        prev_rg = rg       

def warm_up(move_thresh=0.05):
    """
    Wait until a significant movement of the controller is detected.
    The function continuously measures the controller's position and only returns 
    when the displacement exceeds the given threshold (in meters).
    """
    print("\033[1;33mInitializing... Please move the controller significantly until detection is confirmed.\033[0m")

    _, pos_prev, _ = get_hand_pose_and_buttons()
    while True:
        _, pos_cur, _ = get_hand_pose_and_buttons()
        delta = float(np.linalg.norm(pos_cur - pos_prev))  
        if delta > move_thresh:
            print("\033[1;33mController movement detected, starting teleoperation in 5s...\033[0m")
            return

        pos_prev = pos_cur.copy()
        time.sleep(0.01) 


def main():
    hostname = '172.16.0.2'
    panda = panda_py.Panda(hostname)
    gripper = libfranka.Gripper(hostname)
    panda.move_to_start()
    impedance = np.diag([600, 600, 600, 40, 40, 40])  
    # impedance = np.diag([60, 60, 60, 20, 20, 20])  
    t_grasp = threading.Thread(target=grasp, args=(gripper,), daemon=True)
    t_grasp.start()

    ctrl = controllers.CartesianImpedance(
        impedance=impedance,
        damping_ratio=0.7,
        nullspace_stiffness=0.5,
        # filter_coeff=0.1
        filter_coeff=0.8
    )
    panda.get_robot().set_collision_behavior(
        [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0],
        [200.0, 200.0, 200.0, 200.0, 200.0, 200.0, 200.0],
        [200.0, 200.0, 200.0, 200.0, 200.0, 200.0],
        [200.0, 200.0, 200.0, 200.0, 200.0, 200.0]
    )

    rot_prev, pos_prev, buttons_prev = None, None, None
    t_wait_start = time.time()

    print("\033[1;33msetting up oculus reader...\033[0m")
    oculus_reader.wait_until_ready(timeout_s=5.0)


    print("\033[1;33mpress any button to start...\033[0m")
    while True:
        rot_prev, pos_prev, buttons_prev = get_hand_pose_and_buttons()
        if buttons_prev is not None:
            if buttons_prev.get('RG', False) or buttons_prev.get('RTr', False) or buttons_prev.get('A', False) or buttons_prev.get('B', False):
                break
        if time.time() - t_wait_start > 10:
            t_wait_start = time.time()
        time.sleep(0.01)

    warm_up()
    time.sleep(5)
    print("start")
    rot_prev, pos_prev, buttons_prev = get_hand_pose_and_buttons()

    panda.start_controller(ctrl)
    
    x0 = panda.get_position().reshape(3, 1)
    R_hand_ref = R.from_matrix(rot_prev)
    q_robot_ref = panda.get_orientation().copy()   
    q_prev = q_robot_ref.copy()                 

    '''
    scale for fliping the object within a pan & paper drawing: 
        pos_scale = 2
        ori_scale = 1.5

    scale (possibly) for juggling the ball:
        pos_scale = 2.2
        ori_scale = 2
    '''

    pos_scale = 2
    ori_scale = 2

    with panda.create_context(frequency=90) as ctx:
        while ctx.ok():
            rot_curr, pos_curr, buttons_curr = get_hand_pose_and_buttons()
            if pos_curr is None or buttons_curr is None:
                continue

            d_pos = pos_scale * (pos_curr - pos_prev)
            x_d = x0 + d_pos

            R_hand_curr = R.from_matrix(rot_curr)
            R_delta = R_hand_curr * R_hand_ref.inv()
            rotvec = R_delta.as_rotvec()     
            rotvec_scaled = ori_scale * rotvec
            R_scaled = R.from_rotvec(rotvec_scaled)
            R_target = R_scaled * R.from_quat(q_robot_ref)
            q_new = R_target.as_quat()

            if np.dot(q_new, q_prev) < 0:
                q_new = -q_new  

            q_d = q_new

            if exceed_boundary(x_d):
                clamp_pos(x_d)

            if buttons_curr['RTr']:
                pos_prev = pos_curr
                R_hand_ref = R.from_matrix(rot_curr)
                q_robot_ref = q_prev.copy()
                time.sleep(0.1)
                continue

            ctrl.set_control(x_d.reshape(3, 1), q_d)

            x0 = x_d.copy()
            q_prev = q_new           
            rot_prev, pos_prev, buttons_prev = rot_curr, pos_curr, buttons_curr

if __name__ == '__main__':
    main()