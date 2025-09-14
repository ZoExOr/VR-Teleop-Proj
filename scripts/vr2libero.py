import numpy as np
import time
import os
from scipy.spatial.transform import Rotation as R
import tkinter as tk
from tkinter import messagebox

from oculus_reader.reader import OculusReader
from libero.libero.envs import *
from env_wrappers import OnScreenRenderEnv

CURRENT_DIR = os.getcwd()
PARENT_DIR = os.path.dirname(CURRENT_DIR)

oculus_reader = OculusReader()
def get_transformation(R, P):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = P.reshape(3)
    return T
    

def get_hand_pose_and_buttons():
    """
    Read the right-hand controller pose and button states,
    apply coordinate corrections, and transform from head frame
    into robot frame.
    """
    euler_head2rbt = np.array([90, 180, -90])   
    R_head2rbt = R.from_euler('xyz', euler_head2rbt, degrees=True).as_matrix()  
    transforms, buttons = oculus_reader.get_transformations_and_buttons()   
 
    Z = np.diag([1, 1, -1]) 

    T_hand2head = transforms['r']  

    R_hand2head = T_hand2head[:3, :3]
    R_hand2head = Z @ R_hand2head @ Z      
    P_hand2head = T_hand2head[:3, 3].reshape(3, 1)
    P_hand2head = Z @ P_hand2head
   
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


def show_help_popup():
    help_text = (
        "Instructions:\n"
        "-  Trigger (index finger): Hold to pause and calibrate — release when your hand feels right\n"
        "-  Grip (middle finger): Press to close the gripper\n"
        "-  A button: Reset the environment\n"
        "-  B button: Close the environment\n"
        "\n"
        "Close the tab then Press any button to start"
    )
    root = tk.Tk()
    root.withdraw() 
    messagebox.showinfo("Welcome to VR Gripper Control", help_text)
    root.destroy()

def warm_up(warm_up_break):
    warmup_start = time.time()
    while time.time() - warmup_start < warm_up_break:
        _ = get_hand_pose_and_buttons() 
        time.sleep(0.01)
    print("\033[1;33mWarm-up complete, starting teleop...\033[0m")


def main():
    # create the customized environment
    task_bddl_file = [bddl file path]
    env_args = {
        "bddl_file_name": task_bddl_file,
        "camera_heights": 256,
        "camera_widths": 256,
        "has_renderer": True,
        "has_offscreen_renderer": True,
        "ignore_done": True
    }
    env = OnScreenRenderEnv(**env_args)

    env.seed(0)

    print("\033[1;33msetting up oculus reader...\033[0m")
    oculus_reader.wait_until_ready(timeout_s=5.0)

    obs = env.reset()
    P_gripper2rbt_prev = obs['robot0_eef_pos'].reshape(3,1)
    quat = obs['robot0_eef_quat']    
    R_gripper2rbt_prev = R.from_quat(quat).as_matrix()   
    T_gripper2rbt_prev = get_transformation(R_gripper2rbt_prev, P_gripper2rbt_prev)

    R_hand2rbt_prev, P_hand2rbt_prev, buttons_prev = get_hand_pose_and_buttons()
    T_hand2rbt_prev = get_transformation(R_hand2rbt_prev, P_hand2rbt_prev)
    
    T_hand2gripper = np.linalg.inv(T_gripper2rbt_prev) @ T_hand2rbt_prev

    show_help_popup()

    print("\033[1;33mpress any button to start...\033[0m")

    while not (buttons_prev['RG'] or buttons_prev['RTr'] or buttons_prev['A'] or buttons_prev['B']):
        R_hand2rbt_prev, P_hand2rbt_prev, buttons_prev = get_hand_pose_and_buttons()
        time.sleep(0.01)
        
    warm_up(warm_up_break=3.0)

    done = False
    while not done:
        R_hand2rbt_curr, P_hand2rbt_curr, buttons_curr = R_hand2rbt_prev, P_hand2rbt_prev, buttons_prev
        R_hand2rbt_prev, P_hand2rbt_prev, buttons_prev  = get_hand_pose_and_buttons()

        gripper_open_prev = buttons_curr['RG'] 
        need_calibrated = buttons_curr['RTr']

        delta_P_hand2rbt = P_hand2rbt_curr - P_hand2rbt_prev
        dx, dy, dz = 300 * (delta_P_hand2rbt).flatten()
 
        T_hand2rbt_curr = get_transformation(R_hand2rbt_curr, P_hand2rbt_curr)

        #manually calibration
        if need_calibrated:
            R_gripper2rbt_curr = R.from_quat(obs['robot0_eef_quat']).as_matrix()
            P_gripper2rbt_curr = obs['robot0_eef_pos'].reshape(3, 1)
            T_gripper2rbt_curr = get_transformation(R_gripper2rbt_curr, P_gripper2rbt_curr)
            T_hand2gripper = np.linalg.inv(T_gripper2rbt_curr) @ T_hand2rbt_curr
        
            continue

        T_gripper2rbt_curr = T_hand2rbt_curr @ np.linalg.inv(T_hand2gripper)
        R_gripper2rbt_curr = T_gripper2rbt_curr[:3, :3]   
                     
        q_prev = R.from_matrix(R_gripper2rbt_prev).as_quat()
        q_curr = R.from_matrix(R_gripper2rbt_curr).as_quat()
        if np.dot(q_prev, q_curr) < 0:  
            q_curr = -q_curr
        q_delta = R.from_quat(q_curr) * R.from_quat(q_prev).inv()
        rotvec = q_delta.as_rotvec()
        d_orix, d_oriy, d_oriz = 10 * rotvec
        
        
        gripper_width = 1.0 if gripper_open_prev else -1.0
        
        collected_action = np.array([dx, dy, dz, d_orix, d_oriy, d_oriz, gripper_width])
        obs, reward, done, info = env.step(collected_action)

        R_gripper2rbt_prev = R_gripper2rbt_curr

        #press 'A' to restart env, press 'B' to close env
        if buttons_curr['A']:
            env.reset()

        if buttons_curr['B']:
            env.close()
        
        env.env.render()

    env.close()

    print("All finished")


if __name__ == '__main__':
    main()