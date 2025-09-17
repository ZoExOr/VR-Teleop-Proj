# VR2Arm: Real-Time Teleoperation Across Simulation and Reality

## Overview
Collecting high-quality human demonstrations is crucial for advancing human-robot interaction, but existing tools like SpaceMouse and keyboard are unintuitive and limited to simple, slow-paced tasks (e.g., pick-and-place). 

We present a VR-based teleoperation pipeline for natural hand-to-robot control, validated in simulation (LIBERO, a novel benchmark for robot manipulation) and on a Franka Panda robot. Our system runs at 90 FPS (vs. ~20 FPS in prior work) and supports not only basic manipulation tasks but also demanding real-world dynamic scenarios such as flipping an object in a pan, demonstrating strong potential for advancing human-robot interaction research.

The project is developed at the **Social AI Lab, Vrije Universiteit Amsterdam (VU)**.



## File Structure
```
VR2Arm-Proj/
├── scripts/
│   ├── env_wrappers.py   # LIBERO Environment wrappers
│   ├── vr2franka.py      # Teleoperation script for Franka Panda
│   └── vr2libero.py      # Teleoperation script for LIBERO
├── README.md             # Project documentation
└── requirements.txt      # Python dependencies
```

## Setup the project
> Note: This setup has been tested in the Social AI Lab environment. Minor adjustments may be required depending on local system configurations.

### Hardware and System Requirements
- **Oculus Quest 3 controllers** (currently supports right-hand controller only)  
- **Ubuntu 20.04** (other platforms not yet tested)  
- **Python 3.8** recommended  

### 1. Clone the repository
```bash
git clone https://github.com/ZoExOr/VR2Arm-Proj.git
cd VR2Arm-Proj
```

### 2. (Optional) Create and activate a conda environment
```bash
conda create -n vr2arm python=3.8
conda activate vr2arm
```
> Note: This step is optional. The project is developed under **Python 3.8**, so using a dedicated conda environment is recommended for consistency.

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Install Oculus Reader

Setup instructions: https://github.com/rail-berkeley/oculus_reader
> Note: If you are part of the **Social AI Lab**, you do **not** need to download Meta Horizon, since the VR system there is already deployed. You only need to download **ADB**.

### 5. Install LIBERO

Follow the official LIBERO setup instructions: https://github.com/Lifelong-Robot-Learning/LIBERO
> Note: You only need to install the `libero` package. You do **not** need to download the demonstration dataset.

### 6. Install panda-py

Setup instructions: https://github.com/JeanElsner/panda-py
> Note: For compatibility with `libfranka`, it is recommended to download 'panda_py_0.7.5_libfranka_0.10.0.zip' from [its release page](https://github.com/JeanElsner/panda-py/releases) .
> After downloading and extracting, install the wheel with:
```
pip install panda_python-0.7.5+libfranka.0.10.0-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```



## Usage

1. Make sure all dependencies are installed.  
2. Connect your Oculus Quest 3 to the laptop via cable.  
3. For **simulation teleoperation**: align the Quest 3 orientation with the laptop.  
4. For **Franka teleoperation**: place the Quest 3 to the **left** of the laptop, with its cameras facing sideways.
5. Ensure the headset cameras have an unobstructed view of your controller during teleoperation. **Do not block the cameras**, for example by moving the controller under the desk.

   
**Illustration of teleoperation setup**  
   <img width="693" height="186" alt="示意图 drawio" src="https://github.com/user-attachments/assets/96a89b1a-8c2c-4ff3-a49b-b975b8892b50" />

6. Run the corresponding script for teleoperation.

