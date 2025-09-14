# VR-Teleop-Proj

## Overview
This is a **VR-based teleoperation framework** for robotic manipulation, developed at the **Social AI Lab, Vrije Universiteit Amsterdam (VU)**.  

It enables teleoperation in both **LIBERO** simulation environments and on a **Franka Emika Panda** robot, running in real time at **90 FPS**.  

- Uses **Oculus Quest 3 controllers**, currently supporting the right-hand controller only  
- Verified on **Ubuntu 20.04** (other platforms not yet tested)  

## File Structure
```
VR-Teleop-Proj/
├── scripts/
│   ├── env_wrappers.py   # LIBERO Environment wrappers
│   ├── vr2franka.py      # Teleoperation script for Franka Panda
│   └── vr2libero.py      # Teleoperation script for LIBERO
├── README.md             # Project documentation
└── requirements.txt      # Python dependencies
```

## Setup the project

### 1. Clone the repository
```bash
git clone https://github.com/ZoExOr/VR-Teleop-Proj.git
cd VR-Teleop-Proj
```

### 2. Create and activate a conda environment
```bash
conda create -n vrproj python=3.8
conda activate vrproj
```

### 3. Install dependencies
```bash
pip install -r requirements.txt
```

### 4. Install Oculus Reader

Setup instructions: https://github.com/rail-berkeley/oculus_reader
> Note: If you are part of the **Social AI Lab**, you do **not** need to download Meta Horizon, since the VR system there is already deployed. You only need to download **ADB**.

### 5. Install LIBERO

Follow the official LIBERO setup instructions: https://github.com/Lifelong-Robot-Learning/LIBERO

### 6. Install panda-py

Setup instructions: https://github.com/JeanElsner/panda-py

## Usage

1. Make sure all dependencies are installed.  
2. Connect your Oculus Quest 3 to the laptop via cable.  
3. For **simulation teleoperation**: keep the Quest 3 oriented in the same direction as the laptop.  
4. For **Franka teleoperation**: place the Quest 3 on the **left side** of the laptop, with its cameras facing the left side of the laptop.

   
**Illustration of teleoperation setup**  
   <img width="693" height="186" alt="示意图 drawio" src="https://github.com/user-attachments/assets/96a89b1a-8c2c-4ff3-a49b-b975b8892b50" />

5. Run the corresponding script for teleoperation.

