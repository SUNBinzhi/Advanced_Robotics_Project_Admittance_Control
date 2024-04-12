# Mujoco_Admittance_Control_TM5-900
This is a repository for simulation of robot arms admittance controller in mujoco environment

# Mujoco Library Installation Guide

## Steps for Installation

### 1. Download the Mujoco Library

You can download the Mujoco library from the following URL:
```
https://github.com/google-deepmind/mujoco/releases
```

### 2. Create a Hidden Directory

In Linux systems, files or directories starting with a dot (.) are hidden. To create a hidden directory for Mujoco, use:
```
mkdir ~/.mujoco
```

Note: To view hidden files or directories, use the shortcut `ctrl+h`.

### 3. Extract the Mujoco Library

Extract the contents of the Mujoco library `mujoco-3.0.0-linux-x86_64.tar.gz` into the hidden directory `.mujoco` you created. You can do this by right-clicking the downloaded file and selecting "extract to" the hidden directory. If the directory is not visible, press `ctrl+h` to reveal it.![Alt text](TM5-900/Mujoco_Test/pictures/File_Location.png)

### 4. Update the .bashrc File

Add the following lines to your `.bashrc` file, which is located in `/home/your_username/`. Remember to replace `your_username` with your actual username. You can open the `.bashrc` file with:
```
sudo gedit ~/.bashrc
```

Then add these lines at the end of the file (again, replace `user_name` with your actual username):
```
export LD_LIBRARY_PATH=/home/user_name/.mujoco/mujoco-3.0.0-linux-x86_64/mujoco-3.0.0/bin
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/nvidia
export PATH="$LD_LIBRARY_PATH:$PATH"
export LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libGLEW.so
```

Save and exit the file. Then, apply the changes by executing:
```
source ~/.bashrc
```

### 5. Pip Install Mujoco Library to your Python environment

Using this command:
```
pip install mujoco
```

### 6. Test the Installation

To test if the installation was successful, create a Python script and run:
```
import mujoco
from mujoco import viewer

xml = """
<mujoco>
  <worldbody>
    <geom name="red_box" type="box" size=".2 .2 .2" rgba="1 0 0 1"/>
    <geom name="green_sphere" pos=".2 .2 .2" size=".1" rgba="0 1 0 1"/>
  </worldbody>
</mujoco>
"""
m = mujoco.MjModel.from_xml_string(xml)
d = mujoco.MjData(m)

viewer.launch(m)

```

If everything is set up correctly, you should see the program running successfully.![Alt text](TM5-900/Mujoco_Test/pictures/test.png)

---

**Note:** The guide assumes you are familiar with basic Linux commands and operations.

---





