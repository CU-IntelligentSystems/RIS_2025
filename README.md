# DuckieBot Red Line Detection Project

You are about to begin the process of retrieving the necessary information to program the DuckieBot to stop at a red line for 4 seconds and then proceed by moving forward. This README will guide you through setting up, building, and running the project on your system.

**Prerequisites:** Ensure you have `git` installed on your system.
Install the Duckietown Shell (`dts`) and Docker if they are not already installed. Refer to the [Duckietown Documentation](https://docs.duckietown.com/daffy/) for setup instructions. 


## Instructions:

### Step 1. Clone the Repository

Open your terminal and clone the GitHub repository using the following commands:
```bash
git clone https://github.com/Durranee-c/ris-project 
cd ris-project
```

### Step 2. Build the Project

Once you are in the ris-project directory, build the project using:
```bash
dts devel build -f
```

The -f flag (--force) allows the build to proceed even if the repository is not “clean,” i.e., after making local edits.
**Note:** This process may take some time.


### Step 3: Make Python Files Executable

Convert all the Python files into executables using the following commands:
```bash
chmod +x ./packages/my_package/src/red_line_detector.py
chmod +x ./packages/my_package/src/camera_reader_node.py
chmod +x ./packages/my_package/src/led_controller.py
chmod +x ./packages/my_package/src/wheel_controller.py
```

### Step 4: Rebuild the Project

After making the Python files executable, rebuild the project:
``` bash
dts devel build -f
```

### Step 5: Launch the DuckieBot

Finally, launch the launcher file using the following command:
```bash
dts devel run -R ROBOT_NAME -L duckiebot_launcher
```

Replace ROBOT_NAME with the name of the robot you are using (e.g. paul).

### Additional Notes
Ensure your DuckieBot is properly connected to your network and is visible when using the command:
dts fleet discover
on before launching.
If you encounter any issues during the build or execution process, refer to the [Duckietown Troubleshooting Guide](https://docs.duckietown.com/daffy/opmanual-duckiebot/intro.html) .
Contributions and feedback are welcome! 

By following these steps, you should be able to successfully program your DuckieBot to detect the red line, stop for 4 seconds, and resume movement. Enjoy experimenting with your robot!
