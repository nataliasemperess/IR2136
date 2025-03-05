## TERMINAL 1

1) Clone the Ardupilot repository : 
```bash
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot
git submodule update --init --recursive

```

2) Install dependencies : Run the installation script for Ubuntu : 

```bash
./Tools/environment_install/install-prereqs-ubuntu.sh -y
```

3) Source the configurations :

```bash
. ~/.profile
```

4) Compile SITL: Navigate to the Ardupilot folder and compile:

```bash
cd ~/ardupilot/ArduCopter
sim_vehicle.py -v ArduCopter -w --console --map
```
## TERMINAL 2 - battery_gps_node.py

```bash
source /opt/ros/humble/setup.bash
cd ~/Documentos/GitHub/IR2136/lab03/ros2_ws
colcon build
source install/setup.bash
ros2 run lab3 battery_gps 

```

## TERMINAL 3 - mission_control_node.py

```bash
source /opt/ros/humble/setup.bash
cd ~/Documentos/GitHub/IR2136/lab03/ros2_ws
colcon build
source install/setup.bash
ros2 run lab3 mission_control -- -35.36206693 149.156797 10




```
```
