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
