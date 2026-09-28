# Drone Pruner

## Setup

### 1. Clone

```sh
git clone --recursive https://github.com/iniberas/DronePruner.git
cd drone-pruner
```

### 2. Install ArduPilot prerequisites

Follow the official ArduPilot guide for your OS:

- [Linux / Ubuntu](https://ardupilot.org/dev/docs/building-setup-linux.html)
- [macOS](https://ardupilot.org/dev/docs/building-setup-mac.html)
- [Windows (WSL)](https://ardupilot.org/dev/docs/building-setup-windows11.html)

### 3. Install Webots

Download and install from [https://cyberbotics.com](https://cyberbotics.com)

### 4. Install PyTorch

Pick one:

```sh
# CPU only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# With GPU (CUDA)
pip install torch torchvision
```

## Run the Simulation

Run these 4 commands each in a separate terminal:

**Terminal 1: ArduPilot SITL**

```sh
sim/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --model webots-python --add-param-file=params/iris.parm --out=udp:127.0.0.1:14551 --console --map
```

**Terminal 2: Webots**

```sh
webots sim/worlds/sim.wbt
```

**Terminal 3: Drone controller**

```sh
python3 sim/ardupilot/libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py --camera "camera" --camera-port 5555 --rangefinder "range_finder" --rangefinder-port 5556
```

**Terminal 4: Visual servo**

```sh
python3 sim/scripts/visual_servo.py
```
