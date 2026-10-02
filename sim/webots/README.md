## Setup

Download and install webots from [https://cyberbotics.com](https://cyberbotics.com)

## Run the Simulation

Run these 4 commands each in a separate terminal:

**Terminal 1: ArduPilot SITL**

```sh
ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --model webots-python --add-param-file=params/iris.parm --out=udp:127.0.0.1:14551 --console --map
```

**Terminal 2: Webots**

```sh
webots worlds/sim.wbt
```

**Terminal 3: Drone controller**

```sh
python3 ardupilot/libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py --camera "camera" --camera-port 5555 --rangefinder "range_finder" --rangefinder-port 5556
```

**Terminal 4: Visual servo**

```sh
python3 scripts/visual_servo.py
```
