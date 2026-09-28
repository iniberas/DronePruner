# Drone Pruner

Simulasi drone pruner menggunakan ArduPilot SITL + Webots.

## Install

Install PyTorch, pilih salah satu:

```sh
# CPU only
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

# Dengan GPU (CUDA)
pip install torch torchvision
```

## Menjalankan Simulasi

Jalankan 4 perintah berikut secara berurutan, masing-masing di terminal yang berbeda.

**Terminal 1: ArduPilot SITL**

```sh
sim/ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --model webots-python --add-param-file=params/iris.parm --out=udp:127.0.0.1:14551 --console --map
```

**Terminal 2: Webots**

```sh
webots sim/worlds/sim.wbt
```

**Terminal 3: Controller drone**

```sh
python3 sim/ardupilot/libraries/SITL/examples/Webots_Python/controllers/ardupilot_vehicle_controller/ardupilot_vehicle_controller.py --camera "camera" --camera-port 5555 --rangefinder "range_finder" --rangefinder-port 5556
```

**Terminal 4: Visual servo**

```sh
python3 sim/scripts/visual_servo.py
```