## Setup

### Build the Container

```sh
docker build 
```

### Running the Container

```sh
xhost +local:docker

docker compose run --rm ros2-gazebo
```

## Run the Simulation

Run these 4 commands each in a separate terminal:

**Terminal 1: Gazebo + bridge**

```sh
ros2 launch launch/sim.launch.py
```

**Terminal 2: ArduPilot SITL**

```sh
ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --add-param-file=gazebo/params/iris.parm --map --console
```

**Terminal 3: Visual servo**

```sh
docker compose exec -it ros2-gazebo bash
python3 scripts/visual_servo.py
```

**Terminal 4: rqt_image_view**

```sh
docker compose exec -it ros2-gazebo bash
ros2 run rqt_image_view rqt_image_view /ibvs/debug_image
```