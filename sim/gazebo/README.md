## Setup

### Build the Container

```sh
docker compose build 
```

### Running the Container

```sh
xhost +local:docker

docker compose up -d
```

## Run the Simulation

Run these 4 commands each in a separate terminal:

**Terminal 1: Gazebo + bridge**

```sh
docker compose exec -it ros2-gazebo bash
ros2 launch launch/sim.launch.py
# klo mau yang apel
ros2 launch launch/sim.launch.py world_file:=sim_apple.sdf
```

**Terminal 2: ArduPilot SITL**

```sh
ardupilot/Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --add-param-file=gazebo/params/iris.parm --map --console
```

**Terminal 3: Visual servo**

```sh
docker compose exec -it ros2-gazebo bash
python3 scripts/visual_servo.py
# kalo mau yang apel
python3 scripts/visual_servo.py -p weights_path:=/opt/weights/yolov5s.pt target_class_id:=47
```