# Getting Started

By the end of this tutorial, you will have the autonomy stack running on your machine.

## Requirements

You need at least 25GB free to install the Docker image.

Have an NVIDIA GPU >= RTX 3070 to run Isaac Sim locally.

## Setup

### Clone
```
git clone --recursive -j8 git@github.com:castacks/AirStack.git
```
### Docker
Install [Docker Desktop](https://docs.docker.com/desktop/install/ubuntu/). This should come installed with docker compose.

## Configure 

Follow the instructions in `docker/isaac-sim/omni_pass.env` to configure the required settings for your Omniverse Nucelus Server token.
To generate a token, follow the NVIDIA docs [here](https://docs.omniverse.nvidia.com/nucleus/latest/config-and-info/api_tokens.html). See here for more information:
 https://airlab.slite.com/app/docs/X8dZ8w5S3GP9tw

Also set the default OMNI_SERVER and accept the license terms. (Basti: The omni_server variable doesn't seem to work. The content browser might have to be edited manually the first time. To do that click:
"Add new connection ..." and enter airlab-storage.andrew.cmu.edu:8443 in the server field. Also if there is a localhost it should be removed since we are not running a local Nucleus server.

## Getting Docker Images Ready
Now you have two options on how to proceed. You can build the docker image from scratch or pull the existing image on the airlab docker registry. Building the image from scratch can be  useful if you would like to add new dependencies or add new custom functionality. For most users just pulling the existing image will be more conveninent and fast since it doesn't require access to the Nvidia registry.

### Option 1 (Preferred): Use the Airlab Docker registry

To use the AirLab docker registry do the following
```bash
cd AirStack/docker/
docker login airlab-storage.andrew.cmu.edu:5001
## <Enter your andrew id (without @andrew.cmu.edu)>
## <Enter your andrew password>

## Pull the images in the docker compose file
docker compose pull 
```
When you execute docker compose pull in the next step the image will be pulled from the server automatically. This might take a while since the image is large.


#### Option 2: Setup from Scratch
1. SITL (Required until we add to docker image)

    Download the Ascent Spirit SITL software packages from [this link](https://drive.google.com/file/d/1UxgezaTrHe4WJ28zsVeRhv1VYfOU5VK8/view?usp=drive_link).

    Then unzip the file AscentAeroSystemsSITLPackage.zip in this folder:
    ```
    cd AirStack/docker/isaac-sim/
    unzip ~/Downloads/AscentAeroSystemsSITLPackage.zip -d .
    ```

2. Gain access to NVIDIA NGC Containers by following [these instructions](https://docs.nvidia.com/launchpad/ai/base-command-coe/latest/bc-coe-docker-basics-step-02.html).

    Then:
    ```bash
    cd AirStack/docker/
    docker compose build  # build the images locally
    ```

IF you have permission you can now push an updated images to the docker server (only if it changed and is required)
```bash
docker compose push
```

## Launch

### Initialize Docker Containers
```bash
xhost +  # allow docker access to X-Server

# Make sure you are in the AirStack/docker directory.

# Start docker compose services,
#  you can append `--scale robot=[NUM_ROBOTS]` for more robots, default is 1
docker compose up -d
# view running containers
docker ps -a
```

### Launch Isaac Sim:

```bash
# in another terminal under AirStack/docker
docker exec -it isaac-sim bash
# within isaac-sim docker
runapp
```
In the Isaac content browser, under "Omniverse", click "Add New Connection ...".
Type in `airlab-storage.andrew.cmu.edu:8443`.

It takes a few seconds to connect.

Then open the stage from the Nucleus server:
`airlab-storage.andrew.cmu.edu:8443/Projects/AirStack/ascent_fire_academy.usd`


### Launch Robot:

```bash

# create a bash shell in the docker-robot-1 container
docker exec -it docker-robot-1 bash

# in robot docker
bws && sws  # build workspace and source workspace. these are aliases in ~/.bashrc
ros2 launch robot_bringup robot.launch.xml
```

## Move Robot


Once Isaac Sim launches, hit the gray triangle "Play" button on the left side bar. It takes a minute for the GPS to initialize.

Then, find the RQT GUI window. Hit `Takeoff`, then hit `Publish` in the trajectory window like in this video:

<iframe width="840" height="473" src="https://www.youtube.com/embed/kfP5-ZbIBkc?si=knDAR4-CnLkxlGNb" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

Note you can also use the `ros2 topic pub` command to move the robot. For example, to fly to a position:

```bash
# start another terminal in docker container
docker exec -it docker-robot-1 bash
# in docker
# FLY TO POSITION. Put whatever position you want
ros2 topic pub /robot_1/interface/mavros/setpoint_position/local geometry_msgs/PoseStamped \
    "{ header: { stamp: { sec: 0, nanosec: 0 }, frame_id: 'base_link' }, \
    pose: { position: { x: 10.0, y: 0.0, z: 20.0 }, orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 } } }" -1
```

## Shutdown

To pause containers:

```bash
docker compose stop
```

To shutdown and remove docker containers:

```bash
docker compose down
```