# Spot Fetch
A program to get Boston Dynamics' Spot robot to fetch a tennis ball. Works with a Stereolabs ZED 2i camera (but any ZED would likely work), a Spot robot, and a NVIDIA Jetson (in this case, a Jetson AGX Xavier). The tennis ball should be greenish-yellow, although if you want to use a different color tennis ball, you can modify `src/spot_fetch/src/tennisball.py` (the vision pipeline) and `src/spot_fetch/cfg/TennisBall.cfg` (its config values).

## Setup/Dependencies
**Dependencies:**
- [ZED SDK](https://www.stereolabs.com/developers/release/), which requires CUDA, which requires an NVIDIA GPU
- [ZED](https://www.stereolabs.com/zed-2i/)
- [Spot](https://www.bostondynamics.com/products/spot)
![A picture of Spot](https://github.com/clearpathrobotics/spot_ros/blob/master/cp_spot.jpg?raw=true)
- Some type of onboard computer with a GPU (we are using an [NVIDIA Jetson AGX Xavier](https://developer.nvidia.com/embedded/jetson-agx-xavier-developer-kit))
- A tennis ball (this is the most important, obviously)
- Velcro to attach to Spot if you want it to be able to pick up the ball

**Setup:**
- `git submodule init`, `git submodule update`
- To build: `./build.bash`. The script should automatically detect your architecture -- if not, it will ask you if you're using x86_64 or aarch64.
- Add your Spot's username, password, and IP address to `src/spot_ros/spot_driver/launch/driver.launch` 
- Set `auto_claim` and `auto_power_on` to `True` in `src/spot_ros/spot_driver/config/spot_ros.yaml`
- Set `quality` to `4` in `src/zed-ros-wrapper/zed_wrapper/params/common.yaml`

## Usage
- Build code (`./build.bash`)
    - If `tf2_bullet` fails to build, it's safe to delete it or ignore it
    - If the ZED code fails to build, install the [ZED SDK](https://www.stereolabs.com/developers/release/)
- Source the workspace (`. devel/setup.bash`)
- Start the Spot driver (in the current configuration, Spot's motors will power on but it will not stand up yet): `roslaunch spot_driver driver.launch` (needs to run in a separate terminal)
- Start the ZED and the fetch program: `roslaunch spot_fetch fetch.launch` (needs to run in a separate terminal)
- Tell Spot to stand up: `rosservice call /spot/stand`
- Tell Spot to walk to a tennis ball: `rosservice call /fetch/fetch_tennis_ball` Before doing this, it's good to look at the logs of the fetch program to confirm Spot sees the ball before telling Spot to fetch it. For reference, +x = forward, +y = left, +z = up (relative to Spot's direction).
- Once Spot is above the ball: `rosservice call /spot/sit` (we attached velcro to Spot so when it sits down, it will pick up the ball)

## TODOs / future improvements
- Spot is the most accurate when facing the ball. Modify the program to have Spot face towards the ball before walking to it.
- When the fetch service is called, have Spot turn around until it sees a ball. Once a ball is found, it should be fetched.
- Train an object detection model to detect the ball (currently using OpenCV which works, but object detection would be more reliable)
- Get the ZED and Spot in the same tf tree so you can transform from the ZED to the Spot's body (instead of subtracting offsets)

https://user-images.githubusercontent.com/67695217/175784390-49cb8af3-d215-4c48-99dc-18d4d66876af.MOV

# Have fun!
