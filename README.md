# running the code

1. build the dockercontainer in VSCode.
    - have the [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) extension installed
2. `catkin build`
3. `source ./devel/setup.bash`
4.  Run with
    - `roslaunch gaitnet_ros gaitnet.launch` for the actual service (once I finish it) or
    - `roslaunch gaitnet_ros dummyservice.launch` which just outputs a fixed gait (but with an identical interface)

# gaitnet-ros

Read the interface [readme](./src/gaitnet_interface/README.md).

In terms of actually implementing this on a real robot, we look at [Legged Perceptive](https://github.com/qiayuanl/legged_perceptive) as an example.

In this case there are a number of strategic changes which need to be made to the robot's control and visions systems.

## Control changes

The robot needs to be able to execute the perscribed actions returned from the GaitNetInterface service, necessitating a small number of changes.

- Somewhere in the code the MPC will be provided with the desired contact state, likley generated from a user-defined phase cycle. This will need to be replaced with the information provided by GaitNet. The implementation for the MPC in sim can be found [here](https://github.com/opsullivan85/rl-mpc-locomotion/blob/main/mpc/convex_MPC/Gait.py#L231C1-L231C6), but note that this may not match the MPC interface used in Legged Perceptive.
- The swing leg controller should be modified to follow the perscribed swing actions from GaitNet (position and duration of swing). Additionally, the swing should follow a curve  which peaks at 1/3 of the nominal body height of the robot-- 0.0867m in the case of the GO1.

## Vision changes

The only thing needed from the vision pipeline is the cspace of footstep positions for each leg. As is described in the interface [readme](./src/gaitnet_interface/README.md), this takes the form of a 25x25 boolean grid, where the center index is projected straight down from the hip joint (in the world frame), but with rotation from the body frame.

The conversion from height-map to cspace can be done many ways, but in training a simple height threshold was used-- any points significantly below the nominal walking plane are taken to be outside of the cspace.

## General changes

Through whatever means makes the most sense (probably just `rclcpp::WallTimer`), the GaitNet service should be called periodically, with its response being used to update the control system. 25Hz was used for evaluation, but the faster the better.