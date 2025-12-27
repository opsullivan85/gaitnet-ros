# GaitNet Interface

The following is the service interface for GaitNet. At a high level, it takes in

- **stepCSpace**-- a boolean mask determining where the robot can and cannot step.
- **state**-- the state of the robot

and outputs

- **action**-- either noop, or a discrete action for the robot to take at this time (i.e. move this foot here.)

```
gaitnet_interface/StepCSpaceRobot stepCSpace
  gaitnet_interface/StepCSpaceLeg FR
    gaitnet_interface/StepCSpaceRow[25] legCSpace
      bool[25] rowCSpace
  gaitnet_interface/StepCSpaceLeg FL
    gaitnet_interface/StepCSpaceRow[25] legCSpace
      bool[25] rowCSpace
  gaitnet_interface/StepCSpaceLeg RL
    gaitnet_interface/StepCSpaceRow[25] legCSpace
      bool[25] rowCSpace
  gaitnet_interface/StepCSpaceLeg RR
    gaitnet_interface/StepCSpaceRow[25] legCSpace
      bool[25] rowCSpace
gaitnet_interface/GaitNetInput state
  gaitnet_interface/FootPositions footPositions_xy_b
    float64[2] FR_xy_b
    float64[2] FL_xy_b
    float64[2] RL_xy_b
    float64[2] RR_xy_b
  float64 comHeight_z_w
  float64[3] linearVelocity_xyz_b
  float64[3] angularVelocity_xyz_b
  gaitnet_interface/ContactState contactState
    bool FR
    bool FL
    bool RL
    bool RR
  float64[3] projectedGravity_xyz_b
gaitnet_interface/ControlInput control
  float64[2] v_xy_b
  float64 yaw_b
---
gaitnet_interface/FootstepAction action
  int32 LEG_FR=0
  int32 LEG_FL=1
  int32 LEG_RL=2
  int32 LEG_RR=3
  bool noop
  int32 leg
  float64[2] hip_offset_xy
  float64 swing_duration
```

Some of the msg types are significantly verbose, but this is mostly done to keep the leg ordering consistent.

As for the stepCSpace, each leg entry should detail the cspace of possible footstep positions. This takes the form of a 25x25 grid (with a 0.015m cell-to-cell distance) with the middle cell projected straight down from the robot's hip joint. Be very careful to implement this exactly as is described here. For further reference:

- [Implementation in sim](https://github.com/opsullivan85/gaitnet/blob/main/src/simulation/cfg/footstep_scanner.py)
- [Isaac lab docs](https://isaac-sim.github.io/IsaacLab/main/source/api/lab/isaaclab.sensors.html#isaaclab.sensors.RayCasterCfg)

Other specific implementation details cand be found in the corresponding [thesis paper](https://digital.wpi.edu/concern/etds/hx11xk92h?).