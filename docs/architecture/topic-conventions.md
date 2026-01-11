# ROS 2 Topic Conventions

Standard topic names and frame conventions for all robots in the robotlab platform.

## Core Time + Transforms

| Topic | Type | Publisher |
|-------|------|-----------|
| `/clock` | rosgraph_msgs/Clock | Simulator |
| `/tf` | tf2_msgs/TFMessage | All robots |
| `/tf_static` | tf2_msgs/TFMessage | Robot descriptions |

**Frame tree:** `map` → `odom` → `base_link` → ...

---

## Rolling Base

**Publishes:**
| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| `/odom` | nav_msgs/Odometry | 50 Hz | Wheel odometry |
| `/imu/data` | sensor_msgs/Imu | 100 Hz | IMU readings |
| `/joint_states` | sensor_msgs/JointState | 50 Hz | Wheel positions |

**Subscribes:**
| Topic | Type | Notes |
|-------|------|-------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |

**Cameras:**
| Topic | Type | Notes |
|-------|------|-------|
| `/camera/color/image_raw` | sensor_msgs/Image | RGB image |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | Intrinsics |
| `/camera/depth/image_raw` | sensor_msgs/Image | Depth image |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | Intrinsics |

**Camera frames:** `camera_link`, `camera_color_optical_frame`, `camera_depth_optical_frame`

---

## Manipulation (ros2_control + MoveIt)

**Actions:**
| Action | Type | Notes |
|--------|------|-------|
| `/arm_controller/follow_joint_trajectory` | control_msgs/FollowJointTrajectory | Trajectory execution |

**Topics:**
| Topic | Type | Notes |
|-------|------|-------|
| `/gripper_controller/command` | std_msgs/Float64 | Gripper position |

**Frame tree:** `base_link` → `arm_base_link` → ... → `tool0` → `gripper_link`

---

## Drone (via uXRCE-DDS)

**From PX4:**
| Topic | Type | Notes |
|-------|------|-------|
| Vehicle odometry | px4_msgs/VehicleOdometry | State estimate |
| Vehicle status | px4_msgs/VehicleStatus | Flight mode |

**To PX4:**
| Topic | Type | Rate | Notes |
|-------|------|------|-------|
| OffboardControlMode | px4_msgs/OffboardControlMode | ≥2 Hz | Heartbeat |
| TrajectorySetpoint | px4_msgs/TrajectorySetpoint | ≥2 Hz | Desired position/velocity |

**Frame warning:** PX4 uses NED, ROS uses ENU. Create a single `frame_bridge` node for conversions. Nothing else does ad-hoc frame math.

---

## Capability API (Stable Across Robots)

High-level actions that work identically regardless of robot type:

| Action | Type | Notes |
|--------|------|-------|
| `/capabilities/navigate_to_pose` | nav2_msgs/NavigateToPose | Point-to-point navigation |
| `/capabilities/align_to_object` | robotlab_msgs/AlignToObject | Visual servoing |
| `/capabilities/pick` | robotlab_msgs/Pick | Grasp object |
| `/capabilities/place` | robotlab_msgs/Place | Release object |

**Topics:**
| Topic | Type | Notes |
|-------|------|-------|
| `/perception/object_pose` | geometry_msgs/PoseStamped | Latest detected object |

---

## Naming Rules

1. **Namespace by robot ID** when running multiple robots: `/robot1/cmd_vel`
2. **Use standard message types** where possible (geometry_msgs, sensor_msgs, nav_msgs)
3. **Custom messages** go in `robotlab_msgs` package
4. **Frame IDs** match the robot URDF
5. **All time-sensitive nodes** use `use_sim_time:=true` parameter
