# OmniGraph Node Reference

Catalog of common nodes for ROS 2 robotics in Isaac Sim 5.1.

## Node Type Path Format

Node types follow this pattern:
```
{extension}.{category}.{NodeName}
```

Examples:
- `omni.graph.action.OnPlaybackTick`
- `isaacsim.ros2.bridge.ROS2PublishClock`
- `isaacsim.core.nodes.IsaacArticulationController`

## Trigger Nodes

### OnPlaybackTick
**Type:** `omni.graph.action.OnPlaybackTick`

Fires every simulation frame when Play is active.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `outputs:tick` | output | execution | Trigger signal |
| `outputs:deltaSeconds` | output | double | Time since last tick |
| `outputs:frame` | output | int | Current frame number |

### OnImpulseEvent
**Type:** `omni.graph.action.OnImpulseEvent`

Manual trigger for on-demand execution.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `state:enableImpulse` | state | bool | Set `True` to trigger |
| `outputs:execOut` | output | execution | Trigger signal |

## Core Nodes

### IsaacReadSimulationTime
**Type:** `isaacsim.core.nodes.IsaacReadSimulationTime`

Reads current simulation time.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `outputs:simulationTime` | output | double | Current sim time (seconds) |

### IsaacArticulationController
**Type:** `isaacsim.core.nodes.IsaacArticulationController`

Sends commands to robot joints via physics articulation.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:robotPath` | input | string | USD path to articulation root |
| `inputs:jointNames` | input | token[] | Joint names to control |
| `inputs:positionCommand` | input | double[] | Target positions (rad) |
| `inputs:velocityCommand` | input | double[] | Target velocities (rad/s) |
| `inputs:effortCommand` | input | double[] | Target efforts (N*m) |

**Note:** Set `robotPath` to the prim with the ArticulationRoot component.

## ROS 2 Context

### ROS2Context
**Type:** `isaacsim.ros2.bridge.ROS2Context`

Creates ROS 2 domain context for publishing/subscribing.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:domain_id` | input | int | ROS 2 domain ID (default: 0) |
| `inputs:useDomainIDEnvVar` | input | bool | Use `ROS_DOMAIN_ID` env var |
| `outputs:context` | output | uint64 | Context handle |

**Note:** Most ROS 2 nodes have an optional `inputs:context` port. If not connected, they use domain 0.

## ROS 2 Publishers

### ROS2PublishClock
**Type:** `isaacsim.ros2.bridge.ROS2PublishClock`

Publishes `rosgraph_msgs/Clock` to `/clock`.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:timeStamp` | input | double | Simulation time |
| `inputs:topicName` | input | string | Topic name (default: `/clock`) |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

### ROS2PublishJointState
**Type:** `isaacsim.ros2.bridge.ROS2PublishJointState`

Publishes `sensor_msgs/JointState`.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:targetPrim` | input | target | Robot articulation prim |
| `inputs:topicName` | input | string | Topic name (default: `/joint_states`) |
| `inputs:timeStamp` | input | double | Message timestamp |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

### ROS2PublishOdometry
**Type:** `isaacsim.ros2.bridge.ROS2PublishOdometry`

Publishes `nav_msgs/Odometry`.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:topicName` | input | string | Topic name |
| `inputs:chassisPrim` | input | target | Robot chassis prim |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

### ROS2PublishTransformTree
**Type:** `isaacsim.ros2.bridge.ROS2PublishTransformTree`

Publishes `tf2_msgs/TFMessage` to `/tf`.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:targetPrims` | input | target[] | Prims to publish transforms for |
| `inputs:topicName` | input | string | Topic name (default: `/tf`) |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

## ROS 2 Subscribers

### ROS2SubscribeTwist
**Type:** `isaacsim.ros2.bridge.ROS2SubscribeTwist`

Subscribes to `geometry_msgs/Twist`.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:topicName` | input | string | Topic name (default: `/cmd_vel`) |
| `inputs:context` | input | uint64 | Optional ROS 2 context |
| `outputs:execOut` | output | execution | Fires when message received |
| `outputs:linearVelocity` | output | vector3d | Linear velocity (x, y, z) |
| `outputs:angularVelocity` | output | vector3d | Angular velocity (x, y, z) |

### ROS2SubscribeJointState
**Type:** `isaacsim.ros2.bridge.ROS2SubscribeJointState`

Subscribes to `sensor_msgs/JointState`.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:topicName` | input | string | Topic name (default: `/joint_command`) |
| `inputs:context` | input | uint64 | Optional ROS 2 context |
| `outputs:execOut` | output | execution | Fires when message received |
| `outputs:jointNames` | output | token[] | Joint names from message |
| `outputs:positionCommand` | output | double[] | Position values |
| `outputs:velocityCommand` | output | double[] | Velocity values |
| `outputs:effortCommand` | output | double[] | Effort values |

## Wheeled Robot Controllers

**Requirement:** Enable `isaacsim.robot.wheeled_robots` extension.

### DifferentialController
**Type:** `isaacsim.robot.wheeled_robots.DifferentialController`

Converts linear/angular velocity to left/right wheel velocities.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:linearVelocity` | input | double | Desired forward speed (m/s) |
| `inputs:angularVelocity` | input | double | Desired turn rate (rad/s) |
| `inputs:wheelRadius` | input | double | Wheel radius (m) |
| `inputs:wheelDistance` | input | double | Wheel separation (m) |
| `inputs:maxLinearSpeed` | input | double | Max forward speed (m/s) |
| `inputs:maxAngularSpeed` | input | double | Max turn rate (rad/s) |
| `inputs:maxWheelSpeed` | input | double | Max wheel speed (rad/s) |
| `outputs:velocityCommand` | output | double[] | [left_vel, right_vel] (rad/s) |

**Common values:**
- Jetbot: `wheelRadius=0.0325`, `wheelDistance=0.1125`
- TurtleBot3: `wheelRadius=0.025`, `wheelDistance=0.16`

### AckermannController
**Type:** `isaacsim.robot.wheeled_robots.AckermannController`

Converts speed/steering to individual wheel commands for car-like robots.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:linearVelocity` | input | double | Desired speed (m/s) |
| `inputs:angularVelocity` | input | double | Desired steering rate |
| `inputs:wheelBase` | input | double | Front-to-rear axle distance (m) |
| `inputs:trackWidth` | input | double | Left-to-right wheel distance (m) |
| `inputs:frontWheelRadius` | input | double | Front wheel radius (m) |
| `inputs:backWheelRadius` | input | double | Rear wheel radius (m) |
| `inputs:maxWheelRotation` | input | double | Max steering angle (rad) |
| `inputs:maxWheelVelocity` | input | double | Max wheel speed (rad/s) |
| `inputs:maxAcceleration` | input | double | Max accel (m/s^2) |
| `inputs:maxSteeringAngleVelocity` | input | double | Steering rate limit (rad/s) |
| `outputs:frontLeftSteeringAngle` | output | double | Front-left steering angle |
| `outputs:frontRightSteeringAngle` | output | double | Front-right steering angle |
| `outputs:rearLeftWheelVelocity` | output | double | Rear-left wheel speed |
| `outputs:rearRightWheelVelocity` | output | double | Rear-right wheel speed |
| `outputs:frontLeftWheelVelocity` | output | double | Front-left wheel speed |
| `outputs:frontRightWheelVelocity` | output | double | Front-right wheel speed |

**Common values (Leatherback):**
- `wheelBase=0.32`, `trackWidth=0.24`
- `frontWheelRadius=0.052`, `backWheelRadius=0.052`
- `maxWheelRotation=0.7854` (45 degrees)

### HolonomicController
**Type:** `isaacsim.robot.wheeled_robots.HolonomicController`

For omni-directional robots (e.g., mecanum wheels).

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:linearVelocityX` | input | double | Desired X velocity (m/s) |
| `inputs:linearVelocityY` | input | double | Desired Y velocity (m/s) |
| `inputs:angularVelocity` | input | double | Desired rotation (rad/s) |
| `inputs:wheelRadius` | input | double[] | Array of wheel radii |
| `inputs:wheelPositions` | input | vector3d[] | Wheel positions from center |
| `inputs:mecanumAngles` | input | double[] | Mecanum wheel angles (deg) |
| `outputs:jointVelocityCommand` | output | double[] | Wheel velocities (rad/s) |

## Utility Nodes

### BreakVector3 / Break3Vector
**Type:** `omni.graph.nodes.BreakVector3`

Splits a 3D vector into components.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:tuple` | input | vector3d | Input vector |
| `outputs:x` | output | double | X component |
| `outputs:y` | output | double | Y component |
| `outputs:z` | output | double | Z component |

**Use case:** Extract linear.x and angular.z from Twist subscriber.

### MakeArray
**Type:** `omni.graph.nodes.MakeArray`

Creates an array from individual values.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:arraySize` | input | int | Number of elements |
| `inputs:input0` | input | any | First element |
| `inputs:input1` | input | any | Second element |
| `outputs:array` | output | any[] | Combined array |

**Use case:** Combine joint names for ArticulationController.

### ConstantToken
**Type:** `omni.graph.nodes.ConstantToken`

Provides a constant token (string-like) value.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:value` | input | token | Token value |
| `outputs:value` | output | token | Same token value |

**Use case:** Provide joint names like `wheel_left_joint`.

### ConstantString
**Type:** `omni.graph.nodes.ConstantString`

Provides a constant string value.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:value` | input | string | String value |
| `outputs:value` | output | string | Same string value |

## Camera Nodes

### IsaacCreateRenderProduct
**Type:** `isaacsim.core.nodes.IsaacCreateRenderProduct`

Creates render product from camera for image publishing.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:cameraPrim` | input | target | Camera prim path |
| `inputs:width` | input | int | Image width |
| `inputs:height` | input | int | Image height |
| `outputs:renderProductPath` | output | string | Render product path |

### ROS2CameraHelper
**Type:** `isaacsim.ros2.bridge.ROS2CameraHelper`

Publishes camera data (RGB, depth, semantic, etc.).

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | execution | Trigger |
| `inputs:renderProductPath` | input | string | From IsaacCreateRenderProduct |
| `inputs:type` | input | token | `rgb`, `depth`, `semantic_segmentation`, etc. |
| `inputs:topicName` | input | string | Topic name |
| `inputs:frameId` | input | string | TF frame ID |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

**Supported types:**
- `rgb` - Color image
- `depth` - Depth image
- `camera_info` - Camera intrinsics
- `semantic_segmentation` - Labeled masks
- `instance_segmentation` - Instance masks
- `pointcloud` - 3D point cloud

## Finding Node Types

In Isaac Sim GUI:
1. Window > Visual Scripting > Action Graph
2. Right-click in graph canvas
3. Search for node name
4. Hover to see full type path

Or use Python:
```python
import omni.graph.core as og
# List all registered node types
for node_type in og.get_registered_nodes():
    if "ros2" in node_type.lower():
        print(node_type)
```

## References

- [ROS 2 Joint Control Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [TurtleBot Driving](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html)
- [Mobile Robot Controllers](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/mobile_robot_controllers.html)
- [Ackermann Controller Tutorial](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/ros2_tutorials/tutorial_ros2_ackermann_controller.html)
