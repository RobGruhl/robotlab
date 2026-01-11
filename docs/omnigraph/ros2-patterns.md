# ROS 2 OmniGraph Patterns

Complete, copy-paste-ready patterns for common robot configurations.

## Pattern A: Differential Drive (/cmd_vel)

For two-wheeled robots (Jetbot, TurtleBot3, etc.) controlled via Twist messages.

### Data Flow
```
/cmd_vel (Twist) → SubscribeTwist → Break3Vector → DifferentialController → ArticulationController → Robot Joints
```

### Complete Code

```python
import omni.graph.core as og

def create_differential_drive_graph(
    robot_path: str = "/World/jetbot",
    wheel_radius: float = 0.0325,
    wheel_distance: float = 0.1125,
    topic_name: str = "/cmd_vel",
):
    """Create action graph for differential drive robot."""

    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("break_linear", "omni.graph.nodes.BreakVector3"),
                ("break_angular", "omni.graph.nodes.BreakVector3"),
                ("diff_ctrl", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("art_ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # ROS 2 config
                ("context.inputs:domain_id", 0),
                ("subscribe_twist.inputs:topicName", topic_name),
                # Differential controller config
                ("diff_ctrl.inputs:wheelRadius", wheel_radius),
                ("diff_ctrl.inputs:wheelDistance", wheel_distance),
                ("diff_ctrl.inputs:maxLinearSpeed", 1.0),
                ("diff_ctrl.inputs:maxAngularSpeed", 2.0),
                # Articulation controller config
                ("art_ctrl.inputs:robotPath", robot_path),
            ],
            og.Controller.Keys.CONNECT: [
                # Tick triggers
                ("tick.outputs:tick", "subscribe_twist.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "diff_ctrl.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "art_ctrl.inputs:execIn"),
                # Context
                ("context.outputs:context", "subscribe_twist.inputs:context"),
                # Twist decomposition
                ("subscribe_twist.outputs:linearVelocity", "break_linear.inputs:tuple"),
                ("subscribe_twist.outputs:angularVelocity", "break_angular.inputs:tuple"),
                # To differential controller (linear.x, angular.z)
                ("break_linear.outputs:x", "diff_ctrl.inputs:linearVelocity"),
                ("break_angular.outputs:z", "diff_ctrl.inputs:angularVelocity"),
                # To articulation controller
                ("diff_ctrl.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
            ],
        },
    )

# Usage
create_differential_drive_graph(
    robot_path="/World/jetbot",
    wheel_radius=0.0325,
    wheel_distance=0.1125,
)
```

### Robot-Specific Parameters

| Robot | wheel_radius | wheel_distance |
|-------|--------------|----------------|
| Jetbot | 0.0325 | 0.1125 |
| TurtleBot3 Burger | 0.025 | 0.16 |
| TurtleBot3 Waffle | 0.033 | 0.287 |

### Test Commands

```bash
# Forward at 0.3 m/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10

# Spin left at 1.0 rad/s
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 1.0}}" --rate 10

# Arc (forward + turn)
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.5}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

---

## Pattern B: Ackermann Steering

For car-like robots (LIMO, Leatherback) with front-wheel steering.

### Data Flow
```
/cmd_vel (Twist) → SubscribeTwist → AckermannController → ArticulationController (steering + wheels)
```

### Complete Code

```python
import omni.graph.core as og

def create_ackermann_drive_graph(
    robot_path: str = "/World/limo",
    wheel_base: float = 0.32,
    track_width: float = 0.24,
    wheel_radius: float = 0.052,
    topic_name: str = "/cmd_vel",
):
    """Create action graph for Ackermann steering robot."""

    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("break_linear", "omni.graph.nodes.BreakVector3"),
                ("break_angular", "omni.graph.nodes.BreakVector3"),
                ("ackermann_ctrl", "isaacsim.robot.wheeled_robots.AckermannController"),
                ("art_ctrl_steering", "isaacsim.core.nodes.IsaacArticulationController"),
                ("art_ctrl_wheels", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # ROS 2 config
                ("context.inputs:domain_id", 0),
                ("subscribe_twist.inputs:topicName", topic_name),
                # Ackermann controller config
                ("ackermann_ctrl.inputs:wheelBase", wheel_base),
                ("ackermann_ctrl.inputs:trackWidth", track_width),
                ("ackermann_ctrl.inputs:frontWheelRadius", wheel_radius),
                ("ackermann_ctrl.inputs:backWheelRadius", wheel_radius),
                ("ackermann_ctrl.inputs:maxWheelRotation", 0.7854),  # 45 degrees
                ("ackermann_ctrl.inputs:maxWheelVelocity", 20.0),
                ("ackermann_ctrl.inputs:maxAcceleration", 1.0),
                ("ackermann_ctrl.inputs:maxSteeringAngleVelocity", 1.0),
                # Articulation controllers
                ("art_ctrl_steering.inputs:robotPath", robot_path),
                ("art_ctrl_wheels.inputs:robotPath", robot_path),
            ],
            og.Controller.Keys.CONNECT: [
                # Tick triggers
                ("tick.outputs:tick", "subscribe_twist.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "ackermann_ctrl.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "art_ctrl_steering.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "art_ctrl_wheels.inputs:execIn"),
                # Context
                ("context.outputs:context", "subscribe_twist.inputs:context"),
                # Twist decomposition
                ("subscribe_twist.outputs:linearVelocity", "break_linear.inputs:tuple"),
                ("subscribe_twist.outputs:angularVelocity", "break_angular.inputs:tuple"),
                # To Ackermann controller
                ("break_linear.outputs:x", "ackermann_ctrl.inputs:linearVelocity"),
                ("break_angular.outputs:z", "ackermann_ctrl.inputs:angularVelocity"),
                # Steering outputs to articulation (front wheels)
                # Note: Requires setting up joint name arrays for steering joints
                # Wheel velocity outputs to articulation (all wheels)
            ],
        },
    )

    # Note: Complete wiring requires MakeArray nodes for joint names
    # and connecting individual wheel velocity outputs
    print("Ackermann graph created. Manual wiring may be needed for joint arrays.")

# Usage for LIMO
create_ackermann_drive_graph(
    robot_path="/World/limo",
    wheel_base=0.2,
    track_width=0.172,
    wheel_radius=0.045,
)
```

### Robot-Specific Parameters

| Robot | wheel_base | track_width | wheel_radius |
|-------|------------|-------------|--------------|
| LIMO | 0.2 | 0.172 | 0.045 |
| Leatherback | 0.32 | 0.24 | 0.052 |

### Note on Ackermann Wiring

Ackermann is more complex than differential drive because:
1. Steering joints need position control (angles)
2. Wheel joints need velocity control
3. Front-left and front-right have different steering angles (inside/outside wheel)

You may need separate ArticulationControllers or MakeArray nodes to wire the 4-6 outputs to the correct joints.

---

## Pattern C: Manipulator Joint Control

For robot arms controlled via joint state messages.

### Data Flow
```
Robot Joints → PublishJointState → /joint_states
/joint_command → SubscribeJointState → ArticulationController → Robot Joints
```

### Complete Code

```python
import omni.graph.core as og

def create_manipulator_graph(
    robot_path: str = "/World/panda",
    publish_topic: str = "/joint_states",
    subscribe_topic: str = "/joint_command",
):
    """Create action graph for manipulator joint control."""

    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("read_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("pub_joint_state", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("sub_joint_state", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("art_ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # ROS 2 config
                ("context.inputs:domain_id", 0),
                # Publisher config
                ("pub_joint_state.inputs:topicName", publish_topic),
                ("pub_joint_state.inputs:targetPrim", robot_path),
                # Subscriber config
                ("sub_joint_state.inputs:topicName", subscribe_topic),
                # Articulation controller
                ("art_ctrl.inputs:robotPath", robot_path),
            ],
            og.Controller.Keys.CONNECT: [
                # Tick triggers
                ("tick.outputs:tick", "pub_joint_state.inputs:execIn"),
                ("tick.outputs:tick", "sub_joint_state.inputs:execIn"),
                ("tick.outputs:tick", "art_ctrl.inputs:execIn"),
                # Context
                ("context.outputs:context", "pub_joint_state.inputs:context"),
                ("context.outputs:context", "sub_joint_state.inputs:context"),
                # Timestamp
                ("read_time.outputs:simulationTime", "pub_joint_state.inputs:timeStamp"),
                # Joint commands from subscriber to controller
                ("sub_joint_state.outputs:jointNames", "art_ctrl.inputs:jointNames"),
                ("sub_joint_state.outputs:positionCommand", "art_ctrl.inputs:positionCommand"),
                ("sub_joint_state.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
                ("sub_joint_state.outputs:effortCommand", "art_ctrl.inputs:effortCommand"),
            ],
        },
    )

# Usage for Franka Panda
create_manipulator_graph(
    robot_path="/World/panda",
    publish_topic="/joint_states",
    subscribe_topic="/joint_command",
)
```

### Test Commands

```bash
# Check published joint states
ros2 topic echo /joint_states --once

# Send joint command (example for 7-DOF arm)
ros2 topic pub /joint_command sensor_msgs/msg/JointState "{
  name: ['panda_joint1', 'panda_joint2', 'panda_joint3', 'panda_joint4', 'panda_joint5', 'panda_joint6', 'panda_joint7'],
  position: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]
}" --once
```

---

## Pattern D: Camera Publishing

Publish RGB and depth images to ROS 2.

### Complete Code

```python
import omni.graph.core as og

def create_camera_graph(
    camera_path: str = "/World/Camera",
    rgb_topic: str = "/camera/color/image_raw",
    depth_topic: str = "/camera/depth/image_raw",
    info_topic: str = "/camera/color/camera_info",
    frame_id: str = "camera_link",
    width: int = 640,
    height: int = 480,
):
    """Create action graph for camera publishing."""

    og.Controller.edit(
        {"graph_path": "/CameraGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                ("rgb_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("depth_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                ("info_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ],
            og.Controller.Keys.SET_VALUES: [
                # ROS 2 config
                ("context.inputs:domain_id", 0),
                # Render product
                ("render_product.inputs:cameraPrim", camera_path),
                ("render_product.inputs:width", width),
                ("render_product.inputs:height", height),
                # RGB publisher
                ("rgb_helper.inputs:type", "rgb"),
                ("rgb_helper.inputs:topicName", rgb_topic),
                ("rgb_helper.inputs:frameId", frame_id),
                # Depth publisher
                ("depth_helper.inputs:type", "depth"),
                ("depth_helper.inputs:topicName", depth_topic),
                ("depth_helper.inputs:frameId", frame_id),
                # Camera info publisher
                ("info_helper.inputs:type", "camera_info"),
                ("info_helper.inputs:topicName", info_topic),
                ("info_helper.inputs:frameId", frame_id),
            ],
            og.Controller.Keys.CONNECT: [
                # Tick triggers
                ("tick.outputs:tick", "rgb_helper.inputs:execIn"),
                ("tick.outputs:tick", "depth_helper.inputs:execIn"),
                ("tick.outputs:tick", "info_helper.inputs:execIn"),
                # Context
                ("context.outputs:context", "rgb_helper.inputs:context"),
                ("context.outputs:context", "depth_helper.inputs:context"),
                ("context.outputs:context", "info_helper.inputs:context"),
                # Render product to helpers
                ("render_product.outputs:renderProductPath", "rgb_helper.inputs:renderProductPath"),
                ("render_product.outputs:renderProductPath", "depth_helper.inputs:renderProductPath"),
                ("render_product.outputs:renderProductPath", "info_helper.inputs:renderProductPath"),
            ],
        },
    )

# Usage
create_camera_graph(
    camera_path="/World/jetbot/chassis/rgb_camera/jetbot_camera",
    rgb_topic="/camera/color/image_raw",
    depth_topic="/camera/depth/image_raw",
    frame_id="camera_color_optical_frame",
)
```

### Test Commands

```bash
# View image topics
ros2 topic list | grep camera

# Check image info
ros2 topic info /camera/color/image_raw

# View in rqt (if available)
ros2 run rqt_image_view rqt_image_view
```

---

## Pattern E: Clock + TF Publishing

Foundation graph for any robot - provides sim time and transforms.

```python
import omni.graph.core as og

def create_foundation_graph(robot_prims: list = ["/World/jetbot"]):
    """Create foundation graph with clock and TF."""

    og.Controller.edit(
        {"graph_path": "/FoundationGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("read_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("publish_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("context.inputs:domain_id", 0),
                ("publish_clock.inputs:topicName", "/clock"),
                ("publish_tf.inputs:topicName", "/tf"),
                # Note: targetPrims needs special handling for arrays
            ],
            og.Controller.Keys.CONNECT: [
                ("tick.outputs:tick", "publish_clock.inputs:execIn"),
                ("tick.outputs:tick", "publish_tf.inputs:execIn"),
                ("read_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
                ("context.outputs:context", "publish_clock.inputs:context"),
                ("context.outputs:context", "publish_tf.inputs:context"),
            ],
        },
    )

    # Set target prims for TF (requires attribute access after creation)
    # og.Controller.attribute("/FoundationGraph/publish_tf.inputs:targetPrims").set(robot_prims)

create_foundation_graph(["/World/jetbot"])
```

---

## Combining Patterns

For a complete robot setup, create multiple graphs or combine patterns:

```python
# Foundation (clock, TF)
create_foundation_graph(["/World/jetbot"])

# Robot control
create_differential_drive_graph(robot_path="/World/jetbot")

# Sensors
create_camera_graph(camera_path="/World/jetbot/chassis/rgb_camera/jetbot_camera")
```

## Troubleshooting

### Graph doesn't execute
1. Press Play in Isaac Sim
2. Check for red error badges on nodes
3. Verify node connections in Action Graph editor

### No ROS 2 topics
1. Match RMW: `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
2. Match domain: `export ROS_DOMAIN_ID=0`
3. Enable ROS 2 extension: Window > Extensions > "ROS2"

### Robot doesn't move
1. Verify `robotPath` points to ArticulationRoot prim
2. Check wheel joint names match robot URDF
3. Verify physics is enabled (Play button pressed)

## References

- [OmniGraph Python Scripting](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/omnigraph_scripting.html)
- [ROS 2 Joint Control](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [TurtleBot Driving](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html)
- [Mobile Robot Controllers](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/mobile_robot_controllers.html)
