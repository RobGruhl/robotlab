# Jetbot ActionGraph Reference

Complete working ActionGraph for NVIDIA Jetbot in Isaac Sim 5.0.

## Quick Start

Run `scripts/create_jetbot_graph.py` in Isaac Sim's Script Editor to create the full graph.

## Topics Published/Subscribed

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/clock` | `rosgraph_msgs/Clock` | publish | Simulation time |
| `/cmd_vel` | `geometry_msgs/Twist` | subscribe | Velocity commands |
| `/odom` | `nav_msgs/Odometry` | publish | Robot odometry |
| `/tf` | `tf2_msgs/TFMessage` | publish | Transform tree |

## TF Frames Published

```
world
└── chassis
    ├── left_wheel
    └── right_wheel
```

## Robot Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `ROBOT_PATH` | `/World/jetbot` | USD prim path |
| `WHEEL_RADIUS` | 0.0325 m | Wheel radius |
| `WHEEL_DISTANCE` | 0.1125 m | Wheel separation |

---

## Node Attributes (Isaac Sim 5.0)

### IsaacComputeOdometry
**Type:** `isaacsim.core.nodes.IsaacComputeOdometry`

Computes odometry from a chassis prim. **Use this** to get position/velocity data.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | uint | Execution trigger |
| `inputs:chassisPrim` | input | target | Robot chassis prim path |
| `outputs:execOut` | output | uint | Execution output |
| `outputs:position` | output | vector3d | World position |
| `outputs:orientation` | output | quatd | World orientation |
| `outputs:linearVelocity` | output | vector3d | Linear velocity |
| `outputs:angularVelocity` | output | vector3d | Angular velocity |
| `outputs:linearAcceleration` | output | vector3d | Linear acceleration |
| `outputs:angularAcceleration` | output | vector3d | Angular acceleration |

### ROS2PublishOdometry
**Type:** `isaacsim.ros2.bridge.ROS2PublishOdometry`

Publishes `nav_msgs/Odometry`. **Does NOT have chassisPrim** - requires computed values.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | uint | Execution trigger |
| `inputs:topicName` | input | string | Topic name |
| `inputs:odomFrameId` | input | string | Parent frame (e.g., "odom") |
| `inputs:chassisFrameId` | input | string | Child frame (e.g., "base_link") |
| `inputs:position` | input | vector3d | Position from IsaacComputeOdometry |
| `inputs:orientation` | input | quatd | Orientation from IsaacComputeOdometry |
| `inputs:linearVelocity` | input | vector3d | Linear velocity |
| `inputs:angularVelocity` | input | vector3d | Angular velocity |
| `inputs:timeStamp` | input | double | Simulation time |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

**Critical:** Wire `IsaacComputeOdometry` outputs → `ROS2PublishOdometry` inputs.

### ROS2PublishTransformTree
**Type:** `isaacsim.ros2.bridge.ROS2PublishTransformTree`

Publishes TF transforms for robot links.

| Port | Direction | Type | Description |
|------|-----------|------|-------------|
| `inputs:execIn` | input | uint | Execution trigger |
| `inputs:topicName` | input | string | Topic name (default `/tf`) |
| `inputs:targetPrims` | input | target | Robot prim(s) to publish transforms for |
| `inputs:parentPrim` | input | target | Optional parent frame prim |
| `inputs:timeStamp` | input | double | Simulation time |
| `inputs:staticPublisher` | input | bool | Publish to `/tf_static` instead |
| `inputs:context` | input | uint64 | Optional ROS 2 context |

---

## Complete Graph Script

```python
"""
Complete ActionGraph for Jetbot with ROS 2 integration.
Immutable deploy - deletes and recreates graph every time.

Publishes: /clock, /odom, /tf
Subscribes: /cmd_vel
"""
import omni.graph.core as og
import omni.usd
import traceback

GRAPH_PATH = "/World/ActionGraph"
ROBOT_PATH = "/World/jetbot"
WHEEL_RADIUS = 0.0325
WHEEL_DISTANCE = 0.1125
LOG_FILE = "/home/ubuntu/robotlab/scripts/graph_output.txt"

def delete_prim(path):
    """Delete prim using USD API."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
        return True
    return False

def create_jetbot_graph():
    keys = og.Controller.Keys

    # Delete existing graphs
    delete_prim(GRAPH_PATH)
    delete_prim("/ActionGraph")  # Clean up orphan if exists

    # Create fresh graph
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                # Core timing
                ("on_playback_tick", "omni.graph.action.OnPlaybackTick"),
                ("read_sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),

                # Clock publisher
                ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),

                # Twist subscriber + drive control
                ("subscribe_twist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("break_linear", "omni.graph.nodes.BreakVector3"),
                ("break_angular", "omni.graph.nodes.BreakVector3"),
                ("diff_controller", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("art_controller", "isaacsim.core.nodes.IsaacArticulationController"),

                # Odometry: compute from chassis, then publish
                ("compute_odom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("publish_odom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),

                # TF publisher
                ("publish_tf", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.SET_VALUES: [
                # Clock
                ("publish_clock.inputs:topicName", "/clock"),

                # Twist subscriber
                ("subscribe_twist.inputs:topicName", "/cmd_vel"),

                # Differential controller
                ("diff_controller.inputs:wheelRadius", WHEEL_RADIUS),
                ("diff_controller.inputs:wheelDistance", WHEEL_DISTANCE),

                # Articulation controller
                ("art_controller.inputs:robotPath", ROBOT_PATH),

                # Odometry compute
                ("compute_odom.inputs:chassisPrim", ROBOT_PATH),

                # Odometry publish
                ("publish_odom.inputs:topicName", "/odom"),
                ("publish_odom.inputs:odomFrameId", "odom"),
                ("publish_odom.inputs:chassisFrameId", "base_link"),

                # TF publisher
                ("publish_tf.inputs:topicName", "/tf"),
                ("publish_tf.inputs:targetPrims", ROBOT_PATH),
            ],
            keys.CONNECT: [
                # Clock publishing
                ("on_playback_tick.outputs:tick", "publish_clock.inputs:execIn"),
                ("read_sim_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),

                # Twist -> differential drive -> articulation
                ("on_playback_tick.outputs:tick", "subscribe_twist.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "diff_controller.inputs:execIn"),
                ("subscribe_twist.outputs:execOut", "art_controller.inputs:execIn"),
                ("subscribe_twist.outputs:linearVelocity", "break_linear.inputs:tuple"),
                ("subscribe_twist.outputs:angularVelocity", "break_angular.inputs:tuple"),
                ("break_linear.outputs:x", "diff_controller.inputs:linearVelocity"),
                ("break_angular.outputs:z", "diff_controller.inputs:angularVelocity"),
                ("diff_controller.outputs:velocityCommand", "art_controller.inputs:velocityCommand"),

                # Odometry: compute then publish
                ("on_playback_tick.outputs:tick", "compute_odom.inputs:execIn"),
                ("compute_odom.outputs:execOut", "publish_odom.inputs:execIn"),
                ("compute_odom.outputs:position", "publish_odom.inputs:position"),
                ("compute_odom.outputs:orientation", "publish_odom.inputs:orientation"),
                ("compute_odom.outputs:linearVelocity", "publish_odom.inputs:linearVelocity"),
                ("compute_odom.outputs:angularVelocity", "publish_odom.inputs:angularVelocity"),
                ("read_sim_time.outputs:simulationTime", "publish_odom.inputs:timeStamp"),

                # TF publishing
                ("on_playback_tick.outputs:tick", "publish_tf.inputs:execIn"),
                ("read_sim_time.outputs:simulationTime", "publish_tf.inputs:timeStamp"),
            ],
        },
    )

try:
    create_jetbot_graph()
    msg = "SUCCESS: Created ActionGraph with clock, cmd_vel, odom, and tf!"
    print(msg)
    with open(LOG_FILE, "w") as f:
        f.write(msg + "\n")
except Exception as e:
    error_msg = f"ERROR: {e}\n{traceback.format_exc()}"
    print(error_msg)
    with open(LOG_FILE, "w") as f:
        f.write(error_msg)
```

---

## Test Commands

```bash
# Source ROS 2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Check topics
ros2 topic list

# Drive forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" --rate 10

# Spin
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}" --once

# Check odometry
ros2 topic echo /odom --once

# Check TF
ros2 topic echo /tf --once
```

---

## Extending the Graph

To add camera publishing, add these nodes (see `docs/omnigraph/ros2-patterns.md` Pattern D):

```python
("render_product", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
("camera_helper", "isaacsim.ros2.bridge.ROS2CameraHelper"),
```

Discover camera path in your Jetbot USD:
- Likely: `/World/jetbot/chassis/rgb_camera/jetbot_camera`
