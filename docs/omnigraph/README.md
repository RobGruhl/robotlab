# OmniGraph Reference for ROS 2 Robotics

Quick reference for programmatic OmniGraph creation in Isaac Sim.

## What is OmniGraph?

OmniGraph is Isaac Sim's visual programming framework. It connects nodes that process data each simulation tick. For ROS 2 robotics, OmniGraph bridges Isaac Sim's physics simulation to ROS 2 topics.

**Key insight:** You can create graphs programmatically with Python (`og.Controller.edit()`) instead of manual GUI wiring.

## Documentation Index

| Document | Purpose |
|----------|---------|
| [programmatic-creation.md](programmatic-creation.md) | `og.Controller.edit()` API reference |
| [node-reference.md](node-reference.md) | Node catalog with input/output ports |
| [ros2-patterns.md](ros2-patterns.md) | Complete patterns for robots |

## Quick Reference

### Common Node Types

| Purpose | Node Type |
|---------|-----------|
| Tick trigger | `omni.graph.action.OnPlaybackTick` |
| Simulation time | `isaacsim.core.nodes.IsaacReadSimulationTime` |
| ROS 2 context | `isaacsim.ros2.bridge.ROS2Context` |
| Publish clock | `isaacsim.ros2.bridge.ROS2PublishClock` |
| Subscribe twist | `isaacsim.ros2.bridge.ROS2SubscribeTwist` |
| Publish joint state | `isaacsim.ros2.bridge.ROS2PublishJointState` |
| Subscribe joint state | `isaacsim.ros2.bridge.ROS2SubscribeJointState` |
| Articulation controller | `isaacsim.core.nodes.IsaacArticulationController` |
| Differential controller | `isaacsim.robot.wheeled_robots.DifferentialController` |
| Ackermann controller | `isaacsim.robot.wheeled_robots.AckermannController` |

### Minimal Python Pattern

```python
import omni.graph.core as og

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("publish_clock.inputs:topicName", "/clock"),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "publish_clock.inputs:execIn"),
        ],
    },
)
```

## External Documentation

- [OmniGraph Python Scripting](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/omnigraph_scripting.html)
- [ROS 2 Joint Control Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_manipulation.html)
- [TurtleBot Driving Tutorial](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_drive_turtlebot.html)
- [Mobile Robot Controllers](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/robot_simulation/mobile_robot_controllers.html)
