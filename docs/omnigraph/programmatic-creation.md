# Programmatic OmniGraph Creation

Create and edit OmniGraph action graphs using Python instead of the GUI.

## og.Controller.edit() API

The primary API for creating graphs programmatically.

### Basic Syntax

```python
import omni.graph.core as og

(graph_handle, nodes, _, _) = og.Controller.edit(
    {
        "graph_path": "/ActionGraph",      # USD path for the graph
        "evaluator_name": "execution",      # "execution" for action graphs
    },
    {
        og.Controller.Keys.CREATE_NODES: [...],  # Nodes to create
        og.Controller.Keys.SET_VALUES: [...],    # Attribute values to set
        og.Controller.Keys.CONNECT: [...],       # Node connections
    }
)
```

### Graph Configuration Options

| Key | Type | Description |
|-----|------|-------------|
| `graph_path` | string | USD prim path (e.g., `/ActionGraph`, `/World/MyGraph`) |
| `evaluator_name` | string | `"execution"` for action graphs, `"push"` for push graphs |
| `pipeline_stage` | enum | Optional: `og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND` for manual trigger |

### Controller Keys

| Key | Purpose | Example |
|-----|---------|---------|
| `CREATE_NODES` | Instantiate nodes | `("node_name", "node.type.path")` |
| `SET_VALUES` | Set attribute values | `("node_name.inputs:attr", value)` |
| `CONNECT` | Wire nodes together | `("src.outputs:port", "dst.inputs:port")` |
| `DELETE_NODES` | Remove nodes | `"node_name"` |

## CREATE_NODES

Create nodes with a tuple of (node_name, node_type).

```python
og.Controller.Keys.CREATE_NODES: [
    ("tick", "omni.graph.action.OnPlaybackTick"),
    ("read_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
    ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
]
```

**Node name rules:**
- Names are local to the graph (no leading `/`)
- Full path becomes `{graph_path}/{node_name}`
- Keep names short but descriptive

## SET_VALUES

Set input attributes using dot notation.

```python
og.Controller.Keys.SET_VALUES: [
    ("publish_clock.inputs:topicName", "/clock"),
    ("context.inputs:domain_id", 0),
    ("context.inputs:useDomainIDEnvVar", True),
    ("diff_ctrl.inputs:wheelRadius", 0.03),
    ("diff_ctrl.inputs:wheelDistance", 0.1125),
    ("art_ctrl.inputs:robotPath", "/World/jetbot"),
]
```

**Attribute path syntax:**
- `"node_name.inputs:attribute_name"` for inputs
- `"node_name.outputs:attribute_name"` for outputs (rarely set directly)

**Value types:**
- Strings: `"topic_name"`
- Numbers: `0.03`, `10`, `1.0`
- Booleans: `True`, `False`
- Arrays: `[0.1, 0.2, 0.3]`
- Prim paths: `"/World/robot"` (string)

## CONNECT

Wire outputs to inputs.

```python
og.Controller.Keys.CONNECT: [
    ("tick.outputs:tick", "publish_clock.inputs:execIn"),
    ("read_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
    ("diff_ctrl.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
]
```

**Connection rules:**
- Source must be `outputs:` port
- Destination must be `inputs:` port
- Types must be compatible (or automatically converted)

## Attribute Access Outside edit()

Read and write attributes on existing graphs.

```python
# Get attribute value
value = og.Controller.attribute("/ActionGraph/node.inputs:attr").get()

# Set attribute value
og.Controller.attribute("/ActionGraph/node.inputs:attr").set(new_value)

# Short form for setting
og.Controller.set(og.Controller.attribute("/ActionGraph/node.inputs:attr"), value)
```

## Manual Graph Execution

For on-demand graphs (not ticked automatically).

```python
# Create on-demand graph
og.Controller.edit(
    {
        "graph_path": "/ManualGraph",
        "evaluator_name": "execution",
        "pipeline_stage": og.GraphPipelineStage.GRAPH_PIPELINE_STAGE_ONDEMAND
    },
    {...}
)

# Get handle and evaluate
graph = og.get_graph_by_path("/ManualGraph")
graph.evaluate()

# Or trigger via OnImpulseEvent node
og.Controller.set(
    og.Controller.attribute("/ManualGraph/OnImpulseEvent.state:enableImpulse"),
    True
)
```

## Complete Examples

### Clock Publisher

```python
import omni.graph.core as og

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("read_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("context", "isaacsim.ros2.bridge.ROS2Context"),
            ("publish_clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("publish_clock.inputs:topicName", "/clock"),
            ("context.inputs:domain_id", 0),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "publish_clock.inputs:execIn"),
            ("read_time.outputs:simulationTime", "publish_clock.inputs:timeStamp"),
            ("context.outputs:context", "publish_clock.inputs:context"),
        ],
    },
)
```

### Joint State Publisher/Subscriber (Manipulator)

```python
import omni.graph.core as og

og.Controller.edit(
    {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
    {
        og.Controller.Keys.CREATE_NODES: [
            ("tick", "omni.graph.action.OnPlaybackTick"),
            ("read_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("pub_joint", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("sub_joint", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("art_ctrl", "isaacsim.core.nodes.IsaacArticulationController"),
        ],
        og.Controller.Keys.SET_VALUES: [
            ("art_ctrl.inputs:robotPath", "/World/panda"),
            ("pub_joint.inputs:targetPrim", "/World/panda"),
            ("pub_joint.inputs:topicName", "/joint_states"),
            ("sub_joint.inputs:topicName", "/joint_command"),
        ],
        og.Controller.Keys.CONNECT: [
            ("tick.outputs:tick", "pub_joint.inputs:execIn"),
            ("tick.outputs:tick", "sub_joint.inputs:execIn"),
            ("tick.outputs:tick", "art_ctrl.inputs:execIn"),
            ("read_time.outputs:simulationTime", "pub_joint.inputs:timeStamp"),
            ("sub_joint.outputs:jointNames", "art_ctrl.inputs:jointNames"),
            ("sub_joint.outputs:positionCommand", "art_ctrl.inputs:positionCommand"),
            ("sub_joint.outputs:velocityCommand", "art_ctrl.inputs:velocityCommand"),
            ("sub_joint.outputs:effortCommand", "art_ctrl.inputs:effortCommand"),
        ],
    },
)
```

## Running in Isaac Sim

### Script Editor
1. Window > Script Editor
2. Paste Python code
3. Run (Ctrl+Enter)

### Standalone Python
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.graph.core as og
# ... create graph ...

# Start simulation
simulation_app.update()  # Tick once
# or
while simulation_app.is_running():
    simulation_app.update()
```

### Extension
Create a custom extension that builds the graph in `on_startup()`.

## Debugging

### Check if graph exists
```python
graph = og.get_graph_by_path("/ActionGraph")
if graph is not None:
    print("Graph exists")
```

### List all nodes
```python
graph = og.get_graph_by_path("/ActionGraph")
for node in graph.get_nodes():
    print(node.get_prim_path())
```

### Check connections
```python
node = og.get_node_by_path("/ActionGraph/publish_clock")
for attr in node.get_attributes():
    print(f"{attr.get_name()}: {attr.get_upstream_connections()}")
```

## References

- [OmniGraph Python Scripting](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/omnigraph_scripting.html)
- [ROS 2 Bridge Standalone](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_python.html)
