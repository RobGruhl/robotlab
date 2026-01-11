# Programmatic OmniGraph Creation

Create and edit OmniGraph action graphs using Python instead of the GUI.

## Key Lessons Learned

### Graph Creation vs Editing

```python
import omni.graph.core as og

keys = og.Controller.Keys

# CREATE new graph (fails if graph already exists!)
og.Controller.edit(
    {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
    { ... }
)

# EDIT existing graph (use path string, not dict)
og.Controller.edit(
    "/World/ActionGraph",
    { ... }
)
```

### Immutable Deploy Pattern (Recommended)

Delete and recreate the graph every time for clean, repeatable deploys:

```python
import omni.graph.core as og
import omni.usd

def delete_prim(path):
    """Delete prim using USD API."""
    stage = omni.usd.get_context().get_stage()
    if stage.GetPrimAtPath(path):
        stage.RemovePrim(path)
        return True
    return False

def create_graph():
    # Delete existing graph first
    delete_prim("/World/ActionGraph")

    # Create fresh
    og.Controller.edit(
        {"graph_path": "/World/ActionGraph", "evaluator_name": "execution"},
        { ... }
    )
```

**Important:** `og.Controller.delete_graph()` does NOT exist. Use `stage.RemovePrim()` instead.

### Discovering Node Attributes

**Node attributes vary by Isaac Sim version!** Always discover them programmatically:

```python
import omni.graph.core as og
import omni.usd

LOG_FILE = "/home/ubuntu/robotlab/scripts/graph_output.txt"

def discover_node_attributes(node_type):
    """Create temp node and list its attributes."""

    og.Controller.edit(
        {"graph_path": "/TempGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("temp_node", node_type),
            ],
        },
    )

    graph = og.get_graph_by_path("/TempGraph")
    node = graph.get_node("/TempGraph/temp_node")

    output = [f"{node_type} attributes:", "\nInputs:"]
    for attr in node.get_attributes():
        name = attr.get_name()
        if "inputs:" in name:
            output.append(f"  {name} ({attr.get_type_name()})")

    output.append("\nOutputs:")
    for attr in node.get_attributes():
        name = attr.get_name()
        if "outputs:" in name:
            output.append(f"  {name} ({attr.get_type_name()})")

    result = "\n".join(output)
    print(result)
    with open(LOG_FILE, "w") as f:
        f.write(result)

    # Clean up
    stage = omni.usd.get_context().get_stage()
    stage.RemovePrim("/TempGraph")

# Usage
discover_node_attributes("isaacsim.ros2.bridge.ROS2PublishOdometry")
```

### Finding Available Node Types

```python
import omni.graph.core as og

# Search for node types by keyword
for node_type in og.get_registered_nodes():
    if "odom" in node_type.lower():
        print(node_type)
```

### Listing Nodes in Existing Graph

```python
import omni.graph.core as og

# List all graphs in scene
for graph in og.get_all_graphs():
    path = graph.get_path_to_graph()
    print(f"\nGraph: {path}")
    for node in graph.get_nodes():
        print(f"  {node.get_prim_path()}")
```

### Error Logging Pattern

Isaac Sim's Script Editor output is hard to copy. Always log to file:

```python
import traceback

LOG_FILE = "/home/ubuntu/robotlab/scripts/graph_output.txt"

try:
    create_my_graph()
    msg = "SUCCESS: Graph created!"
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

## Common Errors

| Error | Cause | Fix |
|-------|-------|-----|
| "Failed to wrap graph in node" | Graph already exists | Delete with `stage.RemovePrim()` first |
| "Attribute 'inputs:X' does not refer to a legal og.Attribute" | Wrong attribute name for this Isaac Sim version | Use discovery script to find correct names |
| "Failed to connect X -> Y" | Wrong node name or port name | Check node names match CREATE_NODES |

---

## og.Controller.edit() API Reference

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

---

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

---

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

---

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

---

## Running in Isaac Sim

### Script Editor
1. Window > Script Editor
2. Paste Python code
3. Run (Ctrl+Enter)

**Tip:** Save scripts to files and edit in VS Code for easier copy/paste.

### Standalone Python
```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import omni.graph.core as og
# ... create graph ...

while simulation_app.is_running():
    simulation_app.update()
```

---

## References

- [OmniGraph Python Scripting](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/omnigraph/omnigraph_scripting.html)
- [ROS 2 Bridge Standalone](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/tutorial_ros2_python.html)
