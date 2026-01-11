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
