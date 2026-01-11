"""
Chase Camera ONLY - Does NOT touch the ground plane
Isaac Sim 5.0 - Run in Script Editor (Window > Script Editor)

Uses app update events (fires every frame).
Uses Isaac Sim's XFormPrim to get physics-updated transforms.
"""
import omni.usd
import omni.kit.app
from pxr import UsdGeom, Gf
from omni.kit.viewport.utility import get_active_viewport
from omni.isaac.core.prims import XFormPrim
import math
import traceback

# ============ CONFIGURATION ============
ROBOT_PATH = "/World/jetbot"
CAMERA_PATH = "/World/ChaseCamera"
CAMERA_DISTANCE = 0.5   # meters behind robot
CAMERA_HEIGHT = 0.3     # meters above robot
CAMERA_LOOK_AHEAD = 0.1 # meters ahead of robot to look at
LOG_FILE = "/home/ubuntu/robotlab/scripts/graph_output.txt"

# ============ GLOBAL STATE ============
_update_sub = None
_update_count = 0
_last_pos = None

# ============ LOGGING ============
def log(msg):
    print(msg)
    with open(LOG_FILE, "a") as f:
        f.write(msg + "\n")

# ============ CHASE CAMERA ============
def create_chase_camera():
    """Create a camera prim for the chase cam."""
    stage = omni.usd.get_context().get_stage()

    # Delete existing chase camera if present
    if stage.GetPrimAtPath(CAMERA_PATH):
        stage.RemovePrim(CAMERA_PATH)

    # Create camera with wide angle lens
    camera = UsdGeom.Camera.Define(stage, CAMERA_PATH)
    camera.GetFocalLengthAttr().Set(18.0)  # Wide angle
    camera.GetClippingRangeAttr().Set(Gf.Vec2f(0.1, 10000.0))

    log(f"Created chase camera at {CAMERA_PATH}")

def set_active_camera():
    """Set the viewport to use the chase camera."""
    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = CAMERA_PATH
        log("Set chase camera as active viewport")
    else:
        log("WARNING: No active viewport found")

def get_robot_transform():
    """Get robot's world position and yaw angle using Isaac Sim's physics-aware API."""
    try:
        robot = XFormPrim(prim_path=ROBOT_PATH)
        position, orientation = robot.get_world_pose()

        # orientation is [w, x, y, z] quaternion
        w, x, y, z = orientation[0], orientation[1], orientation[2], orientation[3]
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

        return position, yaw
    except Exception as e:
        log(f"Transform error: {e}")
        return None, None

def update_camera(event):
    """Update camera position to follow robot (called every frame)."""
    global _update_count, _last_pos

    pos, yaw = get_robot_transform()
    if pos is None:
        return

    # pos is numpy array [x, y, z]
    px, py, pz = float(pos[0]), float(pos[1]), float(pos[2])

    # Log occasionally to verify updates AND when position changes
    _update_count += 1
    pos_changed = _last_pos is None or abs(pz - _last_pos[2]) > 0.001

    if _update_count % 100 == 0 or (_update_count < 10) or pos_changed:
        log(f"Update #{_update_count}: robot at ({px:.3f}, {py:.3f}, {pz:.3f})")
        _last_pos = (px, py, pz)

    # Camera position: behind and above robot
    cam_x = pos[0] - math.cos(yaw) * CAMERA_DISTANCE
    cam_y = pos[1] - math.sin(yaw) * CAMERA_DISTANCE
    cam_z = pos[2] + CAMERA_HEIGHT

    # Look-at target: ahead of robot
    look_x = pos[0] + math.cos(yaw) * CAMERA_LOOK_AHEAD
    look_y = pos[1] + math.sin(yaw) * CAMERA_LOOK_AHEAD
    look_z = pos[2] + 0.05

    # Update camera transform
    stage = omni.usd.get_context().get_stage()
    camera_prim = stage.GetPrimAtPath(CAMERA_PATH)

    if camera_prim:
        # Create look-at matrix
        cam_mat = Gf.Matrix4d(1.0)
        cam_mat.SetLookAt(
            Gf.Vec3d(cam_x, cam_y, cam_z),
            Gf.Vec3d(look_x, look_y, look_z),
            Gf.Vec3d(0, 0, 1)  # Z-up
        )

        # Apply inverse (SetLookAt creates view matrix)
        xformable = UsdGeom.Xformable(camera_prim)
        xformable.ClearXformOpOrder()
        xform_op = xformable.AddTransformOp()
        xform_op.Set(cam_mat.GetInverse())

def start_chase_camera():
    """Initialize and start the chase camera."""
    global _update_sub

    create_chase_camera()
    set_active_camera()

    # Subscribe to app update events (fires every frame, always)
    app = omni.kit.app.get_app()
    update_stream = app.get_update_event_stream()
    _update_sub = update_stream.create_subscription_to_pop(update_camera)

    log("Chase camera app update subscription active")

# ============ MAIN ============
def setup():
    with open(LOG_FILE, "w") as f:
        f.write("=== Chase Camera Setup (App Update Version) ===\n\n")

    # Verify robot exists
    stage = omni.usd.get_context().get_stage()
    robot_prim = stage.GetPrimAtPath(ROBOT_PATH)
    if robot_prim:
        log(f"Found robot at {ROBOT_PATH}")
    else:
        log(f"WARNING: Robot not found at {ROBOT_PATH}")

    log("Setting up chase camera...")
    start_chase_camera()

    # Do an immediate position check
    pos, yaw = get_robot_transform()
    if pos:
        log(f"Initial robot position: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")

    log("\n" + "=" * 40)
    log("SETUP COMPLETE")
    log("=" * 40)
    log("\nCamera follows robot every frame (no Play needed for updates)")
    log("Watch for 'Update #' messages showing robot Z position")
    log("If Z keeps decreasing, robot is falling through ground!")

# Run setup
try:
    setup()
except Exception as e:
    error_msg = f"ERROR: {e}\n{traceback.format_exc()}"
    print(error_msg)
    with open(LOG_FILE, "w") as f:
        f.write(error_msg)
