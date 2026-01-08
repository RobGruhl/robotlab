Below is a standalone plan document written as if it’s being handed to Claude Code as the “mission + execution blueprint.” It’s intentionally very detailed up front (so you get dopamine quickly and keep momentum), and then becomes more modular + option‑based later (so you can pivot without rewrites).

⸻

Sim‑First Robotics Learning Path

Rolling Base + Arm + Flying Drones + LLM/VLM Scene Analysis & Planning (ROS 2, Transferable to Real Hardware)

Audience

Claude Code (autonomous coding agent + repo builder). Assume the human is the product owner. You are building a durable robotics platform, not a one‑off demo.

⸻

0) North Star Vision

Build a single ROS 2–centric “robot brain” stack that can drive:
	1.	a rolling base (vision-powered RC rover)
	2.	a robot arm (tabletop manipulation → mobile manipulation later)
	3.	a flying drone (camera + offboard autonomy)

…with a shared LLM/VLM cognitive layer for:
	•	scene understanding (“what’s here / what changed / what’s blocking?”)
	•	goal interpretation (“go to the red cone”, “pick up the mug”, “inspect the window”)
	•	task planning (“break down objective into skill calls”)

Key constraint: Everything must be runnable in simulation first, and structured so that swapping in real hardware later is mostly a bringup/driver change, not an architecture rewrite.

⸻

1) Guiding Principles (Non‑Negotiables)

1.1 ROS 2 is the “thin waist”
	•	Every robot (sim or real) must present the same ROS 2 contract:
	•	sensors in (camera/depth/imu/odom)
	•	commands out (cmd_vel for base, trajectories for arm, offboard setpoints for drone)
	•	common TF tree conventions
	•	Anything that isn’t ROS 2 is an implementation detail.

1.2 Drones require an autopilot layer
	•	Unlike ground robots, drones almost always have a flight controller stack (PX4 or ArduPilot) that owns stabilization, safety, arming, failsafes.
	•	ROS 2 autonomy runs “above” it (companion computer).
This is the drone equivalent of /cmd_vel + motor controllers.

1.3 LLM/VLM outputs are untrusted
	•	LLMs can propose goals, scene interpretations, plans.
	•	The executor enforces schemas + guardrails + safety checks.
	•	LLMs never directly command raw motors.

1.4 Dopamine first, then depth

Each phase must end with a 30–90 second demo script you can run anytime.

1.5 Record everything
	•	Every demo must be “bag‑gable”: rosbag2 logs + configs + seed.
	•	Make replay the default debugging loop.

⸻

2) Recommended Tech Stack (Pinned Defaults, Easy to Pivot)

2.1 Ground base + arm simulation
	•	Isaac Sim + Isaac Lab for high‑fidelity sensors + learning workflows.
	•	Default pairing: Isaac Lab 2.3.x on Isaac Sim 5.1 (explicitly stated in release notes).  ￼

2.2 Drone simulation (two supported tracks)

Track A (canonical sim‑to‑real autopilot path):
	•	PX4 SITL + Gazebo (gz / Harmonic)
PX4’s docs treat modern Gazebo as the supported path and note Gazebo (new) supersedes Gazebo Classic on Ubuntu 22.04+.  ￼

Track B (photoreal / synthetic data / Isaac‑centric):
	•	Pegasus Simulator on Isaac Sim + PX4/ArduPilot integration
Pegasus is explicitly built on Omniverse/Isaac Sim and provides PX4 integration.  ￼

2.3 ROS 2 distros
	•	Prefer ROS 2 Jazzy if your main dev box is Ubuntu 24.04.
	•	Keep a ROS 2 Humble container available for compatibility with older packages.

2.4 Autopilot choice
	•	Primary: PX4 using the ROS 2 / DDS integration via uXRCE‑DDS (bridges PX4 uORB messages as ROS 2 topics).  ￼
	•	Alternate: ArduPilot, which documents direct DDS interface compatible with ROS 2 (starting in ArduPilot 4.5) via AP_DDS.  ￼

2.5 LLM/VLM integration approach
	•	Start with a minimal custom ROS 2 node that:
	•	samples frames
	•	calls a frontier VLM API (or local VLM)
	•	returns structured JSON validated against a schema
	•	Optionally study frameworks like ROS‑LLM (a ROS framework for LLM-based interaction/control).  ￼
	•	Optionally study a local VLM workshop repo (OpenVINO‑based) as a template for local inference.  ￼

2.6 Optional “LLM helps RL” later

Isaac Lab has an official “Eureka”‑style example repo: IsaacLabEureka (LLM‑generated reward tuning pipeline) and docs mention it as a resource for LLM‑generated reward functions.  ￼

⸻

3) Repository Blueprint (One Repo, Multiple Worlds)

Create a monorepo that holds everything needed to reproduce the demos.

3.1 Top-level layout

robotlab/
  README.md
  Makefile
  docs/
    demos/
    architecture/
    sim_to_real/
    safety/
  infra/
    docker/
    compose/
    devcontainer/
  ros_ws/
    src/
      robotlab_common/          # shared messages + utilities
      robotlab_bringup/         # launch files + configs
      robotlab_base/            # base control + nav wrappers
      robotlab_arm/             # moveit integration + pick/place skill
      robotlab_drone/           # px4/ardupilot integration + offboard skill
      robotlab_perception/      # camera utils, overlays, detectors
      robotlab_scene_graph/     # scene graph + memory
      robotlab_llm/             # VLM/LLM client + planner + schemas
      robotlab_executor/        # skill registry, safety gate, behavior tree
  sim/
    isaac/
      worlds/
      robots/
      scripts/
    gazebo/
      worlds/
      models/
      launch/
  schemas/
    scene_analysis.schema.json
    action_plan.schema.json
    skill_call.schema.json
  scripts/
    demo_base_teleop.sh
    demo_base_nav.sh
    demo_vlm_scene.sh
    demo_arm_pickplace.sh
    demo_drone_hover.sh
    demo_drone_inspect.sh

3.2 Core architectural modules
	1.	Robot I/O layer (sim or hardware drivers)
	2.	Skills layer (ROS 2 actions/services)
	3.	Scene Graph / World Model (robot memory)
	4.	LLM/VLM cognitive (perception + planning)
	5.	Executor + Safety Gate (trust boundary)

⸻

4) “Dopamine Demo Ladder” (What You’ll Be Able to Do)

These are your dopamine checkpoints. Each must become a single make demo-* command.

D0 — “I can see and drive something”
	•	teleop a simulated rolling base
	•	view camera in RViz/Foxglove

D1 — “Robot describes what it sees”
	•	VLM returns a structured list of objects + obstacles

D2 — “Natural language navigation”
	•	“Go to the red cone” → robot drives there using nav stack

D3 — “Pick and place”
	•	“Pick up the mug” → arm executes pick/place in sim

D4 — “Drone hover with ROS offboard”
	•	takeoff → hover → land with ROS 2 offboard node

D5 — “Drone inspection”
	•	“Inspect the window” → drone flies a short inspection path, returns a scene report

D6 — “North Star Mission”
	•	A combined scenario:
	•	drone scouts & produces a scene report (semantic map or annotations)
	•	rover navigates to a target
	•	arm manipulates an object
	•	LLM produces the plan and the system executes safely

⸻

PHASE 0 (Detailed): Bootstrap the Platform (1–2 sessions)

Goal

Set up the repo + tooling so every future step is fast.

Deliverables
	•	robotlab/ repo created
	•	ros_ws/ builds in a container
	•	first “hello world” nodes run
	•	rosbag2 record/replay works
	•	make demo-smoke runs a minimal end-to-end graph

Tasks for Claude Code
	1.	Create the repo structure exactly as in Section 3.
	2.	Create Docker-based dev env(s):
	•	infra/docker/Dockerfile.ros_jazzy
	•	infra/docker/Dockerfile.ros_humble
	3.	Create docker compose configs:
	•	compose.ros.yml for ROS graph + RViz
	•	compose.tools.yml for lint/test
	4.	Add a Makefile with:
	•	make build
	•	make test
	•	make lint
	•	make demo-smoke
	5.	Implement robotlab_common:
	•	helper libs: logging, time utils, message conversion helpers
	•	a tiny Heartbeat node publishing std_msgs/String or diagnostic_msgs

Smoke demo definition
	•	Run robotlab_common/heartbeat
	•	Run robotlab_common/echo_listener
	•	Record 10 seconds with rosbag2
	•	Replay and confirm listener sees the topic again

⸻

PHASE 1 (Detailed): Rolling Base Simulation + ROS 2 Contract (D0) (2–5 sessions)

Goal

A simulated rolling base that you can:
	•	teleop
	•	see through a camera topic
	•	log and replay

Choose ONE sim route for base

Preferred (matches your earlier direction): Isaac Sim (ROS 2 bridge)
Fallback: Gazebo (gz) for speed + simplicity

Required ROS 2 contract for the base

Topics (minimum viable):
	•	Publish:
	•	/tf
	•	/odom
	•	/camera/image_raw
	•	/camera/camera_info
	•	Subscribe:
	•	/cmd_vel

Frames:
	•	map (optional early)
	•	odom
	•	base_link
	•	camera_link

Dopamine demo (D0)

“Drive around + camera stream visible.”

Tasks for Claude Code
	1.	Create package: robotlab_base
	2.	Implement:
	•	teleop.launch.py
	•	base_bridge.launch.py (sim-specific bridge)
	•	rviz_base.rviz
	3.	Add scripts/demo_base_teleop.sh that:
	•	launches sim (or assumes already running)
	•	launches base_bridge
	•	launches RViz
	•	launches teleop node
	4.	Add make demo-base-teleop

Stretch (still Phase 1)
	•	Add a “safety stop” node:
	•	if no teleop input for N seconds → publish zero cmd_vel
	•	Add rosbag recording helper script.

⸻

PHASE 2 (Detailed): VLM Scene Analysis Node + Scene Graph (D1) (2–6 sessions)

Goal

Take the base camera feed and produce:
	•	structured scene analysis
	•	a persistent scene graph that updates over time
	•	an RViz visualization overlay (markers / text)

This is the first big “modern robotics” dopamine hit.

Architecture (minimum)

2.1 Frame sampler node

robotlab_perception/frame_sampler_node.py
	•	Subscribes: /camera/image_raw
	•	Publishes: /robotlab/frame_sampled (compressed image or encoded payload)
	•	Policy:
	•	sample at 1–2 Hz OR on-demand (service call)
	•	downscale + JPEG compress for API cost control

2.2 VLM scene analyzer node

robotlab_llm/vlm_scene_analyzer_node.py
	•	Input: sampled image + optional prompt
	•	Output: std_msgs/String JSON (for v0), later robotlab_msgs/SceneAnalysis

Hard requirement: output must validate against schemas/scene_analysis.schema.json.

Example JSON shape:

{
  "timestamp": 1730000000.0,
  "camera_frame": "camera_link",
  "objects": [
    {"label":"mug","confidence":0.82,"bbox_xyxy":[10,20,100,120]},
    {"label":"chair","confidence":0.77,"bbox_xyxy":[200,50,360,300]}
  ],
  "obstacles":[{"type":"person","risk":"high","region":"center"}],
  "free_text_summary":"A mug on the table, chair to the right, person ahead."
}

2.3 Scene graph memory node

robotlab_scene_graph/scene_graph_node.py
	•	Subscribes: SceneAnalysis
	•	Maintains:
	•	last N detections
	•	fused object list (simple tracker)
	•	publishes a “current scene” topic:
	•	/robotlab/scene_graph

2.4 Visualization
	•	RViz markers for object labels + bbox as projected markers (or image overlay in Foxglove)
	•	Text summary in RViz

Dopamine demo (D1)

“Robot describes the scene in structured form.”
	•	Run base sim + camera
	•	Run VLM analyzer
	•	Watch scene text update live

Tooling note

You can optionally look at existing ROS frameworks like ROS‑LLM for inspiration on packaging, quickstart, and LLM-driven control concepts.  ￼
You can optionally look at a local VLM workshop repo (OpenVINO-based) for patterns on local inference + ROS 2 packaging if you want to avoid API calls later.  ￼

⸻

PHASE 3 (Medium Detail): Natural Language Navigation (D2) (3–10 sessions)

Goal

Command: “Go to the red cone”
Robot: finds “red cone” (via VLM + scene graph) → chooses a goal pose → navigates there.

Key idea

Use the LLM/VLM for semantic grounding, but keep navigation classical:
	•	nav2 / planners / controllers do motion
	•	LLM picks which target and what constraints

Implementation sketch

3.1 Skills

Create robotlab_executor skills:
	•	NavigateToPose (ROS 2 action wrapper)
	•	Stop
	•	RotateInPlace
	•	ApproachObject (optional: last-meter alignment)

3.2 Planner node (LLM)

robotlab_llm/goal_interpreter_node.py
	•	Input: /robotlab/user_goal (text)
	•	Tools available (function calling style):
	•	get_scene_graph()
	•	resolve_object(label_or_description)
	•	propose_goal_pose(object_id)
	•	call_skill(name, args)
	•	Output: a structured ActionPlan validated by action_plan.schema.json

3.3 Executor with safety gate

robotlab_executor/executor_node.py
	•	Validates plan schema
	•	Checks skills are whitelisted
	•	Applies constraints:
	•	max velocity
	•	geofenced area (in sim)
	•	timeouts
	•	Executes sequentially (or BT)

Dopamine demo (D2)
	•	Place a few obvious objects in the sim scene
	•	Command: “Go to the chair”
	•	Robot navigates to it (even if approximate)

⸻

PHASE 4 (Medium Detail): Arm Simulation + Pick/Place (D3) (5–15 sessions)

Goal

Command: “Pick up the mug.”
Arm: plans + executes pick and place in sim.

Strategy

Start tabletop, stationary base (or fixed arm). Only later combine with mobile base.

Components
	•	MoveIt 2 pipeline (planning scene, IK, collision)
	•	Gripper control (simple open/close)
	•	Perception:
	•	v0: spawn the mug pose directly from sim (cheat for dopamine)
	•	v1: VLM identifies mug in RGB, estimate pose via depth or known table plane

Skills
	•	Pick(object_pose)
	•	Place(pose)
	•	StowArm()

Dopamine demo (D3)
	•	“Pick up the mug and place it in the bin.”

⸻

PHASE 5 (Medium Detail): Drone Simulation + ROS 2 Offboard (D4/D5) (5–20 sessions)

Goal

Drone does takeoff → hover → land (D4), then a simple inspection mission (D5).

Recommended path: PX4 SITL + Gazebo (gz)

Why

It’s the cleanest sim-to-real shape:
	•	autopilot is in the loop from day 1
	•	you learn the real control contract (offboard setpoints + heartbeat)

PX4’s ROS 2 integration story:
	•	uXRCE‑DDS bridges uORB messages so they can be published/subscribed on a companion computer as though they were ROS 2 topics.  ￼
PX4 offboard contract:
	•	requires continuous 2 Hz “proof of life” (can be MAVLink setpoints or ROS 2 OffboardControlMode) or it will exit offboard.  ￼
PX4 provides an explicit ROS 2 offboard control example to demonstrate the pattern.  ￼

Gazebo details:
	•	modern Gazebo is the supported track on Ubuntu 22.04+ (Gazebo supersedes Gazebo Classic).  ￼
	•	PX4’s Gazebo vehicles include ready-made models like:
	•	gz_x500_vision
	•	gz_x500_depth (front-facing depth camera modeled on OAK‑D)  ￼

Drone ROS contract (minimum)
	•	Inputs (from PX4 to ROS 2):
	•	vehicle odometry / state topics (depending on chosen bridge)
	•	Outputs (ROS 2 to PX4):
	•	Offboard control mode heartbeat
	•	trajectory setpoints (position/velocity)

Drone cognitive layer (LLM/VLM)

Use the exact same Phase 2/3 pattern:
	•	frame sampler reads drone camera
	•	VLM produces scene analysis (“window detected”, “obstacle ahead”)
	•	LLM planner turns “inspect the window” into:
	•	takeoff
	•	fly to waypoint(s)
	•	yaw/orbit for a few seconds
	•	return & land
	•	executor gates + runs skills

Dopamine demos

D4: “Takeoff → hover 10 seconds → land.”
D5: “Inspect the window” (fly a rectangle path facing target).

⸻

PHASE 6 (Sketch): Unified North Star Mission (D6)

Goal

A single command like:

“Find the red toolbox and bring it to the marked zone. If you can’t find it, scout from above and report what you see.”

Example mission decomposition
	1.	Drone: takeoff, scan area, produce scene report
	2.	Rover: navigate to target area
	3.	Arm: pick toolbox
	4.	Rover: navigate to drop zone
	5.	Arm: place
	6.	LLM: generate final report (“task complete, objects detected, issues encountered”)

Why this matters

This is the moment where your architecture proves it’s not a toy:
	•	multiple robot types
	•	shared cognitive layer
	•	skills + executor are reusable

⸻

PHASE 7 (Optional Advanced Track): Learning with Isaac Lab + LLM Help

You can add robot learning once the classical skill skeleton works.

Two good “learning insertion points”
	1.	Last-meter alignment (base → object)
	2.	Grasp selection / approach (arm → clutter)

LLM helps learning (optional)

Isaac Lab maintains IsaacLabEureka, described as a pipeline that prompts an LLM to discover/tune reward functions for Isaac Lab tasks, supporting OpenAI/Azure OpenAI APIs.  ￼
Treat this as a later “power tool” once you’re already getting traction.

⸻

PHASE 8 (Sketch): Sim‑to‑Real Readiness Checklists

8.1 Rolling base
	•	Swap sim driver for real base driver, keep /cmd_vel, /odom, /tf, camera topics identical
	•	Calibrate:
	•	wheel radius / wheelbase
	•	camera intrinsics
	•	IMU orientation
	•	Add:
	•	e‑stop
	•	speed limits
	•	watchdog timeouts

8.2 Arm
	•	Confirm URDF/SRDF match real arm
	•	Validate joint limits, collisions, frames
	•	Add gripper force/position constraints
	•	Use identical MoveIt interface between sim and real

8.3 Drone
	•	Keep autopilot consistent (PX4↔PX4 or ArduPilot↔ArduPilot)
	•	Reuse offboard nodes unchanged
	•	Add real-world safety:
	•	RC takeover
	•	geofence
	•	failsafe behaviors
	•	Remember that drones’ low-level stabilization is not “optional”—autopilot is the product

ArduPilot ROS 2 support note:
	•	ArduPilot docs state that starting with 4.5 it supports a direct DDS interface compatible with ROS 2, reducing reliance on MAVROS in some applications.  ￼

⸻

Operating Rules for Claude Code (How to Execute This Plan)
	1.	Always add a demo script for each milestone.
	2.	Add a README “Run This Demo” section for each milestone.
	3.	Prefer Python ROS 2 nodes first (rclpy) for speed; optimize later.
	4.	Validate every LLM response with JSON Schema; reject on mismatch.
	5.	Never let LLM directly publish raw actuator topics.
	6.	Add timeouts everywhere (skills, planner, VLM calls).
	7.	Keep the cognitive layer swappable:
	•	frontier API VLM
	•	local VLM
	•	“no LLM” fallback (scripted baseline)

⸻

Appendix A: Reference URLs (copy/paste)

(Placed in a code block so they’re easy to copy.)

Isaac Lab release notes (2.3.0 built on Isaac Sim 5.1):
https://isaac-sim.github.io/IsaacLab/main/source/refs/release_notes.html

IsaacLabEureka (LLM reward generation pipeline for Isaac Lab):
https://github.com/isaac-sim/IsaacLabEureka

Isaac Lab additional resources (mentions Isaac Lab Eureka):
https://isaac-sim.github.io/IsaacLab/main/source/refs/additional_resources.html

ROS-LLM framework:
https://github.com/Auromix/ROS-LLM

VLMs with ROS2 workshop (OpenVINO-based local inference patterns):
https://github.com/nilutpolkashyap/vlms_with_ros2_workshop

PX4 uXRCE-DDS bridge (PX4 <-> ROS2/DDS):
https://docs.px4.io/main/en/middleware/uxrce_dds

PX4 Offboard mode (2 Hz proof-of-life requirement):
https://docs.px4.io/main/en/flight_modes/offboard

PX4 ROS2 Offboard control example:
https://docs.px4.io/main/en/ros2/offboard_control

PX4 Gazebo (gz) simulation overview:
https://docs.px4.io/main/en/sim_gazebo_gz/

PX4 Gazebo vehicle list (gz_x500_depth, gz_x500_vision):
https://docs.px4.io/main/en/sim_gazebo_gz/vehicles

Pegasus Simulator (Isaac Sim multirotor framework w/ PX4 integration):
https://pegasussimulator.github.io/PegasusSimulator/
https://github.com/PegasusSimulator/PegasusSimulator

ArduPilot ROS2 docs (AP_DDS, DDS interface compatible with ROS 2):
https://ardupilot.org/dev/docs/ros2.html
https://ardupilot.org/dev/docs/ros.html