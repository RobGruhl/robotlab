You can make either workflow work, but for robotics sim + ROS 2 + Isaac Sim/Gazebo the best choice is the one that minimizes “environment mismatch” and “round‑trip friction.” In practice that usually means: run Claude Code where the simulator and ROS graph run.

Below are the main options, what lives where, and the tradeoffs. Then I’ll give a concrete recommendation and a “what goes where” map you can adopt immediately.

⸻

The decision drivers that actually matter

1) Where do commands need to run?

Claude Code is strongest when it can run your real commands: build, test, launch, inspect logs, grep, open files, etc. If the “truth” lives on the AWS workstation (Isaac Sim, ROS, GPU drivers, containers), then having Claude Code there avoids the “push → SSH → run → pull logs back” dance.

2) How often is the GPU workstation running?
	•	If you expect to keep the AWS workstation up for long coding sessions anyway, remote-first is natural.
	•	If you want the GPU instance off most of the time, local-first (with occasional remote sync) saves money.

3) Session/context persistence

Claude Code stores session history locally, and supports claude --continue / claude --resume to restore previous conversations.  ￼
If you run Claude Code on a remote instance and you terminate it (not just stop), you can lose that session history unless the relevant directories are on persistent storage. The docs show config/state live in places like ~/.claude, ~/.claude.json, and per-project .claude/.  ￼

4) Secret management & security posture

Wherever Claude Code runs, it can “see” the environment: shell history, tokens, SSH keys, etc. You’ll want:
	•	least-privilege Git credentials
	•	careful tool allowlists/permissions
	•	a clear boundary for what the agent can do

Anthropic explicitly recommends curating allowed tools and using repo-level CLAUDE.md and .claude/settings.json patterns.  ￼

⸻

Option 1: Remote-first (Claude Code runs on the AWS workstation)

What lives where

AWS GPU workstation
	•	Your “primary” working clone of the repo
	•	Isaac Sim / Gazebo / PX4 SITL, ROS 2 nodes, containers
	•	Claude Code installed and run in the repo directory
	•	Build artifacts, logs, rosbag2 files (ideally on a persistent disk)
	•	.claude/ project settings and ~/.claude session history (persistent)

Laptop (Mac)
	•	Thin client: SSH, VS Code Remote SSH, terminal multiplexer access, streaming client
	•	Optional secondary clone for quick browsing/review, but not required

GitHub
	•	Source of truth / collaboration / rollback

Pros
	•	Fastest iteration loop for sim-heavy robotics: “change code → run sim → see result”
	•	Claude Code can run exactly the commands that matter (launch files, ROS graph introspection, sim scripts)
	•	Less time wasted on dependency mismatches between macOS and Ubuntu/Linux

Cons
	•	You pay for the workstation while you’re coding (unless you split dev vs GPU; more on that below)
	•	If you terminate instances without persistent storage, you can lose state (including Claude sessions) unless you persist ~/.claude and your repo volumes  ￼
	•	You’ll store Git credentials on the remote machine (mitigate with least privilege)

Who this is best for
	•	Anyone doing Isaac Sim + ROS 2 as the daily driver
	•	Anyone who wants dopamine via tight sim iteration and fewer moving parts

⸻

Option 2: Local-first (Claude Code on your laptop; remote is “execution only”)

What lives where

Laptop
	•	Primary repo clone
	•	Claude Code runs locally; writes code locally
	•	You push to GitHub frequently

AWS workstation
	•	Pulls from GitHub when needed
	•	Runs sim and heavy tests
	•	Produces logs/bags/artifacts; you download if needed

Pros
	•	AWS workstation can stay off most of the time → cheaper
	•	Your code + Claude sessions live locally and persist naturally
	•	Easier to keep secrets confined to your laptop

Cons
	•	For robotics, you’ll constantly hit “it works locally but not in sim” (because macOS ≠ Ubuntu + GPU stack)
	•	More friction: push/pull cycles, remote debugging, copying logs, “what changed on remote?”

Who this is best for
	•	If you’re mostly doing algorithm/design/doc work, with occasional sim runs
	•	If cost control is your top constraint and you’re okay with slower sim iteration

⸻

Option 3: Hybrid (recommended pattern for your use case)

There are a few hybrids; the one that tends to work best for your goals is:

Remote-first for anything that must touch sim/GPU/ROS graph,
local-first for reading, note-taking, code review, and quick edits when the instance is off.
Use GitHub as the canonical sync layer.

What lives where (hybrid baseline)

AWS workstation = “runtime + integration dev machine”
	•	The authoritative dev environment for: Isaac Sim / Gazebo / PX4 / ROS 2
	•	Claude Code installed there for: integration work, launch/debug, log-driven fixes
	•	Repo clone lives there on a persistent volume
	•	Session persistence (Claude) stays there (again: persistent storage!)

Laptop = “control plane + review plane”
	•	Optional local clone (can be shallow)
	•	Your docs, task list, design sketches
	•	A place to do PR review and “sanity pass” before pushing merges

Pros
	•	Keeps the high-velocity sim loop
	•	Still allows you to do “cheap work” locally when the GPU box is off
	•	Reduces the “everything depends on cloud uptime” feeling

Cons
	•	You accept that Claude Code sessions are not automatically shared across local/remote (because they’re stored locally per machine)  ￼
	•	Slightly more discipline required: decide “am I in remote mode or local mode?”

⸻

My recommendation for you

Given everything you’ve described (Isaac Sim/Lab, ROS 2, drones later, sim-to-real, dopamine hits):

Recommendation

Install and run Claude Code on the AWS remote workstation, and treat that workstation as your primary robotics dev environment.
Keep an optional local clone for review/notes, but don’t try to make your Mac be the primary executor for this stack.

Why
	•	Your “truth” environment is Linux + GPU + simulators. Running Claude Code there eliminates the biggest source of friction: environment mismatch.
	•	Your early phases will be dominated by “bringup + glue + launch + logs.” Claude Code shines when it can run and iterate in the same place as the problem.
	•	Claude Code supports resuming sessions (--continue, --resume), but session history and settings are stored locally on the machine where it runs, so you want that to be a persistent dev box (or persistent disk) rather than bouncing between local and remote.  ￼

⸻

“What lives where?” – a concrete placement map

GitHub
	•	✅ Source code (everything you want versioned)
	•	✅ .claude/ project settings that you want shared (tool permissions, project config)
	•	✅ CLAUDE.md (your “robotlab operating manual” for the agent)  ￼
	•	✅ Small configs, launch files, schemas
	•	❌ Large datasets, rosbag2 logs, sim cache directories, build artifacts

AWS workstation (persistent disk)
	•	✅ Working clone of repo
	•	✅ Isaac Sim / Gazebo / PX4 SITL runtime
	•	✅ Docker images / ROS build caches (optional but nice)
	•	✅ rosbag2 logs (local), plus periodic export to S3 if you want
	•	✅ Claude Code local state:
	•	~/.claude and ~/.claude.json
	•	per-repo .claude/ directory  ￼
	•	✅ tmux sessions, scripts, and fast iteration tooling

Laptop
	•	✅ Thin client access:
	•	SSH / VS Code Remote SSH
	•	streaming client for Isaac Sim (or DCV)
	•	✅ Optional local clone used for:
	•	reviewing diffs
	•	writing docs
	•	doing “offline” refactors while the instance is stopped
	•	✅ Your long-lived personal secrets/password manager

⸻

How to set up the recommended workflow (practical checklist)

On the AWS workstation
	1.	Install Claude Code (native installer) and verify with claude doctor
Claude Code supports Ubuntu 20.04+ and macOS 10.15+, so your workstation OS should be fine.  ￼
	2.	Create a repo-root CLAUDE.md and check it in
Use it to pin commands, launch sequences, safety rules, and “how to run the demo.” Anthropic recommends CLAUDE.md as the place to store this.  ￼
	3.	Add a .claude/settings.json (repo-local) with a conservative allowlist
Anthropic documents that you can curate allowed tools via /permissions and by editing .claude/settings.json or ~/.claude.json.  ￼
Start conservative, then gradually “always allow” safe commands you use constantly (e.g., read-only commands; or file edits; but be careful with rm -rf class commands).
	4.	Make persistence explicit
Ensure your home directory and repo are on a persistent disk/volume so you don’t lose:

	•	your repo clone
	•	rosbag2 logs
	•	~/.claude session history and settings  ￼

	5.	Run Claude Code inside tmux
This avoids losing your session if your SSH drops, and it fits naturally with long-running ROS/sim processes.
	6.	Use least-privilege GitHub auth on the workstation
Prefer:

	•	a repo-scoped deploy key, or
	•	a fine-scoped token, or
	•	gh auth login with minimal permissions
(You’re minimizing blast radius if the instance is ever compromised.)

On your laptop
	1.	Set up an SSH config entry (and optionally VS Code Remote SSH)
You want “one command to get into the workstation.”
	2.	Optionally keep a local clone
Use it for:

	•	reviewing changes
	•	writing docs
	•	quick refactors while the instance is off
But treat the remote clone as the “real dev environment” for integration work.

	3.	Keep the GPU instance off when you’re not doing sim
If you want cost control, don’t fight the workflow—just adopt a habit:

	•	stop when done
	•	start when you need sim

⸻

A “better-than-either-extreme” variation (if cost becomes a pain)

Once you’re rolling, you can split into two AWS machines:
	•	Cheap CPU instance (always-on or often-on): editing, Claude Code, builds, unit tests
	•	GPU instance (on-demand): Isaac Sim, headless training, heavy rendering

Both mount the same persistent storage (or you sync via Git + S3 for artifacts). This gives you remote-first parity without paying GPU prices for “editing time.” It’s extra plumbing, but it’s the best long-term cost/perf pattern.

⸻

Bottom line

If your goal is dopamine + momentum and you’re serious about sim-to-real:

Run Claude Code on the AWS workstation (where sim and ROS run), keep GitHub as the sync backbone, and use your laptop as a thin client + review station.
This keeps your iteration loop tight and avoids the most common robotics productivity killer: developing in an environment that doesn’t match where the robot/sim actually runs.

If you want, I can also provide a “remote workstation bootstrap script” (idempotent) that:
	•	installs Claude Code
	•	installs your repo prerequisites
	•	sets up tmux helpers
	•	configures safe .claude/settings.json
	•	clones your repo and sets up your make demo-* targets so you can get back to dopamine immediately after starting the instance.