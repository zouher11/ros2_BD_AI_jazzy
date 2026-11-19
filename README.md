# SLAM Mapping TP — Ubuntu 24.04 / ROS 2 Jazzy / Webots R2025a

**Practical setup and troubleshooting guide**  
**Based on:** [sridath28/ros2_BD_AI](https://github.com/sridath28/ros2_BD_AI), folder `slam_mapping_tp_student`  
**Documented environment:** Ubuntu 24.04.5 LTS, ROS 2 Jazzy, Webots R2025a, VirtualBox  
**Status reached during troubleshooting:** Webots opens; external robot controller connects; `/clock`, `/scan`, `/odom`, and TF are published; keyboard teleoperation responds; SLAM Toolbox, after being manually configured and activated, begins displaying a map in RViz. **Map quality, all-room exploration, and final saved deliverables have not yet been validated.**

> **Read first — reproducibility limitation:** This README documents the **locally adapted, working copy**, not an unmodified upstream checkout. The public professor repository uses older Webots launch APIs and lacks the local controller/package setup demonstrated in this session. A person starting from a fresh clone must obtain the corrected `lab_world.wbt`, `go2_sim.launch.py`, `controllers/go2_driver/go2_driver.py`, and the local `CMakeLists.txt`/`package.xml` (or recreate equivalent package files). The precise final contents of the user's VirtualBox files were not uploaded, so this guide records observed steps and expected configurations; it is **not** a byte-for-byte patch of that machine.

---

## Table of contents

1. [Project overview](#1-project-overview)
2. [Environment and prerequisites](#2-environment-and-prerequisites)
3. [Quick start after restarting Ubuntu](#3-quick-start-after-restarting-ubuntu)
4. [How the three terminals communicate](#4-how-the-three-terminals-communicate)
5. [What was changed from the professor's version](#5-what-was-changed-from-the-professors-version)
6. [First-time setup and rebuilding](#6-first-time-setup-and-rebuilding)
7. [Diagnostics and known fixes](#7-diagnostics-and-known-fixes)
8. [RViz / SLAM troubleshooting](#8-rviz--slam-troubleshooting)
9. [Saving the map and lab outputs](#9-saving-the-map-and-lab-outputs)
10. [Shutdown, cleanup, and project handoff](#10-shutdown-cleanup-and-project-handoff)
11. [References and status notes](#11-references-and-status-notes)

---

## 1. Project overview

The professor's TP is a 2D SLAM exercise in a simulated laboratory (nominally about 10 m × 8 m). It uses a **Go2-like differential-drive model** (rectangular chassis, two driven wheels, a caster, LiDAR, and a camera). **It is not a full Unitree Go2 quadruped simulation.** The static world contains walls and box obstacles; the class assignment also includes a separate dynamic-obstacle mapping exercise.

The professor's original **three commands must remain**:

```bash
ros2 launch slam_mapping_tp go2_sim.launch.py
ros2 launch slam_mapping_tp slam_online_async.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

They are run in **different terminals** after sourcing ROS 2 and the workspace. On the current adapted installation, a **SLAM lifecycle configure/activate step** is additionally necessary after Terminal 2 starts. This can be done in Terminal 3 before starting keyboard control; no fourth persistent terminal is required.

### Directory used in this troubleshooting session

```text
~/ros2_ws/
├── src/
│   └── ros2_BD_AI/
│       └── slam_mapping_tp_student/
│           ├── worlds/lab_world.wbt
│           ├── controllers/go2_driver/go2_driver.py    # local standalone driver
│           ├── launch/go2_sim.launch.py                # edited for Jazzy
│           ├── launch/slam_online_async.launch.py      # professor's filename retained
│           ├── config/slam_params.yaml
│           ├── config/slam_config.rviz
│           ├── CMakeLists.txt                          # present in local package
│           └── package.xml                            # present in local package
└── install/
```

The original repository is [github.com/sridath28/ros2_BD_AI](https://github.com/sridath28/ros2_BD_AI). Its README contains a **different clone URL**; for this documented project use the repository above. A clone of the public repo alone may not build as a ROS package because `CMakeLists.txt` and `package.xml` were not available in the checked upstream folder; the local working copy contained them.

---

## 2. Environment and prerequisites

### Tested local environment

| Item | Value / observation |
|---|---|
| Host laptop | AMD Ryzen 7 PRO 4750U with integrated AMD Radeon graphics; no discrete GPU |
| Host RAM | 16 GB total (reported previously) |
| Virtual machine | Oracle VirtualBox, Ubuntu 24.04.5 LTS |
| VM allocation | 4 virtual CPU cores and 6 GB RAM (increased from 2 cores) |
| VM video memory | Increased from a low initial setting; 128 MB was recommended (recheck actual VM setting) |
| Guest graphics | `llvmpipe`, Mesa software rendering, OpenGL 3.3; **not** hardware accelerated |
| ROS 2 | Jazzy |
| Webots | R2025a under `/usr/local/webots` |
| Project workspace | `~/ros2_ws` |

A discrete GPU is not required for the TP, but `llvmpipe` means 3D rendering consumes CPU and can be slow inside VirtualBox.

### Verify the environment

```bash
lsb_release -ds
nproc
free -h
source /opt/ros/jazzy/setup.bash
echo "$ROS_DISTRO"
/usr/local/webots/webots --help 2>&1 | head -n 12
ros2 pkg list | grep -E 'slam_toolbox|teleop_twist_keyboard|webots_ros2_driver|nav2_map_server'
glxinfo -B
```

If `glxinfo` is missing: `sudo apt update && sudo apt install -y mesa-utils`.

### Optional install commands for a Jazzy machine

These packages were detected on the working system; the commands below are **prerequisite examples, not a claim that every command was executed during the troubleshooting session**:

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-webots-ros2 \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-nav2-map-server \
  ros-jazzy-tf2-ros \
  python3-colcon-common-extensions \
  mesa-utils
```

If a package name is unavailable in your apt sources, inspect `apt-cache search ros-jazzy-webots` or install the applicable packages individually. Webots itself was already installed separately at `/usr/local/webots`; installation steps for that binary were not recorded.

---

## 3. Quick start after restarting Ubuntu

**Before launching:** ensure the VM is running, no old Webots process remains, and the adapted package has already been built. The companion file `START_COMMANDS.txt` is the copy/paste version of this section.

### Terminal 1 — Webots + external Python robot controller

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export WEBOTS_HOME=/usr/local/webots
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
ros2 launch slam_mapping_tp go2_sim.launch.py
```

Expected: Webots starts; the robot world is shown; log eventually reports **`'robot' extern controller: connected.`**. If the 3D view is blank, enable rendering in Webots, use Plain Rendering if necessary, and check the *main* 3D window rather than the separate `Camera: Robot` view. Leave Terminal 1 open and Webots in **Real-time** mode.

### Terminal 2 — SLAM Toolbox + RViz

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch slam_mapping_tp slam_online_async.launch.py
```

RViz may initially say **`Fixed Frame [map] does not exist`**. In this local configuration, SLAM's lifecycle node was observed to start **`unconfigured [1]`**. Continue with the next commands to activate it.

### Terminal 3 — activate SLAM, then start keyboard control

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Necessary with the current launch file if SLAM is still unconfigured:
ros2 lifecycle get /slam_toolbox
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox         # expect: active [3]

# Professor's original third command:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

If `/slam_toolbox` is **already active**, **skip** the lifecycle transitions and run the keyboard command. If `configure` fails, inspect Terminal 2 logs; do not proceed blindly to `activate`.

**Keyboard controls** (`teleop_twist_keyboard`): `i` forward; `j` rotate left; `l` rotate right; `k` stop; `,` backward. The terminal must have keyboard focus. The `currently: speed ... turn ...` message reports configured command speeds, **not measured physical speed**. Stop with `k` and `Ctrl+C` as needed. Start slowly in the simulation and check the robot does not tip or clip through obstacles.

### RViz once SLAM is active

1. **Global Options → Fixed Frame:** `map` (temporarily use `odom` for diagnostics only).
2. **Map → Topic:** `/map`.
3. **LaserScan → Topic:** `/scan`.
4. Use an overhead RViz view. As the robot explores, the occupancy grid should gradually fill in; this was observed to begin working in the local run.

> **Important:** The manual lifecycle transitions are currently part of the working startup. We discussed a *future* permanent auto-activation change to `slam_online_async.launch.py`, but we did **not** verify that this change was made. Do not assume lifecycle activation is automatic after reboot.

---

## 4. How the three terminals communicate

```text
TERMINAL 3                             TERMINAL 1
Keyboard teleop ───── /cmd_vel ──────► Go2 Python controller
                                           │
                                          Webots
                                           │
                             /scan, /odom, /tf, /clock
                                           │
                                           ▼
                                      TERMINAL 2
                                    SLAM Toolbox
                                           │
                                     /map + map→odom
                                           │
                                          RViz
```

- `/cmd_vel` — incoming linear/angular motion commands.
- `/scan` — robot's 2D laser distances.
- `/odom` — wheel-encoder-based estimated motion.
- `/tf`, `/tf_static` — transforms between coordinate frames.
- `/clock` — Webots simulation time; SLAM and RViz must use it.
- `/map` — occupancy grid produced by active SLAM Toolbox.
- `/camera/image_raw` — topic was advertised by the local Python driver; the supplied controller excerpt did **not** include publishing image messages, so **do not assume live camera images are available**.

The TF chain required for mapping is **`map → odom → base_link → base_footprint`** (the `base_footprint` link is a static transform in the adapted launch). The published `/scan` used `base_link` as its frame. Validate all frames and spatial axes before treating a map as accurate. The Webots world uses `NUE` (Y-up), while ROS 2 odometry conventions use XY in the ground plane; coordinate alignment should be independently checked.

---

## 5. What was changed from the professor's version

This section separates original upstream facts from the locally applied fixes reported in the troubleshooting conversation. Some exact *final* local file values remain unverified until a `git diff` or uploaded copy is available.

### 5.1 `worlds/lab_world.wbt`: Webots R2025a compatibility

**Original upstream file begins with an invalid header:**

```text
#VR_SIM_V2023a utf8 86400 0
```

The installed Webots R2025a required:

```text
#VRML_SIM R2025a utf8
```

This header fix removed the initial parse failure. The upstream world also contained:

- A duplicated `coordinateSystem "NUE"` field; duplicate was removed in the local repair.
- A `ContactProperties` `friction` field that was adapted to `coulombFriction`.
- `TexturedBackground`/`TexturedBackgroundLight` without the required R2025a PROTO declarations; `EXTERNPROTO` declarations were added from the official `cyberbotics/webots/R2025a` background PROTO paths.
- `Lidar.verticalFieldOfView 0.0`, which produced a Webots warning; it was changed to a small **positive** value (the local fix targeted `0.1`).
- A top-level `DEF WALL_APPEARANCE PBRAppearance` definition that required adjustment for a valid 2025a scene; its first appearance was moved inside a shape in the suggested patch.
- Duplicate unnamed `Solid` siblings (`solid` warnings); unique names such as `left_wheel`, `right_wheel`, and `rear_caster` were proposed/applied through the physics repair.

**Do not run generic search-and-replace patches repeatedly** on a file already corrected. Check it first:

```bash
WORLD="$HOME/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student/worlds/lab_world.wbt"
head -n 8 "$WORLD"
grep -nE 'coordinateSystem|coulombFriction|EXTERNPROTO|verticalFieldOfView|controller' "$WORLD"
```

### 5.2 World physics and visibility

**Problems seen in the original professor world:**

| Area | Original characteristic | Local repair / lesson |
|---|---|---|
| Floor | Visual `Plane`, no `boundingObject` | Replaced with a horizontal collision-capable `Box` ground (10 × 0.1 × 8 m in the suggested patch). |
| Outer/internal walls | Rendered as boxes but no collision shapes | Added corresponding `boundingObject Box` geometries in the suggested repair. |
| Robot body | Chassis shape and bounding box not aligned vertically | Raised/aligned chassis and collision shape in the suggested repair. |
| Wheels | Joint axes `0 1 0` | Adapted joints toward X-axis wheel axles, with matching wheel geometry in the suggested repair. **Inspect final local wheel rotations and test physically**; an earlier explanation about cylinder orientation was contradictory. |
| Wheel sizes | World wheel radius 0.05 m; wheel spacing ~0.36 m | Synced driver calculations from radius 0.08 → 0.05 m and wheelbase 0.30 → 0.36 m. |
| Caster | Passive sphere with physics | Added unique name; caster collision/physics was changed in the suggested repair. Verify stability in current world. |
| Camera / LiDAR | Camera could show sky/mountains | Sensor pose/orientation was modified in the suggested repair; do not mistake the `Camera: Robot` sensor window for the main Webots scene. |
| Viewpoint | Professor's original `Viewpoint` could be confusing | Changed to a more helpful oblique view; final manually selected angle can differ. |

The **initial robot pose** from the professor's world is approximately `translation -1 0 -3.4` with no yaw rotation. Live position changes (for example a small `Y ≈ -0.00026` after testing) are not automatically faults. A previous value `Y ≈ +0.638` was recorded after keyboard movement and is **not sufficient evidence by itself** of a fall or physics failure.

**Inspect selected scene nodes instead of editing blindly:**

```bash
WORLD="$HOME/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student/worlds/lab_world.wbt"
sed -n '/DEF GO2_ROBOT Robot/,$p' "$WORLD"
grep -nE 'name "floor"|boundingObject|axis |translation |rotation ' "$WORLD" | head -n 100
```

In Webots, select `DEF GO2_ROBOT` in the **Scene Tree**; move the main Viewpoint to that object and inspect **live** `translation`/`rotation`. Pause and resume to distinguish physics behavior from keyboard movement. For collision debugging, enable **Optional Rendering → Show All Bounding Objects**. Webots' `Camera: Robot` view is just a camera sensor view and may point at sky; it does not tell you where the robot is on the map.

### 5.3 `launch/go2_sim.launch.py`: old ROS/Webots APIs replaced

**Upstream** calls the now-absent `webots_ros2_core/launch/webots_launch.py` and an executable `webots_ros2_universal_robot/universal_robot` that was not available in the installed Jazzy package. The observed local package instead provides `abb_controller` and `ur5e_controller` in `webots_ros2_universal_robot`.

**Local working strategy:** keep the professor's **filename and CLI command**, but inside that file:

- Use `WebotsLauncher` from `webots_ros2_driver.webots_launcher`.
- Load the **installed** package's `worlds/lab_world.wbt` using `get_package_share_directory('slam_mapping_tp')`.
- Start an **external standalone Python controller** through `ExecuteProcess`, invoking `/usr/local/webots/webots-controller` with `--protocol=ipc`, `--port=1234`, and `--robot-name=robot`.
- Delay controller startup briefly (`TimerAction(period=10.0, ...)` was used in the local test).
- Publish the static transform `base_link → base_footprint`.
- Keep `WEBOTS_HOME=/usr/local/webots` and the window-rendering environment variables.

The `WebotsLauncher` can launch a *temporary* file named like `/tmp/tmp…_world_with_URDF_robot.wbt`. This is **normal**; the source to edit remains `~/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student/worlds/lab_world.wbt`.

**Check installed launch configuration:**

```bash
LAUNCH="$(ros2 pkg prefix slam_mapping_tp)/share/slam_mapping_tp/launch/go2_sim.launch.py"
grep -nE 'WebotsLauncher|go2_driver.py|webots-controller|port=|robot-name|TimerAction' "$LAUNCH"
```

**Important:** A `WebotsController` ROS plugin is a different architecture. Your local `go2_driver.py` imports `controller.Robot` and works as a **standalone Webots Python controller**; merely replacing the old ROS executable with `webots_ros2_driver/driver` does not adapt this Python class into a ROS plugin. Keep the working external-controller architecture until a deliberate, tested migration is made.

### 5.4 `controllers/go2_driver/go2_driver.py`: local robot-to-ROS bridge

This standalone driver was present in the local project (not confirmed in the upstream public files). It:

- Connects to Webots with `from controller import Robot`.
- Reads left/right wheel position sensors and drives wheel motors.
- Publishes `/clock`, `/scan`, `/odom`, and `/tf`.
- Subscribes to `/cmd_vel`.
- Creates a publisher for `/camera/image_raw` but, in the version shared in the chat, **does not actually publish camera frames**.
- Used `wheel_radius=0.08` and `wheel_base=0.30` originally; values were adapted to `0.05` and `0.36` for the simulated wheels (confirm the exact latest local file).

The final physical movement and mapping outputs suggest the bridge is operating, but odometry scale and coordinate alignment still deserve calibration if this becomes a research project or is deployed to real hardware.

### 5.5 `launch/slam_online_async.launch.py`: lifecycle issue identified

The professor's launch file starts `async_slam_toolbox_node` as a normal ROS 2 `Node`, plus RViz; it **does not include automatic lifecycle configure/activate actions**. On the tested machine the node started `unconfigured [1]`, with missing frame parameters and **zero publishers** on `/map`.

**Confirmed fix during troubleshooting:**

```bash
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox  # active [3]
```

After activation, the user reported that the map began forming in RViz as the robot explored.

**Not yet implemented/verified:** automatic lifecycle transitions in `slam_online_async.launch.py`. Such a future enhancement should preserve the **same filename and professor's command**. Until tested, use the three-terminal startup workflow with manual activation in Terminal 3.

### 5.6 Experimental files archived

Extra experimental launch files `go2_sim2.launch.py` and `bridge_only.launch.py` were not part of the desired professor workflow. The suggested safe archival command was:

```bash
PROJECT="$HOME/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student"
BACKUP="$HOME/ros2_tp_archive"
mkdir -p "$BACKUP"
for file in go2_sim2.launch.py bridge_only.launch.py; do
    if [ -f "$PROJECT/launch/$file" ]; then
        mv -n "$PROJECT/launch/$file" "$BACKUP/"
    fi
done
ls "$PROJECT/launch"
```

**Archive, don't destroy**, until the primary workflow has been preserved in version control.

---

## 6. First-time setup and rebuilding

### Get the public professor repository (new machine)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/sridath28/ros2_BD_AI.git
```

**This alone is not yet a complete Jazzy install.** Transfer the **locally adapted** project package files, including `CMakeLists.txt`, `package.xml`, corrected `worlds/lab_world.wbt`, current `launch/go2_sim.launch.py`, and `controllers/go2_driver/go2_driver.py` from the working VM/project. Do **not** overwrite them with the original upstream files. In particular, the publicly inspected repository did not expose an upstream `CMakeLists.txt` or `package.xml` in that folder.

To confirm package install rules on the working machine:

```bash
PROJECT="$HOME/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student"
ls -l "$PROJECT/CMakeLists.txt" "$PROJECT/package.xml"
grep -n -A 10 -B 3 'install(' "$PROJECT/CMakeLists.txt"
```

The installed package needs to contain the world, config, launch files and controller. An `ament_cmake` project can install these directories using its own existing `CMakeLists.txt` rules. **Do not replace a working `CMakeLists.txt` with guessed boilerplate.**

### Rebuild after changing source files

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
python3 -m py_compile src/ros2_BD_AI/slam_mapping_tp_student/controllers/go2_driver/go2_driver.py
colcon build --symlink-install --packages-select slam_mapping_tp
source install/setup.bash
ros2 pkg prefix slam_mapping_tp
```

Verify installed assets:

```bash
SHARE="$(ros2 pkg prefix slam_mapping_tp)/share/slam_mapping_tp"
ls -l "$SHARE/worlds/lab_world.wbt"
ls -l "$SHARE/launch/go2_sim.launch.py"
ls -l "$SHARE/launch/slam_online_async.launch.py"
ls -l "$SHARE/controllers/go2_driver/go2_driver.py"
head -n 1 "$SHARE/worlds/lab_world.wbt"
```

`head -n 1` should print `#VRML_SIM R2025a utf8`. After a VM shutdown with **no changes**, there is no need to rebuild; simply source both ROS environments and run the launch commands.

### Keep backups when editing

```bash
PROJECT="$HOME/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student"
cp "$PROJECT/worlds/lab_world.wbt" "$PROJECT/worlds/lab_world.wbt.backup.$(date +%Y%m%d_%H%M%S)"
cp "$PROJECT/launch/go2_sim.launch.py" "$PROJECT/launch/go2_sim.launch.py.backup.$(date +%Y%m%d_%H%M%S)"
```

If the repo is under Git, capture a diff of your real changes for reproducibility:

```bash
cd ~/ros2_ws/src/ros2_BD_AI
git status --short
git diff -- slam_mapping_tp_student/
```

New untracked files will **not** appear in `git diff` unless added to Git; inspect `git status --short` too.

---

## 7. Diagnostics and known fixes

### A. Webots wouldn't launch: invalid header

**Error:** `Invalid header`, `Please modify the first line ... to "#VRML_SIM R2025a utf8"`.

**Fix:** correct the header in the source world (backup first), rebuild, relaunch. Later errors included missing `TexturedBackgroundLight` PROTO and an invalid LiDAR vertical FOV. These were separately fixed, as described in §5.

### B. Transparent/black Webots viewport

Observed VirtualBox guest:

```text
OpenGL renderer string: llvmpipe (LLVM 20.1.2, 256 bits)
Accelerated: no
OpenGL version string: 3.3 ... Mesa
```

Software rendering was used as a reliable fallback:

```bash
export QT_QPA_PLATFORM=xcb
export LIBGL_ALWAYS_SOFTWARE=1
```

VM allocation was increased to **4 CPUs + 6 GB RAM**. Video memory **128 MB** and VMSVGA were suggested; verify what was actually applied in VirtualBox. Webots has a rendering toggle; if the viewport says **No Rendering**, enable rendering in Webots. A built-in sample world was also initially black until rendering was enabled, after which it displayed correctly.

**Avoid using** `--no-rendering` for GUI validation. `llvmpipe` performs graphics rendering on the CPU, so performance can be limited in VirtualBox. The `glxinfo` reported `5848MB` was **not** dedicated GPU VRAM.

### C. Controller waiting / `Giving up...`

An initial manual `webots-controller` test failed to locate a ready robot. Later the adapted `WebotsLauncher` + external controller connected on **port 1234**, reporting:

```text
INFO: 'robot' extern controller: connected.
```

Useful checks:

```bash
pgrep -af '[w]ebots'
find "/tmp/webots/$USER/1234" -maxdepth 5 -print 2>/dev/null
ros2 node list
ros2 topic list -t
```

Seeing `/tmp/webots/<user>/1234/live.txt` only confirms simulator runtime files exist. Seeing the `ipc/robot/extern` path is encouraging but **does not alone prove a ROS node is publishing**. Confirm the connected message **and** live `/scan` and `/odom` data.

Two processes such as `/usr/local/webots/webots` and `.../webots-bin` normally mean a wrapper script plus the simulator, not two competing simulators. An extra abandoned `webots-bin --no-rendering --minimize` process **can** interfere and should be closed before relaunching.

### D. ROS topics and actual messages

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 node list
ros2 topic list -t
ros2 topic info /scan
ros2 topic info /cmd_vel -v
timeout 10s ros2 topic echo /clock --once
timeout 10s ros2 topic echo /scan --once --field header
timeout 10s ros2 topic echo /odom --once --field pose.pose.position
```

**Observed working example:** `/go2_driver` exists; `/scan` has one publisher; `/clock` contains a Webots timestamp; `/odom` publishes changing estimated positions. **Odometry changing is not proof of physically correct motion**: inspect the actual robot in Webots as well.

### E. TF validation

```bash
timeout 8s ros2 run tf2_ros tf2_echo odom base_link
timeout 8s ros2 run tf2_ros tf2_echo odom base_footprint
timeout 8s ros2 run tf2_ros tf2_echo map odom
```

An *initial* `Invalid frame ID "odom"` warning can occur while data/TF is starting. In the observed test, `tf2_echo odom base_link` subsequently printed valid transforms. `map → odom` will be missing until SLAM Toolbox activates and computes a map.

### F. Terminal appears frozen / no scrolling

- `Ctrl+C` stops the foreground ROS command.
- `Ctrl+Q` resumes output accidentally paused with `Ctrl+S`.
- `F11` toggles many terminals' full-screen mode.
- `Ctrl+Alt+T` opens a new Ubuntu terminal.
- If a prior teleop instance remains, stop it specifically rather than killing all ROS software:

```bash
pkill -INT -f '[t]eleop_twist_keyboard'
```

### G. Clean up leftover simulator processes (only after exiting launch commands)

```bash
pgrep -af '[w]ebots'
```

If an old instance is genuinely stuck, stop the identified PID using `kill <PID>`, check again, then use a stronger signal only if necessary. Avoid indiscriminate `pkill -9` of all project processes while mapping is in progress.

---

## 8. RViz / SLAM troubleshooting

### The exact failure observed

```text
ros2 lifecycle get /slam_toolbox    → unconfigured [1]
ros2 param get /slam_toolbox map_frame → Parameter not set
ros2 topic info /map                → Publisher count: 0
RViz Fixed Frame                    → map does not exist
```

`/scan` had **one publisher and one subscriber**, and `odom → base_link` was available, so the immediate missing piece was an **unconfigured SLAM lifecycle node**, not a broken Webots connection.

### Confirmed manual lifecycle repair

With Webots and the SLAM launch both running:

```bash
ros2 lifecycle get /slam_toolbox
ros2 lifecycle set /slam_toolbox configure
ros2 lifecycle set /slam_toolbox activate
ros2 lifecycle get /slam_toolbox
```

Expected `active [3]`. After that, check:

```bash
ros2 param get /slam_toolbox use_sim_time
ros2 param get /slam_toolbox map_frame
ros2 param get /slam_toolbox odom_frame
ros2 param get /slam_toolbox base_frame
ros2 topic info /map
timeout 8s ros2 run tf2_ros tf2_echo map odom
```

The professor's `config/slam_params.yaml` specifies:

```yaml
slam_toolbox:
  ros__parameters:
    scan_topic: /scan
    map_frame: map
    odom_frame: odom
    base_frame: base_footprint
    use_sim_time: true
```

The *real* YAML also has mapping and solver parameters; don't replace it with only this excerpt. Parameter availability before configuration is not sufficient evidence that the YAML was deleted.

**If map is still empty after active:** make sure the simulation is in Real-time, `/scan` messages continue, `/clock` advances, TF is valid, and drive slowly. In RViz use `map` as Fixed Frame; if it has not yet appeared, `odom` can be a temporary visualization frame, **not a fix**. The map started appearing in the successful observed session, but final map quality and frame calibration were not audited.

### Why RViz can be black even though it launches

RViz can open and render its background with OpenGL 3.3 while no useful map exists. Informational `Stereo is NOT SUPPORTED` logs are **not** the cause of the missing `map` frame. Always check `/map` publishers and SLAM lifecycle before changing graphics settings.

### Note about RobotModel

The professor's RViz profile expects `/robot_description`. The standalone Python controller need not publish this. A blank or erroneous **RobotModel** panel does not mean LiDAR-based SLAM has failed; TF, LaserScan and Map are the essential initial checks.

---

## 9. Saving the map and lab outputs

Once the map is stable and the robot has explored enough of the accessible environment, save it using the professor's designated Nav2 map-saving command:

```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
PROJECT="$HOME/ros2_ws/src/ros2_BD_AI/slam_mapping_tp_student"
mkdir -p "$PROJECT/maps"
ros2 run nav2_map_server map_saver_cli -f "$PROJECT/maps/lab_static"
ls -lh "$PROJECT/maps/lab_static.pgm" "$PROJECT/maps/lab_static.yaml"
```

Expected files: `lab_static.pgm` (map image) and `lab_static.yaml` (map metadata). This was part of the professor's assignment, **not reported as completed in our troubleshooting session**.

For Part B (dynamic obstacles), preserve the completed static world and mapping outputs first. The professor asks students to modify the world to introduce moving obstacles, remap, and save `maps/lab_dynamic.pgm` and `lab_dynamic.yaml`. **Actual Part B implementation was not covered in this session.**

Professor's deliverables include map files, RViz screenshots, edited dynamic world and a short comparison report. Follow the course handout and `SUBMISSION.md` for academic requirements; don't submit this troubleshooting guide in place of the required mapping analysis.

---

## 10. Shutdown, cleanup, and project handoff

### End a session normally

1. Stop keyboard teleop (`k`, then `Ctrl+C`).
2. Stop SLAM/RViz (Terminal 2 `Ctrl+C`).
3. Stop Webots (Terminal 1 `Ctrl+C`).
4. Close remaining Webots windows if necessary.
5. Shut down Ubuntu normally.

**After the next reboot:** you need only the three commands in §3, **plus manual SLAM activation** if still unconfigured; there is no need to re-run setup patches or `colcon build` if no files changed.

### Save the reproducible *adapted* project, not only the README

Before sharing it with another Jazzy student, preserve the **actual local files**:

```text
slam_mapping_tp_student/
  CMakeLists.txt
  package.xml
  worlds/lab_world.wbt
  controllers/go2_driver/go2_driver.py
  launch/go2_sim.launch.py
  launch/slam_online_async.launch.py
  config/slam_params.yaml
  config/slam_config.rviz
  README_JAZZY.md (copy this README here, if desired)
  START_COMMANDS.txt
```

Also capture `git diff` and list of untracked additions. The public professor repository alone does **not** reflect all the patches needed for the observed Jazzy run. Avoid publishing a modified assignment solution if your course's collaboration / academic-integrity rules prohibit it.

### Future improvements (not yet verified)

- Make `slam_online_async.launch.py` automatically configure and activate SLAM Toolbox, **keeping the same command name**.
- Recheck robot wheel axes, physical translation versus odometry, and TF consistency.
- Validate the physical floor and wall collisions and doorway geometry throughout the world.
- Correct/publish camera images if required by a later exercise.
- Save and compare static and dynamic maps.
- For future *real robot* deployment, replace the simulated hardware adapter with actual hardware drivers and validate timing, frames, sensor calibration and safety. The current differential-drive simulation is **not** a drop-in Unitree Go2 controller.

---

## 11. References and status notes

### Primary project source

- Professor repository: <https://github.com/sridath28/ros2_BD_AI>
- Original handout README: <https://github.com/sridath28/ros2_BD_AI/blob/main/slam_mapping_tp_student/README.md>
- Original world: <https://github.com/sridath28/ros2_BD_AI/blob/main/slam_mapping_tp_student/worlds/lab_world.wbt>
- Original Webots launcher: <https://github.com/sridath28/ros2_BD_AI/blob/main/slam_mapping_tp_student/launch/go2_sim.launch.py>
- Original SLAM launcher: <https://github.com/sridath28/ros2_BD_AI/blob/main/slam_mapping_tp_student/launch/slam_online_async.launch.py>
- SLAM params: <https://github.com/sridath28/ros2_BD_AI/blob/main/slam_mapping_tp_student/config/slam_params.yaml>
- Course submission requirements: <https://github.com/sridath28/ros2_BD_AI/blob/main/slam_mapping_tp_student/SUBMISSION.md>

### Upstream documentation

- Webots R2025a: <https://cyberbotics.com/doc/guide/index>
- Webots external controllers: <https://github.com/cyberbotics/webots/blob/master/docs/guide/running-extern-robot-controllers.md>
- ROS 2/Webots integration: <https://github.com/ros2/ros2_documentation/blob/rolling/source/Integrations-and-related/Simulators/Webots/Setting-Up-Simulation-Webots-Basic.rst>
- SLAM Toolbox lifecycle reference: <https://github.com/SteveMacenski/slam_toolbox/blob/ros2/launch/online_async_launch.py>
- Nav2 map saver: <https://api.nav2.org/nav2-jazzy/html/md_nav2_map_server_README.html>

### Evidence for successful intermediate milestones

- `INFO: 'robot' extern controller: connected.` was observed.
- `ros2 node list` included `/go2_driver`.
- `/scan` had one publisher; `/clock`, `/scan` header, and `/odom` returned data.
- Keyboard movement was reported to work when Terminal 3 had focus.
- Initial RViz fixed-frame error was traced to lifecycle state `unconfigured [1]` and `/map` with zero publishers.
- After `configure` + `activate`, the user reported **the SLAM map beginning to fit/form as the robot explored**.

**Last updated:** 19 November 2026 — documentation compiled from the debugging session and a contemporaneous inspection of the professor's public files. This guide intentionally distinguishes **confirmed observations**, **reported local edits**, and **remaining validation tasks**.
