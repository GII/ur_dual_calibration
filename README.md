<div align="center">

<h1 align="center">UR Dual Calibration</h1>

<p align="center"><i>Hand-eye calibration toolkit for the OAK-D camera and the dual UR5 + SoftHand system</i></p>

</div>

## 👋 Welcome

This repository provides the full pipeline to compute and publish the **hand-eye calibration** between an externally mounted **OAK-D camera** and the **dual UR5 / UR5e** workstation with **qb SoftHand 2 Research** end-effectors. The calibration is **eye-to-hand** (camera fixed, marker mounted on the robot), and the final output is a static TF that connects the camera's frame tree to the robot's, enabling perception-driven manipulation pipelines.

It is built as a **companion package** for the dual UR5 base system maintained at GII:  
👉 [`SantaCRC/ur_softhand_dual`](https://github.com/SantaCRC/ur_softhand_dual/tree/SoftHand-jazzy) (branch `SoftHand-jazzy`)

The robot launches, controllers and MoveIt configuration come from that repo; this one only adds the calibration tooling and the static TF launch.

---

## 🧭 Workflow Overview

The calibration is split into clear, independent steps. Each script does **one thing** and can be re-run on its own:

```
1. Sanity check       →  detect_aruco_once.py
2. Sample capture     →  capture_sample.py        (manual)
                         auto_calibrate.py        (automated)
3. Hand-eye solve     →  run_handeye.py           (4 OpenCV methods)
4. Validation         →  validate_handeye.py      (self-consistency + overlays)
5. Export             →  export_handeye_yaml.py   (write handeye.yaml)
6. Publish static TF  →  ros2 launch ur_dual_calibration publish_handeye.launch.py
```

The system implements **eye-to-hand** by inverting the gripper→base transform before feeding OpenCV's `calibrateHandEye()` (which is documented for the eye-in-hand case). All four available methods (TSAI, PARK, HORAUD, DANIILIDIS) are computed and cross-checked; the PARK result is exported by default.

---

## 📂 Repository Overview

```
📂 ur_dual_calibration
 ├── scripts/                         # Standalone calibration tools (run with python3)
 │    ├── detect_aruco_once.py        # Camera + ArUco + solvePnP sanity check
 │    ├── capture_sample.py           # Manual single-sample capture (press Enter)
 │    ├── run_handeye.py              # OpenCV calibrateHandEye, all four methods
 │    ├── validate_handeye.py         # Self-consistency check + visual overlays
 │    ├── export_handeye_yaml.py      # Compose result with OAK internal TF
 │    ├── extract_poses_from_samples.py  # Build pose YAML for automation
 │    ├── auto_calibrate.py           # Automated / semi-automated run
 │    └── archive_calibration.py      # Snapshot previous session before recalibrating
 ├── ur_dual_calibration/             # ROS 2 package (ament_cmake)
 │    ├── config/
 │    │    ├── handeye.yaml           # Current published calibration
 │    │    └── calibration_poses.yaml # Recorded poses for automatic mode
 │    ├── launch/
 │    │    └── publish_handeye.launch.py
 │    ├── CMakeLists.txt
 │    └── package.xml
 ├── LICENSE
 └── README.md                        # You're here! 👋
```

---

## ⚙️ Requirements

✅ **ROS 2 Jazzy** (Humble should also work)  
✅ Workspace named `~/ws_daniel` (any name is fine, paths below assume this)  
✅ Base system from [`ur_softhand_dual`](https://github.com/SantaCRC/ur_softhand_dual/tree/SoftHand-jazzy) (dual UR drivers + MoveIt config)  
✅ **OAK-D** camera with `depthai_ros_driver`  
✅ Python 3.10+ with `opencv-python` (≥ 4.6), `numpy`, `pyyaml`, `cv_bridge`

---

## 🛠️ Setup & Installation

1️⃣ Clone the **base system** first (the dual UR + SoftHand stack):
```bash
# From: ~/ws_daniel/src
git clone -b SoftHand-jazzy https://github.com/SantaCRC/ur_softhand_dual.git
```

2️⃣ Clone this **calibration package** inside the base repo's `src/`:
```bash
# From: ~/ws_daniel/src/ur_softhand_dual/src
git clone https://github.com/GII/ur_dual_calibration.git
```

3️⃣ Install dependencies and build:
```bash
# From: ~/ws_daniel
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

---

## 🤖 Robot Initialization (UR5 Teach Pendant)

Follow this step-by-step guide to power on the UR5 robotic arm, initialize it, and establish the external control communication loop with the master PC.

### 1. System Power-Up

* **Step 01** — Power on the UR5 Control Box and wait for the PolyScope graphical interface to load. Tap the robot status indicator (bottom left corner) to access the initialization screen.

<img src="images/screenshot_0000.png" width="60%" alt="PolyScope Home Screen">

* **Step 02** — Turn on the robot electronics by pressing **ON**.

<img src="images/screenshot_0001.png" width="60%" alt="Initialization Menu">

* **Step 03** — Tap **START** to release the mechanical brakes. You will hear a distinct click from each joint.

<img src="images/screenshot_0002.png" width="60%" alt="Releasing Brakes">

* **Step 04** — Verify that the status indicator turns solid green and shows **Normal**. Return to the main menu.

<img src="images/screenshot_0003.png" width="60%" alt="Robot Status Normal">

### 2. Load the External Control Program

* **Step 05** — Select **Program Robot**.

<img src="images/screenshot_0004.png" width="60%" alt="Program Robot Menu">

* **Step 06** — Load the existing **URCaps → External Control** program.

<img src="images/screenshot_0005.png" width="60%" alt="Loading External Control Program">

* **Step 07** — Open the External Control node and check its parameters.

<img src="images/screenshot_0006.png" width="60%" alt="Verifying Network Parameters">

* **Step 08** — Double-check that the **PC IP address** and **port** match your local network setup. Leave the program ready on the Teach Pendant. **Do not press Play yet.**

<img src="images/screenshot_0007.png" width="60%" alt="Program Ready for Execution">

### 3. Launch the PC Driver and Connect

Open a terminal on the workstation and launch the dual-arm driver:

```bash
ros2 launch ur_dual_control start_robot.launch.py
```

Once the driver is up and waiting for the hardware connection, complete the loop on the Teach Pendant:

* **Step 09** — Press **Play** at the bottom of the Teach Pendant to establish the remote connection loop.

<img src="images/screenshot_0008.png" width="60%" alt="Press Play to Connect">


* **Step 10** — The program switches to active. The driver terminal on the PC reports a successful connection. The UR5 is now listening to external motion commands.

<img src="images/screenshot_0009.png" width="60%" alt="Driver Connected">

---

## 📷 Camera Setup

In a separate terminal, launch the OAK-D driver under the `oak_cam` namespace:

```bash
# From: any directory (workspace already sourced)
ros2 launch depthai_ros_driver camera.launch.py namespace:=oak_cam
```

---

## 🧪 Manual Calibration

Use this flow the **first time** you calibrate or after **any change** to the ArUco mounting hardware.

### 1. Verify ArUco detection

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/detect_aruco_once.py
```

Move the arm so the marker is visible. You should see `ArUco 100 OK | tvec=… | d=… m` lines at ~1 Hz. `Ctrl+C` to exit.

### 2. Capture samples

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/capture_sample.py
```

Move the arm via MoveIt to each pose and press **Enter** at the script prompt. Aim for ~20 poses with diverse orientations (different roll/pitch/yaw of the marker, not just translations). `Ctrl+C` to finish.

### 3. Solve hand-eye

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/run_handeye.py
```

Inspect the table printed in console. The four methods should agree to within a few millimeters.

### 4. Validate

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/validate_handeye.py
```

Check the self-consistency metrics and inspect a few overlays from `calibration_data/overlays/`. Green circles (detection) should sit on red crosses (reprojection).

### 5. Export the YAML

The OAK-D driver must be **running** for this step. Any previous `publish_handeye.launch.py` must be **stopped**.

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/export_handeye_yaml.py
```

This writes `ur_dual_calibration/config/handeye.yaml`.

### 6. Rebuild and publish

```bash
# From: ~/ws_daniel
colcon build --packages-select ur_dual_calibration --symlink-install
source install/setup.bash
ros2 launch ur_dual_calibration publish_handeye.launch.py
```

The static TF is now live. Verify with:

```bash
ros2 run tf2_ros tf2_echo ur_dual_I_base_link oak_rgb_camera_optical_frame
```

---

## ⚡ Automated Calibration

Once a successful manual calibration has been done, the joint configurations from the captured samples can be reused to run future sessions automatically.

### 1. Generate the pose file (once)

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/extract_poses_from_samples.py
```

This reads the existing `calibration_data/samples/*.json` and writes `ur_dual_calibration/config/calibration_poses.yaml`.

### 2. Archive previous data and run

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/archive_calibration.py
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/auto_calibrate.py --mode semi
```

The orchestrator moves the arm to each pose, captures samples, and runs `run_handeye`, `validate_handeye` and `export_handeye_yaml` at the end.

Modes:
- `--mode semi` — confirms each pose interactively (recommended first time)
- `--mode auto` — fully unattended
- `--max-reproj-px N` — rejection threshold (default 1.5)
- `--dry-run` — list poses without moving the robot

### 3. Rebuild and publish

Same as step 6 of the manual flow.

---

## 🔁 Recalibrating After Camera Movement

If only the **tripod was nudged or repositioned** (everything else unchanged), skip the manual flow and go straight to the automated path:

```bash
# From: ~/ws_daniel
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/archive_calibration.py
python3 src/ur_softhand_dual/src/ur_dual_calibration/scripts/auto_calibrate.py --mode auto
```

If the camera moved a lot and some pre-recorded poses no longer see the marker, the automator will skip them; the calibration may still succeed with the remaining valid samples.

---

## 🗂️ Generated Data

These directories are created automatically and **should not be committed**:

```
calibration_data/
 ├── samples/       # Per-capture JSON + PNG (one per pose)
 ├── results/       # handeye_results.json, validation_metrics.json
 ├── overlays/      # Per-sample reprojection overlay PNGs
 └── archive/       # Snapshots of previous sessions (calibration_YYYY-MM-DD_HH-MM/)
```

A working `handeye.yaml` is always kept in `ur_dual_calibration/config/` and **is committed**, it represents the current published calibration.

---

## 🩺 Troubleshooting

* **ArUco not detected** — check marker ID and dictionary in script constants; verify the `MARKER_LENGTH` matches the actual measured black-square side.
* **No `/oak_cam/...` topics** — the camera driver is not running, or the namespace differs. Re-launch with `namespace:=oak_cam`.
* **TF lookup fails for `ur_dual_I_tool0`** — the robot driver is not connected; verify the External Control program is running on the Teach Pendant.
* **Methods diverge by more than ~10 mm** — pose diversity is insufficient. Recapture with more varied rotations.
* **Calibration validates poorly (σ > 15 mm)** — inspect `calibration_data/overlays/`; an outlier sample can usually be identified visually and moved to `samples/rejected/` before re-running `run_handeye.py`.
* **Conflicting TF parent error on `oak_rgb_camera_optical_frame`** — `export_handeye_yaml.py` already handles this by composing the static TF to the `oak-d-base-frame` (the OAK subtree root). Don't try to publish directly to the optical frame.
* **`ros2 launch ur_dual_calibration ...` says package not found** — workspace not sourced after the build. Run `source install/setup.bash`.

---

## 📋 Best Practices

* Measure the black square of the printed ArUco with a **caliper**, not a ruler — the error propagates linearly into the calibration.
* Capture poses with **rotational diversity** (different roll, pitch and yaw of the marker), not just translations. Tsai-Lenz proved this is what makes the system observable.
* **Do not bump the tripod** mid-session. If you do, archive and restart.
* Always `archive_calibration.py` before a fresh run — never let new samples mix with old ones.
* Don't commit `calibration_data/` to the repo. The `.gitignore` already handles this.

---

## 🏆 Acknowledgments

Developed at the **Grupo Integrado de Ingeniería (GII), Universidade da Coruña** as part of a Final Degree Project (TFG) in collaboration with the Instituto Tecnológico de Costa Rica (TEC).

Built on top of [`ur_softhand_dual`](https://github.com/SantaCRC/ur_softhand_dual) by Fabián Álvarez ([@SantaCRC](https://github.com/SantaCRC)).

Special thanks to the GII Lab for hosting this research stay.

---

## 📜 License

MIT — see [`LICENSE`](LICENSE) for details.

---