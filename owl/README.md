# Owl: the Strix coprocessor app

Owl is the coprocessor half of Strix. Choate runs it on an Orange Pi 5 and NEIA on a Jetson Orin Nano. The robot-side half, which reads Owl's results and feeds the pose estimator, is Tusk; see [../tusk/README.md](../tusk/README.md).

For each camera, Owl runs a separate process that goes capture → AprilTag detection → pose solve → filtering → standard deviations. All cameras publish into one NetworkTables topic, `/Vision/observations`, so the robot reads it once per loop and calls `addVisionMeasurement()` for each result. Adding a camera means adding an entry to `config.json`; nothing else changes on either side.

Owl is controlled from one web page, similar to PhotonVision's: live streams, settings, resolution and camera calibration for every camera. Every setting can also be changed live over NetworkTables, from Elastic or AdvantageScope.

Owl uses the RobotPy 2026 libraries even when the robot runs WPILib 2027. [../docs/wpilib-2027.md](../docs/wpilib-2027.md) explains why.

## Files

```
owl/                      copy this folder to the coprocessor
  main.py                 starts a process per camera and publishes the combined topic
  camera_process.py       one camera's capture, detection and pose solve (its own process, so its own CPU core)
  pose_solver.py          multi-tag SQPNP and single-tag IPPE solves, filters, standard deviation factor
  cuda_detector.py        optional GPU tag detection on a Jetson (falls back to the CPU detector)
  config.py               loads config.json and calibrations (also reads PhotonVision calibration exports)
  settings.py             live settings from the web page and NetworkTables, and saving to config.json
  pipelines.py            named per-camera pipelines (sets of camera, detector and filter settings)
  network.py              NetworkTables target, hostname and IP settings from the coprocessor tab
  admin.py                maintenance buttons: logs, settings export and import, factory reset
  validation.py           range checks for every setting
  recovery.py             restarts stalled or crashed camera processes
  stream_server.py        per-camera web server: MJPEG streams and the settings and calibration API
  web_ui.html             the web page
  calibration_session.py  in-browser ChArUco calibration
  camera_controls.py      exposure, gain and resolution (v4l2-ctl on Linux, OpenCV otherwise)
  quality_recording.py    optional recording of tag corners and fit quality for tuning
  vision_types.py         the NetworkTables message format (identical to tusk/robotpy/tusk/vision_types.py)
  test_synthetic.py       renders tags from known poses and checks the math
  list_cameras.py         finds cameras and saves a snapshot from each
  calibrate.py            ChArUco calibration from a terminal (alternative to the web page)
  config.example.json     start here: cp config.example.json config.json
  configs/7407.json       Choate's tuned config (copy to config.json on their Pi)
  calibrations/           <camera>_<W>x<H>.json, one per camera per resolution (not in git)
../deploy/owl.service     systemd service that starts Owl on boot
../deploy/owl-admin       root helper for hostname, IP and reboot (optional; see section 6)
```

## Developing on a Mac

Everything runs on macOS, and it's the easiest place to iterate: RobotPy robot simulation, Owl and AdvantageScope all on one machine.

```bash
cd owl
pip3 install -r requirements.txt
python3 test_synthetic.py          # should print PASS
python3 list_cameras.py            # shows which index is which camera, and saves a snapshot from each
```

The first time Python opens a camera, macOS asks for camera permission for whatever app launched it (Terminal, iTerm, VS Code). If `list_cameras.py` finds nothing, check System Settings > Privacy & Security > Camera.

On a Mac, use camera indices in `config.json` (`"device": 0`), not `/dev/...` paths. The order can change when you replug cameras, and the built-in camera or a nearby iPhone (Continuity Camera) may take index 0. Manual exposure and the resolution list only work on Linux; on macOS cameras run on auto exposure. A video file also works as a `device` (add `"loop_video": true`), which is handy for trying the web page and calibration without a camera.

```bash
# Vision only: Owl is the NetworkTables server. Point AdvantageScope at localhost.
python3 main.py --config config.json --standalone

# With robot code: start the robot simulation first (it becomes the NT server)...
python -m robotpy sim
# ...then, in another terminal, connect Owl to it:
python3 main.py --config config.json --server 127.0.0.1
```

Judge frame rate and processing time on the coprocessor, not the Mac. Apple Silicon is much faster.

## 1. Set up the coprocessor

Owl needs Python 3.11 or newer on ARM64. The default Python 3.10 on Ubuntu 22.04 has no wheel for the pinned RobotPy AprilTag package ([published wheels](https://pypi.org/project/robotpy-apriltag/2026.2.2/#files)). `requirements.txt` picks the right NumPy version for your Python, up to 3.14.

### Orange Pi 5

Use a second SD card so the competition PhotonVision card stays untouched. Choate runs the Orange Pi Ubuntu 22.04 image with Python 3.14 in a venv at `~/vision/coprocessor/venv`.

The Pi usually has no internet, so install from wheels downloaded on a Mac. `owl/wheels/` (not in git) holds the aarch64, Python 3.14 wheels for everything in `requirements.txt`:

```bash
# On the Pi, in ~/vision/coprocessor:
<your python3.14> -m venv venv
venv/bin/pip install --no-index --find-links wheels -r requirements.txt
venv/bin/python test_synthetic.py          # should print PASS
```

Owl uses `v4l2-ctl` for manual exposure and gain and for the resolution list. Normally that's `sudo apt install v4l-utils`, but on the Orange Pi image that package conflicts with the Rockchip-patched `libv4l-0` (`...+rkmpp`), and downgrading that library can break hardware video decoding. Instead, extract just the `v4l2-ctl` binary from Ubuntu 22.04's arm64 `v4l-utils` .deb on the Mac and install it:

```bash
scp v4l2-ctl choate7407@192.168.50.2:~/
ssh -t choate7407@192.168.50.2 "sudo install -m 755 ~/v4l2-ctl /usr/local/bin/ && v4l2-ctl --version"
```

It only needs `libv4l2.so.0`, which the patched library provides.

Give the Pi a static IP (Choate's is 192.168.50.2). To work on it over a direct Ethernet cable, set your Mac's Ethernet adapter to a manual address on the same subnet (for example 192.168.50.10, mask 255.255.255.0, no router) and check with `ping 192.168.50.2`.

### Jetson Orin Nano

NEIA's Jetson runs JetPack 6.2.1 (L4T R36.4.7, Ubuntu 22.04). Install a Python 3.11 or newer (NEIA's Jetson uses Python 3.14 installed with [uv](https://docs.astral.sh/uv/)), make a venv in `owl/`, and install the requirements as above. Owl works on the CPU detector with nothing else.

To detect tags on the GPU, build `libapriltag_gpu_capi.so` from `native/gpu_detector` by following [its README](../native/gpu_detector/capi/README.md#building-on-a-jetson-orin-nano), then turn it on in the config (see [GPU detection](#gpu-detection-jetson-only) below). Check the build with:

```bash
venv/bin/python test_synthetic.py --backend cuda --cuda-library /path/to/libapriltag_gpu_capi.so
```

This compares the GPU detector against the CPU one on the same rendered frames. Install `v4l-utils` for `v4l2-ctl` if it isn't already there.

## 2. Cameras

```bash
ls -l /dev/v4l/by-path/
v4l2-ctl -d /dev/video0 --list-formats-ext    # resolutions and frame rates for each format
```

Use the `/dev/v4l/by-path/...-video-index0` paths in `config.json`. Each camera also shows an `index1`, which is metadata; ignore it. By-path names are tied to the physical USB port, so identical cameras can't swap places between reboots. Moving a cable to a different port does change the name.

**Resolution.** Pick it on the settings page. The dropdown lists what the camera offers in MJPG, with the full-sensor mode first when there is one (for an OV2311, 1600×1300, 16:13). If that isn't available, use 1600×1200 (4:3, nearly the full sensor). Avoid 16:9 modes, which crop the vertical field of view. Higher resolution gives more range and less noise at a distance. 25 to 50 fps per camera is plenty for localization, since the pose estimator corrects for latency.

**Which camera is which.** Cover one lens and see which stream goes dark. Don't go by what's in the picture: when cameras are angled toward each other, the physical left camera sees the right side of the scene. Each config entry's `device` must be the physical camera that its `robot_to_camera` describes, and calibration files are named after the entry. If two cameras on the same robot report very different robot poses (tens of centimeters, tens of degrees), check this first.

## 3. Configure

Run `cp config.example.json config.json` and edit it.

`robot_to_camera` says where each camera is mounted, in WPILib's conventions: x forward, y left, z up, in meters from the center of the robot at floor level. Pitch is negative when the camera is tilted up. Yaw is counterclockwise-positive, so a camera facing the robot's right side is about −90° and a rear camera about 180°.

The field layout lives on the coprocessor, because Owl does the pose solve there; the robot never sends it. `field.layout` is either the name of a layout built into WPILib (the default is `k2026RebuiltWelded`, which comes with the `robotpy-apriltag` package) or a path ending in `.json` to a WPILib-format layout file, for example a measured layout from your practice field. Change it whenever the field changes, and make sure the robot code uses the same layout for anything it does with tag positions.

`nt.standalone: true` makes Owl its own NetworkTables server, for bench testing without a roboRIO. Set it back to `false` on the robot, or the roboRIO never sees the data. Frame rate (`fps`) is set per camera; set it to what the camera mode supports (the resolution dropdown shows the maximum).

On a moving robot, low exposure matters more than anything else, because motion blur stops tags from being detected. Tune it live on the settings page: turn off Auto, lower the exposure until a waved tag stays crisp, raise the gain if the image gets too dark, then save.

## 4. Calibrate

![Calibration in progress: the ChArUco board on a monitor, markers and corners outlined, with the capture panel on the right](../docs/images/owl-calibration.png)

Calibrate from the settings page. A camera boots and streams without a calibration, but it doesn't publish poses (the header says NOT CALIBRATED) until it has one for its current resolution. Pick the resolution first, because each resolution needs its own calibration.

1. **Board.** The default is 6328's Northstar board (12×9 squares, 5×5 markers, 30 mm squares and 23 mm markers). There's also a PhotonVision 8×8 preset, or you can type in your own. **Print board** gives an image that prints at true size at 100% scale. The 6328 board is 360×270 mm, bigger than letter paper, so print it on 11×17 or show it full screen on a flat monitor. Mount it flat and measure a printed square to check the size.
2. **Start calibration.** Pose output pauses for that camera. Hold the board in view and keep it still for a moment. Owl captures a view automatically whenever the board is held still in a new position (a different spot, distance or tilt). The stream outlines every ArUco marker with its ID and marks each ChArUco corner in a color that shows its state: red means too little of the board is visible, light blue means Owl already has this view, yellow means a new view (hold still), and green means it's capturing. A coverage map and a hint tell you what to do next, such as "move the board to the top left" or "tilt the board ~30 deg (left, up)". **Capture now**, **Undo** and **Clear** are there if you want manual control.
3. Aim for 15 or more views, 65% or more coverage including the corners, and at least two views tilted each way. Then press **Calibrate**.
4. **Review.** The results show RMS error (under 0.5 px is good), focal length and principal point with their uncertainty, field of view, and the error for each view. Views much worse than the rest (blurry, or with bad detections) are dropped and the calibration runs again. **Preview undistorted** should make straight edges look straight.
5. **Save calibration.** It's written to `calibrations/<camera>_<W>x<H>.json` and used immediately, with no restart.

If your lens is wider than about 100°, tick **Wide-angle lens** before starting (this uses OpenCV's 8-coefficient model). Older calibration files (`calibrations/<camera>.json` from `calibrate.py`, or PhotonVision exports) still load when their resolution matches. Calibrations aren't in git, so back them up (see the quick reference below).

## 5. The settings page, live tuning and saving

The page is at `http://<coprocessor-ip>:1181/`. Each camera has its own port starting at 1181, and any of them serves the whole page. The page has four tabs, and the URL remembers the tab and camera (`#page=tuning&cam=<name>`):

- **cameras** shows every camera at once, each with its live stream, fps, pose or reason for rejecting the frame, and calibration state.
- **tuning** shows the chosen camera's stream, its pipeline (see [Pipelines](#pipelines)), and its settings in three groups: **input** (resolution, exposure, gain), **apriltag** (detector and filters) and **output** (standard deviation factor, stream fps). Click the pose line under the stream to open the targets table, which lists every tag in the frame with its yaw, pitch, skew, area, distance and decision margin, and whether it was used.
- **calibration** has the board setup, capture and results described above.
- **coprocessor** shows diagnostics (CPU load per core, GPU load on a Jetson, temperatures, memory, disk, uptime, IP addresses, the NetworkTables connection, every camera process with whether it's running, how often it has restarted, which detector it uses and its fps, and software versions), plus the networking and maintenance controls described in section 6.

![The tuning tab on a Jetson Orin Nano: tag 6 outlined in green with a 3D box, the pose line underneath, and the CUDA detector at 80 fps](../docs/images/owl-tuning.png)

![The cameras tab: one camera's stream, fps, detect time, pose and calibration state](../docs/images/owl-cameras.png)

![The coprocessor tab on a Jetson: CPU per core, GPU load, temperatures, NetworkTables, and the camera process running "cuda on gpu"](../docs/images/owl-coprocessor.png)

The header shows, for the chosen camera, camera fps (what the camera delivers), processed fps, pose fps, detection time and calibration status. The stream has two views. *Annotated* outlines tags used in the pose in green and filtered tags in orange, puts a red dot on corner 0, draws a 3D box on each tag once the camera is calibrated, and prints the pose or the reason the frame was rejected. *Raw* is the plain camera image.

Streams are full resolution and only encode while someone is watching, so an unwatched stream costs nothing. `stream_max_fps` caps the encoding rate while someone is watching. Both streams are registered under `/CameraPublisher`, so Elastic's camera widget finds them. The field network limits robot bandwidth, so keep streams closed during matches.

### Live tuning

Every setting takes effect within about 0.1 seconds, whether you change it from the page, Elastic or AdvantageScope. They all edit the same NetworkTables entries:

```
/Vision/cameras/<name>/settings/resolution, exposure, gain, std_dev_factor     camera
/Vision/cameras/<name>/settings/quad_decimate, min_cluster_pixels              detector
/Vision/cameras/<name>/settings/min_decision_margin, max_single_tag_ambiguity,
                                max_reprojection_error_px, max_tag_distance_m   filters
/Vision/settings/stream_max_fps
/Vision/settings/board_*                                                         calibration board
```

The per-camera settings belong to that camera's active pipeline. The two `/Vision/settings` groups apply to every camera. For exposure and gain, -1 means auto (NetworkTables has no null). Exposure is in the camera's own units (often 100 µs steps); the page reads the range from the camera.

Owl checks every setting before using it, whether it comes from `config.json`, the web page or NetworkTables. An invalid value (for example NaN, zero or a negative trust factor) is rejected, the last valid value stays in place, and the page shows that the setting was rejected.

### Saving

Changes apply immediately but aren't saved until you press **Save settings** (or set `/Vision/settings/save` to true). Owl writes `config.json` atomically, so a power cut during a save can't corrupt it. On startup `config.json` always wins, so unsaved experiments disappear on restart. After a tuning session, copy the coprocessor's `config.json` back into the repo.

### Pipelines

Each camera has one or more named pipelines, as in PhotonVision. A pipeline holds every per-camera setting above: resolution, exposure, gain, std dev factor, detector and filters. Editing a setting edits the active pipeline. The bar above the tuning tabs switches pipelines and has buttons to add one (with default settings), duplicate, rename, make it the default (★) and delete (a camera always keeps at least one). Pipeline changes apply at once but, like settings, aren't written to `config.json` until you press **Save settings**.

Switching only applies the values that differ. Pipelines at the same resolution switch immediately. A different resolution reopens the camera (about a second on USB cameras) and uses that resolution's calibration, so calibrate every resolution you use. Once a camera has more than one pipeline, the cameras tab shows which one is active.

The robot switches pipelines by name. Write the name to `/Vision/cameras/<name>/pipelineRequest`; `/Vision/cameras/<name>/pipelineState` reports the active one, and `/Vision/cameras/<name>/pipelines` lists them all. Because they're names rather than PhotonVision-style indices, they don't shift when a pipeline is deleted. Owl acts on a request when its value changes, so an old request doesn't undo a switch made on the page, and an unknown name is ignored. A switch from the robot lasts until Owl restarts. Each camera starts in its default pipeline, which can only be changed from the page. Observations don't say which pipeline produced them, so read `pipelineState` if the robot needs to know.

In `config.json`, a camera's pipelines are a `pipelines` list plus `pipeline`, the name of the default. Any setting a pipeline leaves out comes from the camera's own fields and the shared `detector` and `filters` sections. A camera without `pipelines` gets one called `default`, so older configs load unchanged. When you save, the camera's `width`, `height`, `exposure`, `gain` and `std_dev_factor` are set to match its default pipeline.

### Camera status in NetworkTables

Each camera's status is under `/Vision/cameras/<name>/` in NetworkTables: `fps`, `processMs`, `tagFps`, `connected`, `lastReject`, and `robotPose` (a plain Pose3d you can drag onto AdvantageScope's field view). The robot reads `/Vision/observations`.

## 6. Run on boot

`../deploy/owl.service` assumes a user called `strix` with the repo at `~/strix`. Change `User`, `WorkingDirectory` and `ExecStart` to match your setup. `ExecStart` must be the full path to the venv's `python`, because systemd doesn't use your shell's PATH or an activated venv. Then:

```bash
sudo cp deploy/owl.service /etc/systemd/system/
sudo systemctl daemon-reload && sudo systemctl enable --now owl
```

After editing the installed service, run `sudo systemctl daemon-reload` and restart it.

### Networking from the page

The coprocessor tab sets the NetworkTables target (a team number, or a server IP or hostname) and the standalone bench-test server. Both take effect as soon as you press **Save networking**, without a restart, and are saved in the `nt` section of `config.json`. If neither a target nor standalone mode is set, Owl keeps running and the page shows a warning.

Turn on **manage hostname and IP address** to set the coprocessor's hostname and its wired interface's IP (DHCP, or static with a netmask and an optional gateway). Choosing static with a team number filled in suggests `10.TE.AM.11`. Owl applies these through NetworkManager on a connection of its own, `owl-<interface>`, and leaves the original connection alone, so deleting `owl-<interface>` (for example `sudo nmcli connection delete owl-eth0`) goes back to it. Changes are applied only when you save, and the `network` section of `config.json` records what was last applied. A new hostname or IP moves the page, which then links to the new address. This needs the admin helper (below).

### Maintenance buttons

Also on the coprocessor tab:

- **View logs** and **download logs** show Owl's journal (`journalctl -u owl`). The service user must be in the `adm` or `systemd-journal` group.
- **Export settings** downloads a zip of `config.json` and `calibrations/`.
- **Import settings** takes an exported zip or a bare `config.json`. The config must load before anything is replaced. Calibrations in the zip are added, overwriting any with the same name.
- **Restart owl** makes Owl exit cleanly (each camera releases its device), and systemd's `Restart=always` starts it again. This only works when Owl runs as a service.
- **Reboot coprocessor** needs the admin helper (below).
- **Factory reset** sets every setting back to the defaults in `config.example.json`. It keeps what Owl needs to start (each camera's name, device, mode and mounting position, and the NT target) and removes the calibrations.

Import and factory reset first copy `config.json` and `calibrations/` into `backups/<time>-<what>/`, then restart Owl. To undo one, copy that folder back.

### Admin helper

Changing the hostname or IP and rebooting need root, and Owl doesn't run as root. Those actions go through `../deploy/owl-admin`, a small script Owl runs with `sudo -n`. Install it owned by root and outside the repo (it imports nothing from the Owl checkout, which the service user can write to), and allow only that script in sudoers. Replace `strix` with the service's user:

```bash
sudo install -o root -g root -m 755 deploy/owl-admin /usr/local/sbin/owl-admin
echo 'strix ALL=(root) NOPASSWD: /usr/local/sbin/owl-admin' | sudo tee /etc/sudoers.d/owl-admin
sudo chmod 440 /etc/sudoers.d/owl-admin && sudo visudo -c
```

Reinstall it whenever `deploy/owl-admin` changes. Without it everything else on the page still works, and the networking card says why hostname and IP are unavailable.

The page has no login, so anyone on the robot network can use these buttons, just as they can change tuning settings.

## 7. Robot code

Set up Tusk in the robot project next: [../tusk/README.md](../tusk/README.md). That page also covers Choate's 2026 robot code, how much to trust vision, and how single tags are handled. Deploy Owl and the robot code together whenever the message format changes.

## Quick reference: Choate's Orange Pi

The Pi is at 192.168.50.2, the user is `choate7407`, the code is in `~/vision/coprocessor`, and the `vision` systemd service runs it. Run these from the repo root on your Mac.

| What | How |
|---|---|
| Settings page (all cameras) | `http://192.168.50.2:1181/` (camera dropdown at the top) |
| AdvantageScope / Elastic | connect to `192.168.50.2` (run `main.py --standalone` when there's no roboRIO) |
| Is it running? | `ssh choate7407@192.168.50.2 "systemctl status vision --no-pager"` |
| Logs | `ssh choate7407@192.168.50.2 "journalctl -u vision -n 50 --no-pager"` (`-f` to follow) |
| Restart | `ssh -t choate7407@192.168.50.2 "sudo systemctl restart vision"` |
| Deploy code | `scp owl/*.py owl/web_ui.html owl/web_helpers.js choate7407@192.168.50.2:~/vision/coprocessor/`, then restart |
| Pull the tuned config back | `scp choate7407@192.168.50.2:~/vision/coprocessor/config.json owl/configs/7407.json` |
| Back up calibrations | `scp -r choate7407@192.168.50.2:~/vision/coprocessor/calibrations owl/` |

Don't copy your `config.json` to the Pi without pulling the Pi's copy first. Settings saved on the Pi (exposure, resolution, filters) only exist in the Pi's copy. Run `ssh-copy-id choate7407@192.168.50.2` once and you won't have to type the SSH password again.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| `status=203/EXEC` in `systemctl status` | The `ExecStart` path is wrong. Check it with `ls -l <path>`; usually the username in `/home/<user>/...` is missing. |
| `status=1/FAILURE`, restarting | `main.py` itself failed. The reason is in `journalctl -u <service> -n 50 --no-pager` (a config typo, a bad device path, and so on). |
| Page header says NOT CALIBRATED | There's no calibration for this camera at this resolution. Calibrate on the page, or switch to a resolution you've calibrated. |
| Low camera fps | Auto exposure lengthens frames in dim light, so set exposure manually. Also check the mode's maximum fps in the dropdown and `fps` in the config. |
| Camera fps fine, processed fps low | The CPU can't keep up: detection takes longer than the time between frames. Raise `quad_decimate` or lower the resolution. Check with the page closed, since encoding the stream also uses CPU. |
| Two cameras disagree on the robot's pose | `device` and `robot_to_camera` don't describe the same physical camera (see "Which camera is which"), or a yaw or pitch sign is wrong. |
| Rejected: robot height wrong | `robot_to_camera` is off (height, or the sign of the pitch). This is normal on a desk, since the camera isn't where the config says it is. |
| AdvantageScope can't connect | Nothing is running a NetworkTables server. Run `main.py --standalone` (or set `"standalone": true`) for bench tests, and check that your laptop is on the coprocessor's subnet. |
| Exposure slider does nothing | `v4l2-ctl` isn't installed; see section 1. |
| A camera keeps restarting | Owl restarts a camera process that crashes or stops sending frames. Check the logs and the coprocessor tab for the reason, and check the USB cable. |

## Tuning notes

### Detector

`quad_decimate` finds tag outlines on a copy of the image shrunk by that factor. Corners are still refined at full resolution. Higher values are much faster but lose small, far tags, so this trades range for speed.

`min_cluster_pixels` is the smallest candidate outline considered, measured on the shrunken image. It filters out noise. WPILib's default of 300 loses far tags, so keep it low (around 20). With `quad_decimate` at 2, a value of 20 found more tags than no decimation at all, at about a third of the CPU cost.

`min_decision_margin` is how confidently a tag's bits were read. Raise it if you see phantom tags or jumpy far-tag poses. Lower it (to around 15) if good tags get dropped in dim light or at a distance.

If detection time is close to or above the time between frames (20 ms at 50 fps), raise `quad_decimate` or lower the resolution. Owl always processes the newest frame, so falling behind costs frame rate, not latency.

If you print test tags, use the official AprilRobotics images. OpenCV's generated tag36h11 images are rotated 180° from the official FRC tags.

### Reprojection error limit

`filters.max_reprojection_error_px` (default 3 px) is how far, on average, each tag's detected corners may be from where the solved pose says they should be. If any tag in a frame is over the limit, Owl rejects the whole frame instead of guessing which tag is wrong. Frames that contain the same tag ID twice are also rejected. The 3 px default is a starting point and hasn't been tuned against field recordings yet.

To collect data for tuning it, set `STRIX_QUALITY_RECORD_DIR` to a folder before starting Owl. Each camera then records tag corners, calibration, camera transform, settings, residuals and whether each frame was accepted, at up to 5 frames per second, in three rotating files of about 5 MB each. No images are recorded. Summarize a recording with:

```bash
python tools/analyze_quality_recording.py /path/to/camera-0.jsonl --threshold 3
```

Record good scenes and failure cases separately. Residuals show whether a solve is internally consistent, not whether it's accurate: a tag placed in the wrong spot on the field can still fit perfectly, which is why Tusk also checks poses on the robot side.

### Standard deviations

Owl computes each observation's `stdDevFactor` as (average tag distance)² / (tag count)² × the camera's `std_dev_factor`. Raise a camera's `std_dev_factor` to trust it less than the others. The robot-side coefficients that turn this into standard deviations are described in [../tusk/README.md](../tusk/README.md#trusting-the-measurements).

### GPU detection (Jetson only)

Set `"backend": "cuda"` in the `detector` section of `config.json` to detect tags on the GPU with FRC 971's CUDA detector, through `libapriltag_gpu_capi.so` built from `native/gpu_detector` ([build steps](../native/gpu_detector/capi/README.md)). Put the `.so` in `owl/`, or point `detector.cuda_library` or the `APRILTAG_GPU_LIB` environment variable at it. The default, `"cpu"`, uses WPILib's detector and needs none of this.

On the GPU:

- decimation is always 2 (`quad_decimate` is ignored),
- `min_cluster_pixels` below 24 is treated as 24,
- a resolution that isn't a multiple of 8 is cropped by up to 7 px around the center (1600×1300 is detected as 1600×1296).

If the library is missing, the resolution is unsupported (a side over 2048 px) or the detector reports an error, that camera logs why and continues on the CPU detector. The stream overlay and the coprocessor tab show which detector each camera is using. If the GPU detector crashes the camera process, Owl restarts that camera on the CPU and keeps it there until Owl itself restarts. Capture errors and restarts for stalled cameras keep the configured detector.

The GPU library must be built from this checkout (C API version 2). Owl refuses older version 1 builds and uses the CPU instead.

## What's been verified

- **Pose math** (`test_synthetic.py`): renders real tag36h11 images on the 2026 field from known robot poses through a pitched, yawed and offset camera. Median error is 0.3 cm with multiple tags and 0.7 cm with one.
- **End to end with synthetic cameras**: two synthetic camera feeds went through `main.py`, NT4 and Tusk into a `SwerveDrive4PoseEstimator` that started 1.8 m off and converged to within 1.8 cm of the truth. `tests/owl_integration.py` runs two generated video cameras through the full app in CI, including killing and freezing camera processes to check that they recover.
- **Calibration**: a simulated OV2311 at 1600×1300 with strong barrel distortion, calibrated through the web page from a video, recovered the focal lengths and principal point to within about 0.5 px of the truth at 0.28 px RMS.
- **Live settings**: changes from the page, Elastic and AdvantageScope stay in sync, saving writes the config atomically, and switching resolution loads the matching calibration.
- **Orange Pi 5**: runs as a service with two USB cameras. Streams, the settings page, and exposure and gain through `v4l2-ctl` all work.
- **Jetson Orin Nano**: runs with an OV9281 at 1280×800 on both the CPU and CUDA detectors, as in the screenshots above, and has fed NEIA's robot code in simulation. With the CUDA detector, detection took about 9 ms per frame. Proper CPU-versus-GPU benchmarks are still on the [roadmap](../docs/roadmap.md).

## Not included

Owl doesn't fuse the cameras into a single pose; the robot fuses each camera's observations itself. It also has no object detection.

## Network security

The web page and its settings API have no login. They're meant for a trusted robot network. Browsers can only send changes from a page served by the same coprocessor, and request sizes and connection counts are limited. `--nt-port` picks a different NetworkTables port for isolated bench tests; on the robot, leave it at the default 5810.
