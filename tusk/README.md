# Tusk: the robot side of Strix

Tusk goes in your robot code. Every loop it reads the observations Owl publishes on the coprocessor, drops the ones that can't be trusted, and feeds the rest to WPILib's pose estimator with `addVisionMeasurement`. If you haven't set up Owl yet, start with [../owl/README.md](../owl/README.md).

There are two versions with the same classes and behavior:

| Version | Where | Use it if your robot code is |
| --- | --- | --- |
| RobotPy | `tusk/robotpy/tusk/` | Python |
| Java | `tusk/java/` | Java, on WPILib 2026 (roboRIO) or 2027 (Systemcore) |

## The classes

| Class | What it does |
| --- | --- |
| `VisionObservation` | One camera's result for one frame: the robot pose, the alternate pose for single tags, which tags were seen, their average distance, a trust factor and the frame's age. It must match `owl/vision_types.py` byte for byte. |
| `TuskSource` | Reads `/Vision/observations` once per loop and returns each observation with its capture time on the robot's clock. It ignores the first 2 seconds of auto, and asks Owl to slow to 1 fps after the robot has been disabled for 5 seconds. |
| `select_pose` (Python) / `Tusk.selectPose` (Java) | Picks the pose to use. With two or more tags there is only one. With one tag, it picks whichever of the two candidates matches the robot's heading, and can drop it if that's still too far off. |
| `TuskVision` | The all-in-one option. Calls the source, picks poses, checks them and calls `addVisionMeasurement`. Most teams only need this. |

In Python, `TuskVision` and `TuskSource` are also exported under their older names, `BatchedVision` and `BatchedVisionSource`.

## RobotPy

Copy the `tusk/robotpy/tusk/` folder into your robot project next to `robot.py`. Then, wherever you own the pose estimator (usually the drivetrain):

```python
from tusk import TuskVision

class Drivetrain(commands2.Subsystem):
    def __init__(self):
        ...
        self.vision = TuskVision()

    def periodic(self):
        self.pose_estimator.update(self.gyro.getRotation2d(), self.module_positions())
        self.vision.update(self.pose_estimator)
```

`update()` returns how many observations were accepted. `self.vision.last_observations` holds the latest batch if you want to log it.

## Java

Build the library for your WPILib season:

```bash
cd tusk/java
./gradlew build -PwpilibYear=2026          # jar in build/libs/strix-tusk-2026-*.jar
./gradlew build -PwpilibYear=2027          # the 2027 build downloads JDK 25 if needed
```

Either add the jar to your robot project as a dependency, or copy the source files in. `./gradlew exportSources -PwpilibYear=2026` writes plain `.java` files for that season to `build/export/tusk-src-2026/`; copy the `strix/tusk/` folder from there into your project's `src/main/java/`.

Then:

```java
import strix.tusk.TuskVision;

TuskVision vision = new TuskVision();

// in periodic(), after updating the pose estimator from odometry:
vision.update(poseEstimator);
```

`update(estimator)` uses the estimator's pose history (`sampleAt`) to find where the robot was when each frame was captured. If your drivetrain wraps the estimator (CTRE swerve, for example), pass the three pieces yourself:

```java
vision.update(getPose(), poseEstimator::sampleAt, this::addVisionMeasurement);
```

### AdvantageKit

If your robot code uses the AdvantageKit vision template (`Vision` and `VisionIO`), copy [java/examples/VisionIOTusk.java](java/examples/VisionIOTusk.java) next to `VisionIO.java`, change its package, and add `TUSK` to `VisionIO.PoseObservationType`. One `VisionIOTusk` covers every Owl camera:

```java
vision = new Vision(drive::addVisionMeasurement,
                    new VisionIOTusk(new TuskSource(), drive::samplePoseAt, drive::getPose));
```

The example's imports are for WPILib 2026. For 2027, change `edu.wpi.first` imports to their `org.wpilib` names using `java/wpilib-2026-imports.txt`. This adapter ran in NEIA's 2026 robot code in simulation, fed by Owl on a Jetson.

## Trusting the measurements

Each observation's std devs are a coefficient times Owl's `stdDevFactor`, which is:

```
stdDevFactor = (average tag distance)² / (tag count)² × the camera's std_dev_factor
```

So a far, lone tag is trusted much less than a close group of tags. The coefficients are WPILib Preferences. You can tune them in Elastic under `/Preferences/Vision/`, and the roboRIO keeps them across reboots and deploys.

| Preference | Default | Meaning |
| --- | --- | --- |
| `XYStdDevCoefficient` | 0.01 | x/y standard deviation in meters = coefficient × stdDevFactor |
| `ThetaStdDevCoefficient` | 0.03 | heading standard deviation in radians = coefficient × stdDevFactor |
| `TrustSingleTagTheta` | false | When false, the heading from single-tag frames is ignored and the gyro owns heading |

To tune them, drive around while watching the pose. If the pose lags behind or drifts away from what vision shows, lower the coefficients. If it jitters or jumps, raise them. To trust one camera less than the others, raise its `std_dev_factor` on Owl's settings page instead.

## What gets rejected

Tusk checks every observation before it reaches the pose estimator. It drops observations that:

- are malformed (non-finite numbers, a tag count outside 1 to 64, a trust factor or tag distance that isn't positive),
- are more than 1 second old, or claim to be more than 20 ms in the future,
- arrive while the robot is enabled and the estimator has no pose history for their capture time.

While the robot is enabled it also drops:

- single-tag poses more than 1 m from where the estimator thinks the robot was at capture time,
- single-tag poses whose heading is more than 5° off,
- multi-tag poses more than 2 m away.

While disabled these distance and heading checks are off, so vision can set the robot's starting position. All three limits are constructor arguments on `TuskVision`.

### Why single tags need special handling

A single tag fits two camera positions almost equally well. Owl sends both: `robotPose`, the better fit, and `altRobotPose`. For multi-tag frames they are the same. Tusk picks whichever has a heading closer to the robot's heading at the time the frame was captured. That has to be the heading at capture time, because a frame that is 80 ms old from a robot turning at 180°/s was taken 14° ago. For a single tag a position error comes with a heading error, so the 5° heading check also catches noisy far-away solves. This approach comes from FRC 6328's Northstar.

### Timing

Each observation carries its age when Owl published it, measured from the camera driver's frame timestamp when there is one and otherwise from when Owl read the frame. NetworkTables converts the publish time to the robot's clock, so `poll()` returns capture times in the same time base as `Timer.getTimestamp()`. That's what `addVisionMeasurement` expects.

### Auto and disabled

For the first 2 seconds of auto, `poll()` returns nothing, so the starting pose you set in `autonomousInit` isn't pulled around by a noisy first frame.

After the robot has been disabled for 5 seconds, Tusk publishes `/Vision/throttleFps = 1` and Owl drops to one frame per second, which keeps the coprocessor cool in the queue and the pits. The throttle pauses while someone has a camera's page or stream open or is calibrating, so tuning always runs at full rate. Enabling the robot brings Owl back to full rate within about 0.1 seconds.

## Choate's 2026 robot (7407-DriveCode-Rebuilt)

`tusk/robotpy/examples/field_odometry_7407.py` replaces the PhotonVision `FieldOdometry` in Choate's robot code:

1. Copy the `tusk/robotpy/tusk/` package into the robot project, and copy `field_odometry_7407.py` into `sensors/` as `field_odometry.py`.
2. In `sensors/__init__.py`, export `BatchedVisionSource` (from `tusk`) alongside `FieldOdometry`.
3. In `robotcontainer.py`, instead of building `PhotonCamCustom`s for `front_cam` and `front_left_cam`, create one source and pass it to `FieldOdometry` along with the estimator's history lookup:
   ```python
   self.vision = BatchedVisionSource()
   self.field_odometry = FieldOdometry(self.drivetrain, self.vision, self.backup_gyro,
                                       self.pose_estimator.sampleAt)
   ```
   `robot.py` keeps calling `self.robot.field_odometry.update()` every loop, and `enable()` and `disable()` work as before.
4. Deploy the robot code and Owl together (see below).

This `FieldOdometry` keeps the old one's enable and disable, its single-tag filter (rejecting tags beyond `robot_constants.odometry_tag_distance` or above 0.4 ambiguity, or `robot_constants.odometry_single_tag_ambiguity` if set), and the backup-gyro reset when `gyro_broken`. It also picks the single-tag candidate by heading, and while enabled drops single-tag poses more than `Single Tag Max Yaw Error Deg` off the gyro.

The old hand-tuned standard deviation tiers become the one formula above, with its own Preferences under `/Preferences/Vision/`:

| Preference | Default | Meaning |
| --- | --- | --- |
| `XY Std Dev Coeff` | 0.2 | x/y standard deviation in meters = coefficient × stdDevFactor |
| `Theta Std Dev Coeff` | 5.0 | heading standard deviation in radians = coefficient × stdDevFactor, multi-tag only |
| `Single Tag Theta Std Dev` | 100 | heading standard deviation for single tags (large, so the gyro owns heading) |
| `Single Tag Max Yaw Error Deg` | 5.0 | while enabled, drop single-tag poses whose heading is further than this from the gyro |

The defaults roughly reproduce the old tiers (3 tags at 3 m gives 0.2 m, 1 tag at 2 m gives 0.8 m). The old code trusted `front_cam` more when it saw two tags; to get the same effect now, give `front_left_cam` a higher `std_dev_factor` on Owl's settings page.

Capture times from `poll()` are FPGA seconds, which is what the drivetrain's `add_vision_measurement` expects (it converts to Phoenix time itself).

The cameras in Owl's `config.json` are named to match `robot_constants.py` (`front_cam` and `front_left_cam`), and their `robot_to_camera` values are copied from `front_cam_transform` and `front_left_cam_transform`, converted from inches to meters. If a transform changes in the robot code, change it in Owl's config too.

## Deploy Owl and robot code together

The message format is versioned by name: it's currently `VisionObservationV3` (V3 added the alternate single-tag pose). If Owl and Tusk disagree on the version, the robot gets no vision data at all rather than misread data. `owl/vision_types.py` and `tusk/robotpy/tusk/vision_types.py` must be identical (CI checks this), and the Java `VisionObservation` must match them.

`Vision/Coprocessor Connected` on SmartDashboard (Choate's example) and `isConnected()` only mean Owl is alive and sending its heartbeat. Check that observations are actually arriving too.

## Tests

```bash
python -m pytest -q tusk/robotpy/tests       # run with RobotPy 2026 and again with 2027
(cd tusk/java && ./gradlew build -PwpilibYear=2027 && ./gradlew build -PwpilibYear=2026)
```

Both versions are tested against the same message bytes in `tusk/testdata/observation_v3.json`, so they can't drift apart. [../docs/wpilib-2027.md](../docs/wpilib-2027.md) explains how one set of sources builds for both seasons.
