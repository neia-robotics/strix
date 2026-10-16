# WPILib 2026 and 2027

Strix has to work with WPILib 2026 on the roboRIO now and WPILib 2027 on the Systemcore next season, and 2027 renames a lot of the API. This page lists what changed between the two seasons, how each part of Strix copes with it, and what has been tested.

Versions checked: WPILib Java `2027.0.0-alpha-6` and RobotPy `2027.0.0a7` (October 2026), against WPILib `2026.2.1` and RobotPy `2026.2.2`.

## What changed in 2027

| Area | 2026 | 2027 |
| --- | --- | --- |
| Java packages | `edu.wpi.first.*` | `org.wpilib.*` (and the Maven group is `org.wpilib`) |
| Java version | 17 | 25 |
| `Matrix`, `VecBuilder` | `edu.wpi.first.math` | `org.wpilib.math.linalg` |
| `Timer` | `wpilibj.Timer` (`getFPGATimestamp`, `getTimestamp`) | `org.wpilib.system.Timer` (`getTimestamp` only) |
| Robot mode | `DriverStation.isEnabled()`, `isAutonomousEnabled()` | `org.wpilib.driverstation.RobotState` |
| `Preferences` | `wpilibj.Preferences` | `org.wpilib.util.Preferences` |
| `PubSubOption` | `sendAll(true)`, `keepDuplicates(true)` | `SEND_ALL`, `KEEP_DUPLICATES` constants |
| `Struct` | has `kSizeInt32` etc. | size constants removed |
| RobotPy names | camelCase (`Timer.getTimestamp`, `pose.toPose2d()`, `pose.X()`) | snake_case (`Timer.get_timestamp`, `pose.to_pose2d()`), and some getters became properties (`pose.x`) |
| RobotPy geometry | `wpimath.geometry` | top-level `wpimath` |
| RobotPy `PubSubOptions` | `sendAll=`, `keepDuplicates=`, `pollStorage=` | `send_all=`, `keep_duplicates=`, `poll_storage=` |
| RobotPy NT server | `startServer(persist, listen, nt3_port, nt4_port)` | `start_server(persist, listen, mdns_service, port)` |
| RobotPy Linux ARM wheels | `manylinux_2_35` (installs on Ubuntu 22.04) | `manylinux_2_39`, which needs Ubuntu 24.04 or newer |

The `VisionObservationV3` message (160 bytes) has the same layout and schema string in both seasons, and NT4 itself didn't change, so a 2026 client and a 2027 server can talk to each other.

## How each part handles it

### Tusk Java (`tusk/java`)

The sources in `src/main/java` use the 2027 names. To build for 2026, Gradle copies them and rewrites the imports using the list in `wpilib-2026-imports.txt`. The one call that differs by more than its package (the `PubSubOption` style) lives in `src/compat2026` and `src/compat2027`. Elsewhere the code sticks to calls that exist in both seasons (`Timer.getTimestamp`, `RobotState`), so only imports need rewriting. If a 2027 import has no 2026 mapping, the build fails, which catches new imports that weren't added to the list.

```bash
./gradlew build                     # 2027 (downloads JDK 25 if needed)
./gradlew build -PwpilibYear=2026   # 2026
```

### Tusk RobotPy (`tusk/robotpy`)

`tusk/_compat.py` wraps the calls whose names changed. `call(obj, "camelName")` calls the camelCase method on 2026, and the snake_case method (or property) on 2027. Names that didn't change (`rotation()`, `radians()`, `distance()`, `subscribe()`, `publish()`, `.time`, `.value`) are called directly.

### Owl (`owl/`)

Owl stays on the RobotPy 2026 libraries for now (`owl/requirements.txt` pins them below 2027). It runs on the coprocessor, not the robot, so it doesn't need to match the robot's season; NT4 works between the two. It can't move yet anyway: RobotPy 2027's Linux ARM wheels need glibc 2.39 (Ubuntu 24.04), and the Orange Pi image and JetPack 6 are both Ubuntu 22.04. Porting Owl to 2027 is on the roadmap for when the coprocessors can run it.

### `vision_types.py`

The message definition shared by Owl and Tusk RobotPy imports `Pose3d` from whichever location exists, so the same file works in both seasons.

## What's been tested

| Check | 2026 | 2027 |
| --- | --- | --- |
| Tusk Java builds; struct and `selectPose` tests pass | yes | yes |
| Tusk Java decodes the bytes RobotPy encodes (`tusk/testdata/observation_v3.json`) | yes | yes |
| Tusk RobotPy tests, including a local NT publish/poll and `TuskVision.update` | yes | yes |
| `VisionObservationV3` schema and size identical | yes | yes |
| NT4 between Owl and robot, all four combinations (`tests/interop_nt.py`) | yes | yes |
| The AdvantageKit example compiles against NEIA's `VisionIO` | yes | not tried (no 2027 AdvantageKit checked) |
| The AdvantageKit example receives observations from Owl on a Jetson, in NEIA's robot code in simulation | yes | not tried |

The tests ran on a Mac and in CI against the published alpha builds. Nothing has run on a Systemcore yet.

## Still open

- 2027 is still in alpha, so names can change before kickoff. CI builds both seasons on every push, so a breaking alpha shows up as a failing 2027 job. When that happens, update `wpilib-2026-imports.txt`, the compat files, and the version in `tusk/java/gradle.properties`.
- Owl finds the robot by team number. Once we have a Systemcore, check that its NT server is reachable the same way (address, port 5810).
- Tusk converts NT message times to capture times with the robot's `Timer.getTimestamp()`. On 2027 that's `RobotController.getTime()`; confirm on Systemcore that NT's local timestamps use the same clock.
- The AdvantageKit example adapter targets the 2026 template; 2027 AdvantageKit hasn't been checked.

### Timestamp units

NetworkTables timestamps are in microseconds in 2026 and in Java 2027 alpha-6, but in nanoseconds in RobotPy 2027 alpha-7. Tusk RobotPy checks which release is installed and converts accordingly. Tusk Java assumes microseconds, which is right for the alpha-6 it's pinned to; when upgrading Java past alpha-6, update its conversion and the regression test that covers it. Both versions convert to seconds before subtracting an observation's age or checking whether the heartbeat is fresh. The cross-season integration test checks the resulting capture times against the robot clock, as well as checking the message contents.
