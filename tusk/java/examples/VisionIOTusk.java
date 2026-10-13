// Example: plugging Tusk into the AdvantageKit vision template (the Vision / VisionIO
// classes in NEIA's 2026-Robot-Code). Not compiled with Tusk: it depends on your
// project's VisionIO. Copy it next to VisionIO.java and change the package.
//
// One VisionIOTusk covers every Owl camera (Owl batches them into one NT topic):
//
//   vision = new Vision(drive::addVisionMeasurement,
//                       new VisionIOTusk(new TuskSource(), drive::samplePoseAt, drive::getPose));
//
// Add TUSK to VisionIO.PoseObservationType. Imports below are WPILib 2026; for 2027 apply
// the renames in tusk/java/wpilib-2026-imports.txt in reverse (edu.wpi.first -> org.wpilib).

package org.neiacademy.robotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.DoubleFunction;
import java.util.function.Supplier;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import strix.tusk.Tusk;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import strix.tusk.TuskSource;
import strix.tusk.VisionObservation;

public class VisionIOTusk implements VisionIO {
  private final TuskSource source;
  private final DoubleFunction<Optional<Pose2d>> sampleAt;
  private final Supplier<Pose2d> currentPose;

  /** sampleAt is the pose estimator's capture-time lookup (timestamp in seconds). */
  public VisionIOTusk(TuskSource source, DoubleFunction<Optional<Pose2d>> sampleAt, Supplier<Pose2d> currentPose) {
    this.source = source;
    this.sampleAt = sampleAt;
    this.currentPose = currentPose;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = source.isConnected();

    List<PoseObservation> poses = new ArrayList<>();
    Set<Integer> tagIds = new LinkedHashSet<>();
    for (var timed : source.poll()) {
      VisionObservation obs = timed.observation();
      var chosen = Tusk.selectTimedPose(obs, timed.captureTimeSeconds(), Timer.getTimestamp(),
          sampleAt, currentPose.get(), DriverStation.isEnabled(),
          OptionalDouble.of(2), OptionalDouble.of(Math.toRadians(5)), OptionalDouble.of(2));
      if (chosen.isEmpty()) continue;
      tagIds.addAll(obs.tagIds());
      poses.add(
          new PoseObservation(
              timed.captureTimeSeconds(),
              chosen.get().equals(obs.robotPose().toPose2d()) ? obs.robotPose() : obs.altRobotPose(),
              obs.ambiguity(),
              obs.tagCount(),
              obs.avgTagDistance(),
              PoseObservationType.TUSK));
    }
    inputs.poseObservations = poses.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

}
