// Example: plugging Tusk into the AdvantageKit vision template (the Vision / VisionIO
// classes in NEIA's 2026-Robot-Code). Not compiled with Tusk: it depends on your
// project's VisionIO. Copy it next to VisionIO.java and change the package.
//
// One VisionIOTusk covers every Owl camera (Owl batches them into one NT topic):
//
//   vision = new Vision(drive::addVisionMeasurement,
//                       new VisionIOTusk(new TuskSource(), drive::getRotation));
//
// Add TUSK to VisionIO.PoseObservationType. Imports below are WPILib 2026; for 2027 apply
// the renames in tusk/java/wpilib-2026-imports.txt in reverse (edu.wpi.first -> org.wpilib).

package org.neiacademy.robotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;
import strix.tusk.TuskSource;
import strix.tusk.VisionObservation;

public class VisionIOTusk implements VisionIO {
  private final TuskSource source;
  private final Supplier<Rotation2d> heading;

  /**
   * @param heading the robot's current heading (gyro or pose estimate), used to pick between
   *     the two candidate poses of a single-tag observation
   */
  public VisionIOTusk(TuskSource source, Supplier<Rotation2d> heading) {
    this.source = source;
    this.heading = heading;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    inputs.connected = source.isConnected();

    List<PoseObservation> poses = new ArrayList<>();
    Set<Integer> tagIds = new LinkedHashSet<>();
    for (var timed : source.poll()) {
      VisionObservation obs = timed.observation();
      tagIds.addAll(obs.tagIds());
      poses.add(
          new PoseObservation(
              timed.captureTimeSeconds(),
              pickPose(obs),
              obs.ambiguity(),
              obs.tagCount(),
              obs.avgTagDistance(),
              PoseObservationType.TUSK));
    }
    inputs.poseObservations = poses.toArray(new PoseObservation[0]);
    inputs.tagIds = tagIds.stream().mapToInt(Integer::intValue).toArray();
  }

  /** Multi-tag: robotPose. Single tag: the candidate whose heading matches the robot's. */
  private Pose3d pickPose(VisionObservation obs) {
    if (obs.tagCount() != 1) {
      return obs.robotPose();
    }
    Rotation2d h = heading.get();
    double errBest = Math.abs(obs.robotPose().toPose2d().getRotation().minus(h).getRadians());
    double errAlt = Math.abs(obs.altRobotPose().toPose2d().getRotation().minus(h).getRadians());
    return errBest <= errAlt ? obs.robotPose() : obs.altRobotPose();
  }
}
