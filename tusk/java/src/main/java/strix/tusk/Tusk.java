// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import java.util.Optional;
import java.util.OptionalDouble;

/** Pose selection shared by every Tusk consumer. Mirrors {@code tusk.select_pose} in RobotPy. */
public final class Tusk {
  private Tusk() {}

  /**
   * The pose to use from one observation, or empty to drop it.
   *
   * <p>Multi-tag: robotPose. Single tag: whichever candidate's heading is closest to {@code
   * heading} (the gyro / current estimate), dropped if it's still more than {@code
   * maxYawErrorRad} off.
   *
   * @param maxYawErrorRad empty = never drop
   */
  public static Optional<Pose2d> selectPose(
      VisionObservation obs, Rotation2d heading, OptionalDouble maxYawErrorRad) {
    Pose2d best = obs.robotPose().toPose2d();
    if (obs.tagCount() != 1) {
      return Optional.of(best);
    }
    Pose2d alt = obs.altRobotPose().toPose2d();
    double errBest = Math.abs(best.getRotation().minus(heading).getRadians());
    double errAlt = Math.abs(alt.getRotation().minus(heading).getRadians());
    Pose2d pose = errBest <= errAlt ? best : alt;
    double err = Math.min(errBest, errAlt);
    if (maxYawErrorRad.isPresent() && err > maxYawErrorRad.getAsDouble()) {
      return Optional.empty();
    }
    return Optional.of(pose);
  }

  /** {@link #selectPose(VisionObservation, Rotation2d, OptionalDouble)} that never drops. */
  public static Optional<Pose2d> selectPose(VisionObservation obs, Rotation2d heading) {
    return selectPose(obs, heading, OptionalDouble.empty());
  }
}
