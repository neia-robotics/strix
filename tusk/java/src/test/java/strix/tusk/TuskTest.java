// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;

/** Same cases as tusk/robotpy/tests/test_tusk.py. */
class TuskTest {
  private static VisionObservation obs(int tagCount, double yawBest, double yawAlt) {
    return new VisionObservation(
        0,
        new Pose3d(new Translation3d(1.0, 1.0, 0.0), new Rotation3d(0.0, 0.0, yawBest)),
        new Pose3d(new Translation3d(2.0, 2.0, 0.0), new Rotation3d(0.0, 0.0, yawAlt)),
        0.0,
        1.0,
        tagCount,
        2.0,
        0.1,
        1L);
  }

  @Test
  void multiTagUsesBestPose() {
    var pose = Tusk.selectPose(obs(2, 0.0, 3.0), new Rotation2d(3.0));
    assertEquals(1.0, pose.orElseThrow().getX(), 1e-9);
  }

  @Test
  void singleTagPicksCandidateMatchingHeading() {
    var pose = Tusk.selectPose(obs(1, 0.0, Math.PI / 2), new Rotation2d(Math.PI / 2));
    assertEquals(2.0, pose.orElseThrow().getX(), 1e-9);
  }

  @Test
  void singleTagDroppedWhenHeadingTooFar() {
    var dropped =
        Tusk.selectPose(
            obs(1, 0.0, Math.PI),
            new Rotation2d(Math.PI / 2),
            OptionalDouble.of(Math.toRadians(5)));
    assertTrue(dropped.isEmpty());
    assertTrue(Tusk.selectPose(obs(1, 0.0, Math.PI), new Rotation2d(Math.PI / 2)).isPresent());
  }
}
