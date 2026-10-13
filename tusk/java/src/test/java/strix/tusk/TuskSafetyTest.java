package strix.tusk;

import static org.junit.jupiter.api.Assertions.*;
import java.util.Optional;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.*;
import org.wpilib.math.estimator.DifferentialDrivePoseEstimator;
import org.wpilib.math.kinematics.DifferentialDriveKinematics;

class TuskSafetyTest {
  private VisionObservation observation(double confidence, int tags) {
    return new VisionObservation(0, new Pose3d(3, 3, 0, new Rotation3d()),
        new Pose3d(4, 3, 0, new Rotation3d(0, 0, Math.PI)), .08, confidence, tags, 2, .1, 1);
  }
  @Test void pinnedDependenciesUseMicroseconds() {
    assertEquals(1.25, WpilibCompat.ntSeconds(1_250_000), 1e-12);
  }

  @Test void actualEstimatorProvidesHistoricalHeading() {
    var estimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(.6),
        new Rotation2d(), 0, 0, new Pose2d(3, 3, new Rotation2d()));
    estimator.updateWithTime(10, new Rotation2d(), 0, 0);
    estimator.updateWithTime(10.08, Rotation2d.fromDegrees(14.4), 0, 0);
    assertTrue(Tusk.selectTimedPose(observation(1, 1), 10, 10.08, estimator::sampleAt,
        estimator.getEstimatedPosition(), true, OptionalDouble.of(1),
        OptionalDouble.of(Math.toRadians(5)), OptionalDouble.of(2)).isPresent());
  }

  @Test void rejectsBadConfidenceAndCoefficients() {
    for (double value : new double[]{0, -1, Double.NaN, Double.POSITIVE_INFINITY}) {
      assertFalse(Tusk.validObservation(observation(value, 1)));
      assertThrows(IllegalArgumentException.class,
          () -> Tusk.stdDevs(observation(1, 1), value, .03, false));
    }
    assertEquals(Double.POSITIVE_INFINITY, Tusk.stdDevs(observation(1, 1), .01, .03, false).get(2, 0));
  }
  @Test void delayedRotatingFrameUsesHistoricalHeading() {
    var selected = Tusk.selectTimedPose(observation(1, 1), 10, 10.08,
        timestamp -> { assertEquals(10, timestamp); return Optional.of(new Pose2d(3, 3, new Rotation2d())); },
        new Pose2d(3, 3, Rotation2d.fromDegrees(14.4)), true,
        OptionalDouble.of(2), OptionalDouble.of(Math.toRadians(5)), OptionalDouble.of(2));
    assertTrue(selected.isPresent());
  }
  @Test void missingHistoryDropsEnabledObservation() {
    for (boolean enabled : new boolean[]{true, false}) {
      var selected = Tusk.selectTimedPose(observation(1, 1), 10, 10.08,
          timestamp -> Optional.empty(), new Pose2d(3, 3, new Rotation2d()), enabled,
          OptionalDouble.of(2), OptionalDouble.of(.1), OptionalDouble.of(2));
      assertEquals(!enabled, selected.isPresent());
    }
  }
  @Test void rejectsStaleFutureAndNonfiniteTimes() {
    for (double timestamp : new double[]{8, 11, Double.NaN, Double.POSITIVE_INFINITY})
      assertFalse(Tusk.validCaptureTime(timestamp, 10));
  }
  @Test void disabledRobotCanSeedFarFromDefaultOrigin() {
    for (boolean history : new boolean[]{false, true}) {
      assertTrue(Tusk.selectTimedPose(observation(1, 2), 10, 10.08,
          timestamp -> history ? Optional.of(new Pose2d()) : Optional.empty(), new Pose2d(), false,
          OptionalDouble.of(2), OptionalDouble.of(.1), OptionalDouble.of(2)).isPresent());
    }
  }

  @Test void rejectsMultiTagJump() {
    assertTrue(Tusk.selectTimedPose(observation(1, 2), 10, 10.08,
        timestamp -> Optional.of(new Pose2d(10, 3, new Rotation2d())), new Pose2d(), true,
        OptionalDouble.of(2), OptionalDouble.of(.1), OptionalDouble.of(2)).isEmpty());
  }
}
