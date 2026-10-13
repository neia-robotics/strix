// Copyright (c) 2026 Strix contributors (NEIA Robotics 9644, Choate Robotics 7407)
// Use of this source code is governed by the MIT license in the LICENSE file at the
// root of this repository.

package strix.tusk;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.estimator.PoseEstimator;
import org.wpilib.system.Timer;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.driverstation.RobotState;
import org.wpilib.util.Preferences;
import java.util.ArrayList;
import java.util.List;
import java.util.OptionalDouble;
import java.util.Optional;
import java.util.function.DoubleFunction;

/**
 * Minimal consumer: feeds every observation into a pose estimator, with std devs from WPILib
 * Preferences (tunable in Elastic under /Preferences/Vision/ and persisted on the roboRIO).
 * Mirrors {@code tusk.TuskVision} (BatchedVision) in RobotPy, with the same Preference keys.
 *
 * <pre>
 *   Vision/XYStdDevCoefficient      xy std dev (m)      = coefficient * stdDevFactor
 *   Vision/ThetaStdDevCoefficient   theta std dev (rad) = coefficient * stdDevFactor
 *   Vision/TrustSingleTagTheta      false: single-tag heading ignored (gyro wins)
 * </pre>
 *
 * <p>Usage, e.g. in a drive subsystem's periodic():
 *
 * <pre>
 *   vision.update(poseEstimator); // uses sampleAt() for each capture timestamp
 * </pre>
 */
public class TuskVision {
  /** Matches the signature of SwerveDrivePoseEstimator.addVisionMeasurement. */
  @FunctionalInterface
  public interface VisionConsumer {
    void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> stdDevs);
  }

  /** What happened to one observation in the last {@link #update} call. */
  public record Result(
      double captureTimeSeconds, VisionObservation observation, boolean accepted) {}

  public static final String XY_KEY = "Vision/XYStdDevCoefficient";
  public static final String THETA_KEY = "Vision/ThetaStdDevCoefficient";
  public static final String TRUST_THETA_KEY = "Vision/TrustSingleTagTheta";

  private final TuskSource source;
  private final OptionalDouble maxSingleTagJumpM;
  private final OptionalDouble maxMultiTagJumpM;
  private final OptionalDouble maxSingleTagYawErrorRad;
  private List<Result> lastObservations = List.of();

  /** Defaults: single tags rejected if more than 1 m or 5 deg from the estimate while enabled. */
  public TuskVision() {
    this(new TuskSource(), OptionalDouble.of(1.0), OptionalDouble.of(5.0));
  }

  /**
   * While enabled, single-tag observations are rejected if they're more than {@code
   * maxSingleTagJumpM} from the capture-time estimate or their heading is more than {@code
   * maxSingleTagYawErrorDeg} off it. Empty disables either check. While disabled valid measurements are
   * accepted, so vision can seed the starting pose.
   */
  public TuskVision(
      TuskSource source, OptionalDouble maxSingleTagJumpM, OptionalDouble maxSingleTagYawErrorDeg) {
    this(source, maxSingleTagJumpM, maxSingleTagYawErrorDeg, OptionalDouble.of(2.0));
  }

  public TuskVision(TuskSource source, OptionalDouble maxSingleTagJumpM,
      OptionalDouble maxSingleTagYawErrorDeg, OptionalDouble maxMultiTagJumpM) {
    for (var limit : List.of(maxSingleTagJumpM, maxSingleTagYawErrorDeg, maxMultiTagJumpM)) {
      if (limit.isPresent() && (!Double.isFinite(limit.getAsDouble()) || limit.getAsDouble() <= 0))
        throw new IllegalArgumentException("pose rejection limits must be finite and positive");
    }
    this.source = source;
    this.maxMultiTagJumpM = maxMultiTagJumpM;
    this.maxSingleTagJumpM = maxSingleTagJumpM;
    this.maxSingleTagYawErrorRad =
        maxSingleTagYawErrorDeg.isPresent()
            ? OptionalDouble.of(Math.toRadians(maxSingleTagYawErrorDeg.getAsDouble()))
            : OptionalDouble.empty();
    // init only sets a value if the key doesn't exist yet, so tuned values survive deploys.
    Preferences.initDouble(XY_KEY, 0.01);
    Preferences.initDouble(THETA_KEY, 0.03);
    Preferences.initBoolean(TRUST_THETA_KEY, false);
  }

  public TuskSource getSource() {
    return source;
  }

  public boolean isConnected() {
    return source.isConnected();
  }

  /** Every observation from the last {@link #update} call, with whether it was accepted. */
  public List<Result> getLastObservations() {
    return lastObservations;
  }

  public Matrix<N3, N1> stdDevs(VisionObservation obs) {
    return Tusk.stdDevs(obs, Preferences.getDouble(XY_KEY, 0.01),
        Preferences.getDouble(THETA_KEY, 0.03), Preferences.getBoolean(TRUST_THETA_KEY, false));
  }

  /** Preferred API: use WPILib's existing history instead of comparing to the current heading. */
  public int update(PoseEstimator<?> estimator) {
    return update(estimator.getEstimatedPosition(), estimator::sampleAt, estimator::addVisionMeasurement);
  }

  /** Feeds valid measurements using capture-time history. Returns the number accepted. */
  public int update(Pose2d currentEstimate, DoubleFunction<Optional<Pose2d>> sampleAt,
      VisionConsumer consumer) {
    boolean enabled = RobotState.isEnabled();
    int accepted = 0;
    List<Result> results = new ArrayList<>();
    for (var timed : source.poll()) {
      VisionObservation obs = timed.observation();
      var pose = Tusk.selectTimedPose(obs, timed.captureTimeSeconds(), Timer.getTimestamp(),
          sampleAt, currentEstimate, enabled, maxSingleTagJumpM,
          maxSingleTagYawErrorRad, maxMultiTagJumpM);
      boolean ok = pose.isPresent();
      Matrix<N3, N1> deviations = null;
      if (ok) {
        try { deviations = stdDevs(obs); }
        catch (IllegalArgumentException e) { ok = false; }
      }
      if (ok) {
        consumer.accept(pose.get(), timed.captureTimeSeconds(), deviations);
        accepted++;
      }
      results.add(new Result(timed.captureTimeSeconds(), obs, ok));
    }
    lastObservations = results;
    return accepted;
  }
}
