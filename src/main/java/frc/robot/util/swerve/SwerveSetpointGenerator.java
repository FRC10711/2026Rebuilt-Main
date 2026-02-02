package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class SwerveSetpointGenerator {
  private static final double EPS = 1e-9;

  private final SwerveDriveKinematics kinematics;
  private final Translation2d[] moduleLocations;

  public SwerveSetpointGenerator(
      SwerveDriveKinematics kinematics, Translation2d[] moduleLocations) {
    this.kinematics = kinematics;
    this.moduleLocations = moduleLocations;
  }

  private static boolean epsilonEquals(double a, double b) {
    return Math.abs(a - b) < EPS;
  }

  private static boolean isStop(ChassisSpeeds speeds) {
    return epsilonEquals(speeds.vxMetersPerSecond, 0.0)
        && epsilonEquals(speeds.vyMetersPerSecond, 0.0)
        && epsilonEquals(speeds.omegaRadiansPerSecond, 0.0);
  }

  /** True if it would be faster to flip the wheel direction and aim 180° away. */
  private boolean flipHeading(Rotation2d prevToGoal) {
    return Math.abs(prevToGoal.getRadians()) > Math.PI / 2.0;
  }

  private double unwrapAngle(double ref, double angle) {
    double diff = angle - ref;
    if (diff > Math.PI) {
      return angle - 2.0 * Math.PI;
    } else if (diff < -Math.PI) {
      return angle + 2.0 * Math.PI;
    } else {
      return angle;
    }
  }

  @FunctionalInterface
  private interface Function2d {
    double f(double x, double y);
  }

  /** Find the root of the generic 2D parametric function 'func' using regula falsi. */
  private double findRoot(
      Function2d func,
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      int iterations_left) {
    if (iterations_left < 0 || epsilonEquals(f_0, f_1)) {
      return 1.0;
    }
    var s_guess = Math.max(0.0, Math.min(1.0, -f_0 / (f_1 - f_0)));
    var x_guess = (x_1 - x_0) * s_guess + x_0;
    var y_guess = (y_1 - y_0) * s_guess + y_0;
    var f_guess = func.f(x_guess, y_guess);
    if (Math.signum(f_0) == Math.signum(f_guess)) {
      return s_guess
          + (1.0 - s_guess)
              * findRoot(func, x_guess, y_guess, f_guess, x_1, y_1, f_1, iterations_left - 1);
    } else {
      return s_guess
          * findRoot(func, x_0, y_0, f_0, x_guess, y_guess, f_guess, iterations_left - 1);
    }
  }

  protected double findSteeringMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_deviation,
      int max_iterations) {
    f_1 = unwrapAngle(f_0, f_1);
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_deviation) {
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_deviation;
    Function2d func = (x, y) -> unwrapAngle(f_0, Math.atan2(y, x)) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  protected double findDriveMaxS(
      double x_0,
      double y_0,
      double f_0,
      double x_1,
      double y_1,
      double f_1,
      double max_vel_step,
      int max_iterations) {
    double diff = f_1 - f_0;
    if (Math.abs(diff) <= max_vel_step) {
      return 1.0;
    }
    double offset = f_0 + Math.signum(diff) * max_vel_step;
    Function2d func = (x, y) -> Math.hypot(x, y) - offset;
    return findRoot(func, x_0, y_0, f_0 - offset, x_1, y_1, f_1 - offset, max_iterations);
  }

  // ------------------------------
  // NEW: drive-accel feasibility + bisection helpers
  // ------------------------------

  /**
   * Checks drive accel limit using the module speed magnitudes. Constraint: |v_i(new)| must be
   * within +/- maxVelStep of |v_i(prev)|.
   *
   * <p>We use magnitudes (abs) to match the original intent of limiting wheel speed change.
   */
  private boolean satisfiesDriveAccel(
      SwerveSetpoint prevSetpoint, ChassisSpeeds candidateSpeeds, double maxVelStep) {

    SwerveModuleState[] candStates = kinematics.toSwerveModuleStates(candidateSpeeds);

    // Safety: ensure candidate respects max wheel speed if you ever feed weird omega/translation
    // mixes.
    // (Optional) You can remove this if you don't want desaturation to interfere.
    // NOTE: desaturating here can slightly change the solution, but generally helps robustness.
    // SwerveDriveKinematics.desaturateWheelSpeeds(candStates, limits.maxDriveVelocity());

    for (int i = 0; i < candStates.length; i++) {
      double prevMag = Math.abs(prevSetpoint.moduleStates()[i].speedMetersPerSecond);
      double newMag = Math.abs(candStates[i].speedMetersPerSecond);
      if (Math.abs(newMag - prevMag) > maxVelStep + 1e-12) {
        return false;
      }
    }
    return true;
  }

  /**
   * Bisection on s in [0, sMax] for a monotonic feasibility check: - if feasible at s, should be
   * feasible at smaller s (true for our accel constraint).
   */
  private double bisectMaxS(java.util.function.DoublePredicate feasible, double sMax, int iters) {
    double lo = 0.0;
    double hi = sMax;
    for (int k = 0; k < iters; k++) {
      double mid = 0.5 * (lo + hi);
      if (feasible.test(mid)) {
        lo = mid;
      } else {
        hi = mid;
      }
    }
    return lo;
  }

  /**
   * Generate a new kinematically-feasible setpoint.
   *
   * @param limits Kinematic limits to respect
   * @param prevSetpoint Previous setpoint (usually from last iteration)
   * @param desiredState Desired chassis speeds (robot-relative)
   * @param dt Loop time in seconds
   */
  public SwerveSetpoint generateSetpoint(
      final ModuleLimits limits,
      final SwerveSetpoint prevSetpoint,
      ChassisSpeeds desiredState,
      double dt) {

    final Translation2d[] modules = moduleLocations;
    SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);

    // Make sure desiredState respects velocity limits.
    if (limits.maxDriveVelocity() > 0.0) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocity());
      desiredState = kinematics.toChassisSpeeds(desiredModuleState);
    }

    // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so
    // just use the previous angle.
    boolean need_to_steer = true;
    if (isStop(desiredState)) {
      need_to_steer = false;
      for (int i = 0; i < modules.length; ++i) {
        desiredModuleState[i].angle = prevSetpoint.moduleStates()[i].angle;
        desiredModuleState[i].speedMetersPerSecond = 0.0;
      }
    }

    // For each module, compute local Vx and Vy vectors.
    double[] prev_vx = new double[modules.length];
    double[] prev_vy = new double[modules.length];
    Rotation2d[] prev_heading = new Rotation2d[modules.length];
    double[] desired_vx = new double[modules.length];
    double[] desired_vy = new double[modules.length];
    Rotation2d[] desired_heading = new Rotation2d[modules.length];
    boolean all_modules_should_flip = true;

    for (int i = 0; i < modules.length; ++i) {
      prev_vx[i] =
          prevSetpoint.moduleStates()[i].angle.getCos()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_vy[i] =
          prevSetpoint.moduleStates()[i].angle.getSin()
              * prevSetpoint.moduleStates()[i].speedMetersPerSecond;
      prev_heading[i] = prevSetpoint.moduleStates()[i].angle;
      if (prevSetpoint.moduleStates()[i].speedMetersPerSecond < 0.0) {
        prev_heading[i] = prev_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
      }

      desired_vx[i] =
          desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
      desired_vy[i] =
          desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
      desired_heading[i] = desiredModuleState[i].angle;
      if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
        desired_heading[i] = desired_heading[i].rotateBy(Rotation2d.fromRadians(Math.PI));
      }

      if (all_modules_should_flip) {
        double required_rotation_rad =
            Math.abs(prev_heading[i].unaryMinus().rotateBy(desired_heading[i]).getRadians());
        if (required_rotation_rad < Math.PI / 2.0) {
          all_modules_should_flip = false;
        }
      }
    }

    if (all_modules_should_flip && !isStop(prevSetpoint.chassisSpeeds()) && !isStop(desiredState)) {
      // Likely faster to stop first, rotate modules, then accelerate.
      return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
    }

    // Interpolate between start and goal; find maximum s such that constraints are satisfied.
    double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds().vxMetersPerSecond;
    double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds().vyMetersPerSecond;
    double dtheta =
        desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

    double min_s = 1.0;

    // In cases where an individual module is stopped, remember the right steering angle to command.
    List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);

    // Enforce steering velocity limits.
    final double max_theta_step = dt * limits.maxSteeringVelocity();
    for (int i = 0; i < modules.length; ++i) {
      if (!need_to_steer) {
        overrideSteering.add(Optional.of(prevSetpoint.moduleStates()[i].angle));
        continue;
      }
      overrideSteering.add(Optional.empty());

      if (epsilonEquals(prevSetpoint.moduleStates()[i].speedMetersPerSecond, 0.0)) {
        // If module is stopped, limit purely on rotation in place.
        if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
          // Goal angle doesn't matter.
          overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates()[i].angle));
          continue;
        }
        var necessaryRotation =
            prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(desiredModuleState[i].angle);
        if (flipHeading(necessaryRotation)) {
          necessaryRotation = necessaryRotation.rotateBy(Rotation2d.fromRadians(Math.PI));
        }
        final double numStepsNeeded = Math.abs(necessaryRotation.getRadians()) / max_theta_step;
        if (numStepsNeeded <= 1.0) {
          overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
          continue;
        } else {
          overrideSteering.set(
              i,
              Optional.of(
                  prevSetpoint.moduleStates()[i].angle.rotateBy(
                      Rotation2d.fromRadians(
                          Math.signum(necessaryRotation.getRadians()) * max_theta_step))));
          min_s = 0.0;
          continue;
        }
      }

      if (min_s == 0.0) {
        continue;
      }

      final int kMaxIterations = 8;
      double s =
          findSteeringMaxS(
              prev_vx[i],
              prev_vy[i],
              prev_heading[i].getRadians(),
              desired_vx[i],
              desired_vy[i],
              desired_heading[i].getRadians(),
              max_theta_step,
              kMaxIterations);
      min_s = Math.min(min_s, s);
    }

    // ------------------------------
    // CHANGED: drive accel limits (decouple translation vs omega)
    // ------------------------------
    final double max_vel_step = dt * limits.maxDriveAcceleration();

    // 1) Find max translation progress sTrans <= min_s with omega held at prevOmega.
    //    This keeps the path direction from being "twisted" by omega when accel-limited.
    double sTrans = min_s;
    if (sTrans > 0.0 && !epsilonEquals(limits.maxDriveAcceleration(), 0.0)) {
      final double prevVx = prevSetpoint.chassisSpeeds().vxMetersPerSecond;
      final double prevVy = prevSetpoint.chassisSpeeds().vyMetersPerSecond;
      final double prevOm = prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

      // Quick accept if already feasible at sTrans.
      ChassisSpeeds candFullTrans =
          new ChassisSpeeds(prevVx + sTrans * dx, prevVy + sTrans * dy, prevOm);
      if (!satisfiesDriveAccel(prevSetpoint, candFullTrans, max_vel_step)) {
        // Bisect down.
        final int iters = 14;
        sTrans =
            bisectMaxS(
                (s) -> {
                  ChassisSpeeds c = new ChassisSpeeds(prevVx + s * dx, prevVy + s * dy, prevOm);
                  return satisfiesDriveAccel(prevSetpoint, c, max_vel_step);
                },
                sTrans,
                iters);
      }
    }

    // 2) With translation fixed at sTrans, find max omega progress sOmega <= sTrans.
    double sOmega = sTrans;
    if (sTrans > 0.0
        && !epsilonEquals(dtheta, 0.0)
        && !epsilonEquals(limits.maxDriveAcceleration(), 0.0)) {
      final double prevVx = prevSetpoint.chassisSpeeds().vxMetersPerSecond;
      final double prevVy = prevSetpoint.chassisSpeeds().vyMetersPerSecond;
      final double prevOm = prevSetpoint.chassisSpeeds().omegaRadiansPerSecond;

      final double vxFixed = prevVx + sTrans * dx;
      final double vyFixed = prevVy + sTrans * dy;

      ChassisSpeeds candFull = new ChassisSpeeds(vxFixed, vyFixed, prevOm + sOmega * dtheta);
      if (!satisfiesDriveAccel(prevSetpoint, candFull, max_vel_step)) {
        final int iters = 14;
        sOmega =
            bisectMaxS(
                (s) -> {
                  ChassisSpeeds c = new ChassisSpeeds(vxFixed, vyFixed, prevOm + s * dtheta);
                  return satisfiesDriveAccel(prevSetpoint, c, max_vel_step);
                },
                sOmega,
                iters);
      }
    }

    // Build retSpeeds using decoupled interpolation.
    ChassisSpeeds retSpeeds =
        new ChassisSpeeds(
            prevSetpoint.chassisSpeeds().vxMetersPerSecond + sTrans * dx,
            prevSetpoint.chassisSpeeds().vyMetersPerSecond + sTrans * dy,
            prevSetpoint.chassisSpeeds().omegaRadiansPerSecond + sOmega * dtheta);

    // (Optional safety) keep within max wheel speed.
    var retStates = kinematics.toSwerveModuleStates(retSpeeds);
    if (limits.maxDriveVelocity() > 0.0) {
      SwerveDriveKinematics.desaturateWheelSpeeds(retStates, limits.maxDriveVelocity());
      retSpeeds = kinematics.toChassisSpeeds(retStates);
      // Recompute states after saturation so later steering overrides are applied to final speeds.
      retStates = kinematics.toSwerveModuleStates(retSpeeds);
    }

    // Apply steering overrides + minimize rotation (same as original)
    for (int i = 0; i < modules.length; ++i) {
      final var maybeOverride = overrideSteering.get(i);
      if (maybeOverride.isPresent()) {
        var override = maybeOverride.get();
        if (flipHeading(retStates[i].angle.unaryMinus().rotateBy(override))) {
          retStates[i].speedMetersPerSecond *= -1.0;
        }
        retStates[i].angle = override;
      }

      final var deltaRotation =
          prevSetpoint.moduleStates()[i].angle.unaryMinus().rotateBy(retStates[i].angle);
      if (flipHeading(deltaRotation)) {
        retStates[i].angle = retStates[i].angle.rotateBy(Rotation2d.fromRadians(Math.PI));
        retStates[i].speedMetersPerSecond *= -1.0;
      }
    }

    return new SwerveSetpoint(retSpeeds, retStates);
  }
}
