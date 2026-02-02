// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.swerve.ModuleLimits;
import frc.robot.util.swerve.SwerveSetpoint;
import frc.robot.util.swerve.SwerveSetpointGenerator;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  // --- Setpoint generation (steering vel + drive accel limiting) ---

  private static final double DEFAULT_MAX_DRIVE_ACCEL_MPS2 = 40;
  private static final double DEFAULT_MAX_STEER_VEL_RAD_PER_SEC = 100.0;
  private final SwerveSetpointGenerator setpointGenerator =
      new SwerveSetpointGenerator(
          new SwerveDriveKinematics(getModuleTranslations()), getModuleTranslations());
  private SwerveSetpoint prevSetpoint = null;
  private double lastSetpointTimestampSec = Double.NaN;

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));
  // --- Field-relative acceleration estimation ---
  private double lastAccelTimestampSec = Double.NaN;
  private Translation2d lastFieldVel = new Translation2d();
  private Translation2d fieldAccelFiltered = new Translation2d();

  // Tune this: lower = more smoothing (slower response), higher = less smoothing (noisier)
  private static final double ACCEL_CUTOFF_HZ = 12.0; // 8~20Hz 常见

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              TunerConstants.FrontLeft.WheelRadius,
              TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
              TunerConstants.FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIO.GyroIOInputs gyroInputs = new GyroIO.GyroIOInputs();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  // --- Tilt detection (pitch/roll) ---
  private final Debouncer tiltTripDebouncer =
      new Debouncer(
          Constants.DrivetrainConstants.TILT_TRIP_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
  private final Debouncer tiltClearDebouncer =
      new Debouncer(
          Constants.DrivetrainConstants.TILT_CLEAR_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
  private boolean isTilted = false;

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        this::runVelocity,
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  private void updateFieldAccelerationEstimate() {
    // Use FPGA timestamp to match WPILib timing
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();

    ChassisSpeeds vField = getFieldRelativeSpeeds();
    Translation2d fieldVelNow =
        new Translation2d(vField.vxMetersPerSecond, vField.vyMetersPerSecond);

    if (Double.isNaN(lastAccelTimestampSec)) {
      // First sample: initialize
      lastAccelTimestampSec = now;
      lastFieldVel = fieldVelNow;
      fieldAccelFiltered = new Translation2d();
      return;
    }

    double dt = now - lastAccelTimestampSec;
    // Guard against weird dt (e.g., sim hiccups)
    if (dt <= 1e-4 || dt > 0.1) {
      lastAccelTimestampSec = now;
      lastFieldVel = fieldVelNow;
      // Do not update accel on bad dt
      return;
    }

    // Raw acceleration from finite difference
    Translation2d accelRaw = fieldVelNow.minus(lastFieldVel).div(dt);

    // First-order low-pass filter (exponential smoothing)
    double alpha = lowPassAlpha(ACCEL_CUTOFF_HZ, dt);
    fieldAccelFiltered =
        new Translation2d(
            fieldAccelFiltered.getX() + alpha * (accelRaw.getX() - fieldAccelFiltered.getX()),
            fieldAccelFiltered.getY() + alpha * (accelRaw.getY() - fieldAccelFiltered.getY()));

    lastAccelTimestampSec = now;
    lastFieldVel = fieldVelNow;

    // Optional logs to AdvantageKit
    Logger.recordOutput("Drive/FieldVel", fieldVelNow);
    Logger.recordOutput("Drive/FieldAccelRaw", accelRaw);
    Logger.recordOutput("Drive/FieldAccelFiltered", fieldAccelFiltered);
  }

  /** Applies a Limelight MegaTag2 pose update (translation only; heading not corrected). */
  private void updatePoseWithLimelightMegaTag2() {
    final String llName = "limelight";

    // Provide robot orientation to Limelight for MegaTag2 filtering.
    double yawDeg = getRotation().getDegrees();
    double yawRateDegPerSec = Math.toDegrees(getChassisSpeeds().omegaRadiansPerSecond);
    LimelightHelpers.SetRobotOrientation(llName, yawDeg, yawRateDegPerSec, 0.0, 0.0, 0.0, 0.0);

    // Get MegaTag2 pose estimate (WPILib Blue coordinate system).
    LimelightHelpers.PoseEstimate est =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(llName);
    if (est == null || est.tagCount <= 0) {
      return;
    }

    // Translation std dev based on TA -> dev interpolation.
    double ta = LimelightHelpers.getTA(llName);
    double xyStdDev =
        Math.max(0.05, Constants.VisionConstants.taToXYStdDevMeters.get(Math.max(0.0, ta)));

    // Heading should not be corrected: force rotation to current heading and give huge theta std
    // dev.
    Pose2d visionPoseNoHeading = new Pose2d(est.pose.getTranslation(), getRotation());

    addVisionMeasurement(
        visionPoseNoHeading,
        est.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, Constants.VisionConstants.thetaStdDevRad));

    Logger.recordOutput("Vision/Limelight/TA", ta);
    Logger.recordOutput("Vision/Limelight/XYStdDev", xyStdDev);
    Logger.recordOutput("Vision/Limelight/TagCount", est.tagCount);
    Logger.recordOutput("Vision/Limelight/Pose", visionPoseNoHeading);
  }

  private static double lowPassAlpha(double cutoffHz, double dt) {
    // alpha = 1 - exp(-2*pi*fc*dt)
    double x = -2.0 * Math.PI * cutoffHz * dt;
    return 1.0 - Math.exp(x);
  }
  /** Returns filtered field-relative linear acceleration (m/s^2). */
  public Translation2d getFieldRelativeAcceleration() {
    return fieldAccelFiltered;
  }

  @AutoLogOutput(key = "SwerveChassisAccel/MeasuredFieldOriented")
  public ChassisSpeeds getLoggedFieldRelativeAcceleration() {
    return new ChassisSpeeds(fieldAccelFiltered.getX(), fieldAccelFiltered.getY(), 0);
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data

    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }
    updateFieldAccelerationEstimate();
    updatePoseWithLimelightMegaTag2();

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    // Tilt detection (uses pitch/roll)
    updateTiltDetection();
  }

  private void updateTiltDetection() {
    double pitchRad = gyroInputs.pitchPosition.getRadians();
    double rollRad = gyroInputs.rollPosition.getRadians();

    // Absolute tilt angle from combining pitch+roll.
    // Model: the "up" direction after pitch+roll has z component cos(pitch)*cos(roll),
    // so tilt = acos(zUp). This behaves like sqrt(p^2+r^2) for small angles.
    double zUp = Math.cos(pitchRad) * Math.cos(rollRad);
    zUp = MathUtil.clamp(zUp, -1.0, 1.0);
    double tiltRad = Math.acos(zUp);
    double tiltDeg = Math.toDegrees(tiltRad);

    if (!isTilted) {
      if (tiltTripDebouncer.calculate(tiltDeg >= Constants.DrivetrainConstants.TILT_TRIP_DEG)) {
        isTilted = true;
        tiltClearDebouncer.calculate(false);
      }
    } else {
      if (tiltClearDebouncer.calculate(tiltDeg <= Constants.DrivetrainConstants.TILT_CLEAR_DEG)) {
        isTilted = false;
        tiltTripDebouncer.calculate(false);
      }
    }

    Logger.recordOutput("Drive/Tilt/PitchDeg", Math.toDegrees(pitchRad));
    Logger.recordOutput("Drive/Tilt/RollDeg", Math.toDegrees(rollRad));
    Logger.recordOutput("Drive/Tilt/AbsTiltDeg", tiltDeg);
    Logger.recordOutput("Drive/Tilt/IsTilted", isTilted);
  }

  public Rotation2d getPitch() {
    return gyroInputs.pitchPosition;
  }

  public Rotation2d getRoll() {
    return gyroInputs.rollPosition;
  }

  /** Returns absolute tilt angle (deg) from pitch+roll. */
  public double getTiltMagnitudeDeg() {
    double pitchRad = getPitch().getRadians();
    double rollRad = getRoll().getRadians();
    double zUp = Math.cos(pitchRad) * Math.cos(rollRad);
    zUp = MathUtil.clamp(zUp, -1.0, 1.0);
    return Math.toDegrees(Math.acos(zUp));
  }

  public boolean isTilted() {
    return isTilted;
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Estimate dt for setpoint limiting
    double now = Timer.getFPGATimestamp();
    double dt = Double.isNaN(lastSetpointTimestampSec) ? 0.02 : (now - lastSetpointTimestampSec);
    lastSetpointTimestampSec = now;
    // Guard against weird dt (sim hiccups / first cycle)
    if (dt <= 1e-4 || dt > 0.1) {
      dt = 0.02;
    }

    // Initialize previous setpoint from measured state (avoids startup discontinuities)
    if (prevSetpoint == null) {
      prevSetpoint = new SwerveSetpoint(getChassisSpeeds(), getModuleStates());
    }

    // Apply setpoint generator limits (robot-relative)
    double maxVel = getMaxLinearSpeedMetersPerSec();
    ModuleLimits limits =
        new ModuleLimits(maxVel, DEFAULT_MAX_DRIVE_ACCEL_MPS2, DEFAULT_MAX_STEER_VEL_RAD_PER_SEC);

    // Discretize desired speeds first for consistency with standard swerve control
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, dt);
    SwerveSetpoint generated =
        setpointGenerator.generateSetpoint(limits, prevSetpoint, discreteSpeeds, dt);
    SwerveModuleState[] setpointStates = generated.moduleStates();

    // Enforce global wheel speed limit
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, maxVel);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", generated.chassisSpeeds());

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);

    // Save the applied setpoint as the next previous setpoint (after optimization/flips)
    prevSetpoint = new SwerveSetpoint(generated.chassisSpeeds(), setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  @AutoLogOutput(key = "SwerveChassisSpeeds/MeasuredFieldOriented")
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getPose().getRotation());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
