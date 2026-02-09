package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.Constants.MegaTrackIterativeCommandConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.WantedState;
import org.littletonrobotics.junction.Logger;

/**
 * Moving-shot compensation using fixed-point iteration on flight time.
 *
 * <p>Solves for T such that:
 *
 * <pre>
 * r(T) = g - p - T*v
 * d(T) = |r(T)|
 * T = flightTimeMap(d(T))
 * </pre>
 *
 * using fixed-point iteration each cycle, then uses r(T*) for heading/distance.
 */
public class MegaTrackIterativeCommand extends Command {
  private enum State {
    ALIGN,
    SHOOT
  }

  private final RobotContainer robot;
  private final frc.robot.subsystems.drive.Drive drive;
  private final Translation2d targetTranslation;
  private final java.util.function.DoubleSupplier xSupplier;
  private final java.util.function.DoubleSupplier ySupplier;

  private final PIDController headingController =
      new PIDController(
          Constants.MegaTrackIterativeCommandConstants.HEADING_KP,
          Constants.MegaTrackIterativeCommandConstants.HEADING_KI,
          Constants.MegaTrackIterativeCommandConstants.HEADING_KD);

  private double lastFlightTimeSec = AutoShootConstants.FlyTime;
  private Rotation2d heldHeading = new Rotation2d();
  private State state = State.ALIGN;

  // Per-cycle values (kept as fields to avoid long argument lists)
  private Translation2d shotOrigin = new Translation2d();
  private ChassisSpeeds vField = new ChassisSpeeds();
  private double tSec = AutoShootConstants.FlyTime;
  private Translation2d r = new Translation2d();
  private double dist = 0.0;
  private double shooterRps = 0.0;
  private double hoodDeg = 0.0;
  private boolean distOk = false;
  private boolean triggerHeld = false;
  private double flywheelErrRps = 0.0;
  private double hoodErrDeg = 0.0;
  private double headingErrRad = 0.0;

  public MegaTrackIterativeCommand(RobotContainer robot, Translation2d targetTranslation) {
    this.robot = robot;
    this.drive = robot.drive;
    this.targetTranslation = targetTranslation;
    this.xSupplier = robot.getDriveXSupplier();
    this.ySupplier = robot.getDriveYSupplier();
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(
        robot.drive, robot.shooter, robot.hood, robot.feeder, robot.indexer, robot.intake);
  }

  private Translation2d getShotOriginField() {
    Pose2d pose = drive.getPose();
    Translation2d offsetRobot =
        new Translation2d(
            Constants.ShooterConstants.FLYWHEEL_OFFSET_X_METERS,
            Constants.ShooterConstants.FLYWHEEL_OFFSET_Y_METERS);
    Translation2d offsetField = offsetRobot.rotateBy(pose.getRotation());
    return pose.getTranslation().plus(offsetField);
  }

  /** Returns r(T) = g - p - T*v (all in field frame). */
  private Translation2d compensatedVector(
      Translation2d shotOrigin, ChassisSpeeds vField, double tSec) {
    return targetTranslation
        .minus(shotOrigin)
        .minus(new Translation2d(vField.vxMetersPerSecond, vField.vyMetersPerSecond).times(tSec));
  }

  private double iterateFlightTimeSec(Translation2d shotOrigin, ChassisSpeeds vField) {
    double t =
        MathUtil.clamp(
            lastFlightTimeSec,
            Constants.MegaTrackIterativeCommandConstants.MIN_T_SEC,
            Constants.MegaTrackIterativeCommandConstants.MAX_T_SEC);

    // If last T is invalid/uninitialized, seed with map(distance at T=0)
    if (!Double.isFinite(t)) {
      double d0 = targetTranslation.minus(shotOrigin).getNorm();
      t = AutoShootConstants.flightTimeMap.get(d0);
    }

    for (int i = 0; i < Constants.MegaTrackIterativeCommandConstants.MAX_ITERS; i++) {
      Translation2d r = compensatedVector(shotOrigin, vField, t);
      double d = r.getNorm();
      double tNext = AutoShootConstants.flightTimeMap.get(d);
      tNext =
          MathUtil.clamp(
              tNext,
              Constants.MegaTrackIterativeCommandConstants.MIN_T_SEC,
              Constants.MegaTrackIterativeCommandConstants.MAX_T_SEC);

      double relax = Constants.MegaTrackIterativeCommandConstants.RELAX;
      double tNew = (1.0 - relax) * t + relax * tNext;
      Logger.recordOutput(
          Constants.MegaTrackIterativeCommandConstants.LOG_PREFIX + "/Iter/T_" + i, t);
      Logger.recordOutput(
          Constants.MegaTrackIterativeCommandConstants.LOG_PREFIX + "/Iter/D_" + i, d);
      Logger.recordOutput(
          Constants.MegaTrackIterativeCommandConstants.LOG_PREFIX + "/Iter/TNext_" + i, tNext);

      if (Math.abs(tNew - t) < Constants.MegaTrackIterativeCommandConstants.EPS_T_SEC) {
        t = tNew;
        break;
      }
      t = tNew;
    }

    lastFlightTimeSec = t;
    return t;
  }

  @Override
  public void initialize() {
    headingController.reset();
    robot.intake.setWantedState(WantedState.FLICK_BACK);
    heldHeading = drive.getRotation();
    state = State.ALIGN;
  }

  private void stopFeed() {
    robot.feeder.stop();
    robot.indexer.stop();
  }

  private void runFeed() {
    robot.feeder.setVelocity(Constants.MegaTrackIterativeCommandConstants.FEEDER_RPS);
    robot.indexer.setVoltage(Constants.MegaTrackIterativeCommandConstants.INDEXER_VOLTS);
  }

  private void traceTarget() {
    shotOrigin = getShotOriginField();
    vField = drive.getFieldRelativeSpeeds();

    tSec = iterateFlightTimeSec(shotOrigin, vField);
    r = compensatedVector(shotOrigin, vField, tSec);
    dist = r.getNorm();

    // Heading target comes from compensated vector (if too close, just hold current).
    if (dist > 1e-3) {
      heldHeading = r.getAngle();
    }

    // Driver translation (field-relative)
    Translation2d linearVelocity =
        DriveCommands.getLinearVelocityFromJoysticks(
            xSupplier.getAsDouble(), ySupplier.getAsDouble());

    // Heading control + simple omega FF using rDot=-v
    double omegaPid =
        headingController.calculate(drive.getRotation().getRadians(), heldHeading.getRadians());
    double den = Math.max(dist * dist, 0.40 * 0.40);
    double omegaFf =
        (r.getX() * (-vField.vyMetersPerSecond) - r.getY() * (-vField.vxMetersPerSecond)) / den;
    double omega =
        MathUtil.clamp(
            omegaPid + omegaFf,
            -MegaTrackIterativeCommandConstants.MAX_OMEGA,
            MegaTrackIterativeCommandConstants.MAX_OMEGA);

    ChassisSpeeds fieldRelative =
        new ChassisSpeeds(
            linearVelocity.getX() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            linearVelocity.getY() * AutoShootConstants.MAX_SHOOTING_VELOCITY,
            omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelative,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));

    // Shooter setpoints based on compensated distance
    shooterRps = AutoShootConstants.shooterSpeedMap.get(dist);
    hoodDeg = AutoShootConstants.hoodAngleMap.get(dist);
    robot.shooter.setVelocity(shooterRps);
    robot.hood.setAngle(hoodDeg);

    // Gating values
    distOk =
        dist >= Constants.MegaTrackIterativeCommandConstants.MIN_SHOOT_DIST_METERS
            && dist <= Constants.MegaTrackIterativeCommandConstants.MAX_SHOOT_DIST_METERS;
    triggerHeld =
        robot.getRightTriggerAxisSupplier().getAsDouble()
            > Constants.MegaTrackIterativeCommandConstants.TRIGGER_AXIS_THRESHOLD;
    flywheelErrRps = shooterRps - robot.shooter.getFlywheelVelocityRps();
    hoodErrDeg = hoodDeg - robot.hood.getAngleDeg();
    headingErrRad =
        MathUtil.angleModulus(heldHeading.getRadians() - drive.getRotation().getRadians());

    // Logs
    String lp = Constants.MegaTrackIterativeCommandConstants.LOG_PREFIX;
    Logger.recordOutput(lp + "/State", state.toString());
    Logger.recordOutput(lp + "/ShotOrigin", shotOrigin);
    Logger.recordOutput(lp + "/T", tSec);
    Logger.recordOutput(lp + "/TargetPosition", r.plus(shotOrigin));
    Logger.recordOutput(lp + "/Dist", dist);
    Logger.recordOutput(lp + "/HeldHeadingDeg", heldHeading.getDegrees());
    Logger.recordOutput(lp + "/TriggerHeld", triggerHeld);
    Logger.recordOutput(lp + "/FlywheelErrRps", flywheelErrRps);
    Logger.recordOutput(lp + "/HoodErrDeg", hoodErrDeg);
    Logger.recordOutput(lp + "/HeadingErrDeg", Math.toDegrees(headingErrRad));
  }

  private boolean okToEnterShoot() {
    boolean ok =
        distOk
            && triggerHeld
            && Math.abs(flywheelErrRps)
                <= Constants.MegaTrackIterativeCommandConstants.ENTER_FLYWHEEL_RPS_TOL
            && Math.abs(hoodErrDeg)
                <= Constants.MegaTrackIterativeCommandConstants.ENTER_HOOD_DEG_TOL
            && Math.abs(headingErrRad)
                <= Constants.MegaTrackIterativeCommandConstants.ENTER_HEADING_TOL_RAD;
    Logger.recordOutput(
        Constants.MegaTrackIterativeCommandConstants.LOG_PREFIX + "/OkEnterShoot", ok);
    return ok;
  }

  private boolean okToStayShoot() {
    boolean ok =
        distOk
            && triggerHeld
            && Math.abs(flywheelErrRps)
                <= Constants.MegaTrackIterativeCommandConstants.EXIT_FLYWHEEL_RPS_TOL
            && Math.abs(hoodErrDeg)
                <= Constants.MegaTrackIterativeCommandConstants.EXIT_HOOD_DEG_TOL
            && Math.abs(headingErrRad)
                <= Constants.MegaTrackIterativeCommandConstants.EXIT_HEADING_TOL_RAD;
    Logger.recordOutput(
        Constants.MegaTrackIterativeCommandConstants.LOG_PREFIX + "/OkStayShoot", ok);
    return ok;
  }

  private void align() {
    traceTarget();
    stopFeed();
    if (okToEnterShoot()) {
      state = State.SHOOT;
    }
  }

  private void shoot() {
    traceTarget();
    if (!okToStayShoot()) {
      stopFeed();
      state = State.ALIGN;
      return;
    }
    runFeed();
  }

  @Override
  public void execute() {
    switch (state) {
      case ALIGN -> align();
      case SHOOT -> shoot();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    robot.feeder.stop();
    robot.indexer.stop();
    robot.hood.stop();
    robot.shooter.stop();
    robot.intake.setWantedState(WantedState.UP_STOW_STOP);
  }
}
