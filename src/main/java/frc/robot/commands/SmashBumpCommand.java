package frc.robot.commands;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

/**
 * Automatic bump traversal.
 *
 * <p>Flow:
 *
 * <ul>
 *   <li>ALIGN: use Autopilot to drive to pre-bump pose (no 90° snap)
 *   <li>RUN: sprint at fixed heading/speed, wait until tilt detected
 *   <li>GOING_UP: keep sprinting while tilted
 *   <li>DOWNHILL: after tilt clears, keep sprinting briefly then stop and finish
 * </ul>
 */
public class SmashBumpCommand extends Command {
  private enum State {
    ALIGN,
    RUN,
    GOING_UP,
    DOWNHILL,
    DONE
  }

  private final RobotContainer robot;

  private final PIDController headingController =
      new PIDController(
          Constants.BumpCommandConstants.HEADING_KP,
          Constants.BumpCommandConstants.HEADING_KI,
          Constants.BumpCommandConstants.HEADING_KD);

  // Frozen line selection (set once at entry to ALIGN)
  private FieldConstants.BumpApproachLine chosenLine = null;
  private Translation2d chosenClosestPoint = null;

  // Targets / references
  private Pose2d alignTargetPose = new Pose2d();
  private APTarget alignTarget = new APTarget(alignTargetPose);
  private Rotation2d sprintHeading = new Rotation2d();

  private State state = State.ALIGN;
  private double downhillStartSec = Double.NaN;

  public SmashBumpCommand(RobotContainer robot) {
    this.robot = robot;
    headingController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(robot.drive);
  }

  @Override
  public void initialize() {
    setState(State.ALIGN);
  }

  @Override
  public void execute() {
    Logger.recordOutput("SmashBump/State", state.toString());
    switch (state) {
      case ALIGN -> align();
      case RUN -> run();
      case GOING_UP -> goingUp();
      case DOWNHILL -> downhill();
      case DONE -> robot.drive.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return state == State.DONE;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("SmashBump/Interrupted", interrupted);
    robot.drive.stop();
  }

  private void setState(State next) {
    state = next;
    Logger.recordOutput("SmashBump/StateEnter", state.toString());

    if (state == State.ALIGN) {
      selectNearestLine(robot.drive.getPose().getTranslation());

      // Pre-bump pose (no 90° snap)
      alignTargetPose = FieldConstants.getNearestBumpPrePose(robot.drive.getPose());
      sprintHeading = chosenLine.approachHeading();
      alignTarget =
          new APTarget(alignTargetPose)
              .withEntryAngle(chosenLine.approachHeading())
              .withVelocity(Constants.BumpCommandConstants.ALIGN_END_VELOCITY_MPS);

      headingController.reset();
      Logger.recordOutput("Bump/AlignTargetPose", alignTargetPose);
      Logger.recordOutput("Bump/ChosenClosestPoint", chosenClosestPoint);
      Logger.recordOutput("Bump/SprintHeadingDeg", sprintHeading.getDegrees());
    } else if (state == State.DOWNHILL) {
      downhillStartSec = Timer.getFPGATimestamp();
    }
  }

  private void selectNearestLine(Translation2d robotTranslation) {
    FieldConstants.BumpApproachLine bestLine = null;
    double bestDist = Double.POSITIVE_INFINITY;
    boolean bestBehind = false;
    Translation2d bestClosestPoint = null;

    for (FieldConstants.BumpApproachLine line : FieldConstants.BUMP_APPROACH_LINES) {
      Translation2d closest = line.closestPointOnSegment(robotTranslation);
      double dist = line.distanceToSegment(robotTranslation);

      Translation2d delta = robotTranslation.minus(closest);
      boolean behind =
          delta.getX() * line.dirUnit().getX() + delta.getY() * line.dirUnit().getY() < 0.0;

      if (dist < bestDist - 1e-9) {
        bestLine = line;
        bestDist = dist;
        bestBehind = behind;
        bestClosestPoint = closest;
      } else if (Math.abs(dist - bestDist) <= 1e-9) {
        if (behind && !bestBehind) {
          bestLine = line;
          bestBehind = true;
          bestClosestPoint = closest;
        }
      }
    }

    chosenLine = bestLine;
    chosenClosestPoint = bestClosestPoint;
  }

  private void align() {
    var out =
        Constants.kAutopilot.calculate(
            robot.drive.getPose(), robot.drive.getChassisSpeeds(), alignTarget);

    double vx = out.vx().in(Units.MetersPerSecond);
    double vy = out.vy().in(Units.MetersPerSecond);

    double omega =
        headingController.calculate(
            robot.drive.getRotation().getRadians(), sprintHeading.getRadians());
    omega =
        MathUtil.clamp(
            omega,
            -robot.drive.getMaxAngularSpeedRadPerSec(),
            robot.drive.getMaxAngularSpeedRadPerSec());

    robot.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robot.drive.getRotation()));

    Logger.recordOutput("Bump/AlignVxCmdMps", vx);
    Logger.recordOutput("Bump/AlignVyCmdMps", vy);
    Logger.recordOutput("Bump/AlignOmegaCmdRps", omega);

    Pose2d pose = robot.drive.getPose();
    boolean xyOk =
        pose.getTranslation().getDistance(alignTargetPose.getTranslation())
            <= Constants.BumpCommandConstants.ALIGN_XY_TOL_METERS;
    double thetaErr =
        MathUtil.angleModulus(pose.getRotation().getRadians() - sprintHeading.getRadians());
    boolean headingOk =
        Math.abs(thetaErr) <= Math.toRadians(Constants.BumpCommandConstants.ALIGN_THETA_TOL_DEG);

    Logger.recordOutput("Bump/AlignXyOk", xyOk);
    Logger.recordOutput("Bump/AlignHeadingOk", headingOk);

    if (xyOk && headingOk) {
      robot.drive.stop();
      setState(State.RUN);
    }
  }

  private void sprint() {
    Translation2d u = chosenLine.dirUnit();
    double vx = u.getX() * Constants.BumpCommandConstants.SPRINT_SPEED_MPS;
    double vy = u.getY() * Constants.BumpCommandConstants.SPRINT_SPEED_MPS;

    double omega =
        headingController.calculate(
            robot.drive.getRotation().getRadians(), sprintHeading.getRadians());
    omega =
        MathUtil.clamp(
            omega,
            -robot.drive.getMaxAngularSpeedRadPerSec(),
            robot.drive.getMaxAngularSpeedRadPerSec());

    robot.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robot.drive.getRotation()));

    Logger.recordOutput("Bump/SprintVxCmdMps", vx);
    Logger.recordOutput("Bump/SprintVyCmdMps", vy);
    Logger.recordOutput("Bump/SprintOmegaCmdRps", omega);
    Logger.recordOutput("Bump/TiltDeg", robot.drive.getTiltMagnitudeDeg());
    Logger.recordOutput("Bump/IsTilted", robot.drive.isTilted());
  }

  private void run() {
    sprint();
    if (robot.drive.isTilted()) {
      setState(State.GOING_UP);
    }
  }

  private void goingUp() {
    sprint();
    if (!robot.drive.isTilted()) {
      setState(State.DOWNHILL);
    }
  }

  private void downhill() {
    sprint();
    if (!Double.isFinite(downhillStartSec)) {
      downhillStartSec = Timer.getFPGATimestamp();
    }
    double t = Timer.getFPGATimestamp() - downhillStartSec;
    Logger.recordOutput("Bump/DownhillTimeSec", t);
    if (t >= Constants.BumpCommandConstants.DOWNHILL_TIME_SEC) {
      robot.drive.stop();
      setState(State.DONE);
    }
  }
}
