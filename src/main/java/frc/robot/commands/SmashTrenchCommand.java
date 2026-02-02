package frc.robot.commands;

import com.therekrab.autopilot.APTarget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

/**
 * SmashTrench state machine command.
 *
 * <p>States:
 *
 * <ul>
 *   <li>ALIGN: align/setup before trench interaction
 *   <li>RUN: drive/run into/through trench
 *   <li>GOINGUP: climbing phase
 *   <li>LAND: landing/settling phase
 * </ul>
 */
public class SmashTrenchCommand extends Command {
  public enum State {
    ALIGN,
    RUN
  }

  private final RobotContainer robot;

  private State state = State.ALIGN;

  // --- Frozen line selection (set once at entry to ALIGN) ---
  private FieldConstants.TrenchApproachLine chosenLine = null;
  private Translation2d chosenClosestPoint = null;

  // --- Targets ---
  private Pose2d alignTargetPose = new Pose2d();
  private APTarget alignTarget = new APTarget(alignTargetPose);
  private Pose2d runTargetPose = new Pose2d();
  private APTarget runTarget = new APTarget(runTargetPose);
  private Pose2d runStartPose = new Pose2d();

  private final ProfiledPIDController headingController =
      new ProfiledPIDController(
          Constants.TrenchCommandConstants.HEADING_KP,
          Constants.TrenchCommandConstants.HEADING_KI,
          Constants.TrenchCommandConstants.HEADING_KD,
          new TrapezoidProfile.Constraints(
              Constants.TrenchCommandConstants.HEADING_MAX_VEL_RAD_PER_SEC,
              Constants.TrenchCommandConstants.HEADING_MAX_ACCEL_RAD_PER_SEC2));

  public SmashTrenchCommand(RobotContainer robot) {
    this.robot = robot;
    addRequirements(robot.drive);
    headingController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void initialize() {
    setState(State.ALIGN);
  }

  @Override
  public void execute() {
    Logger.recordOutput("SmashTrench/State", state.toString());
    switch (state) {
      case ALIGN -> align();
      case RUN -> runState();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    Logger.recordOutput("SmashTrench/Interrupted", interrupted);
    robot.drive.stop();
  }

  private void setState(State next) {
    state = next;
    Logger.recordOutput("SmashTrench/StateEnter", state.toString());

    if (state == State.ALIGN) {
      selectNearestTrenchLine(robot.drive.getPose().getTranslation());

      // Pose selection (including heading snap) is handled in FieldConstants.
      alignTargetPose = FieldConstants.getNearestTrenchPrePose(robot.drive.getPose());
      alignTarget =
          new APTarget(alignTargetPose)
              .withEntryAngle(chosenLine.approachHeading())
              .withVelocity(Constants.TrenchCommandConstants.ALIGN_END_VELOCITY_MPS);

      headingController.reset(robot.drive.getRotation().getRadians());
      Logger.recordOutput("Trench/AlignTargetPose", alignTargetPose);
      Logger.recordOutput("Trench/ChosenClosestPoint", chosenClosestPoint);
      Logger.recordOutput("Trench/ChosenApproachHeading", chosenLine.approachHeading());
    } else if (state == State.RUN) {
      runStartPose = alignTargetPose;

      Translation2d u = chosenLine.dirUnit();
      Translation2d targetPoint =
          runStartPose
              .getTranslation()
              .plus(u.times(Constants.TrenchCommandConstants.RUN_PUSH_DISTANCE_METERS));
      runTargetPose = new Pose2d(targetPoint, alignTargetPose.getRotation());
      runTarget = new APTarget(runTargetPose).withEntryAngle(chosenLine.approachHeading());

      headingController.reset(robot.drive.getRotation().getRadians());
      Logger.recordOutput("Trench/RunStartPose", runStartPose);
      Logger.recordOutput("Trench/RunTargetPose", runTargetPose);
    }
  }

  private void selectNearestTrenchLine(Translation2d robotTranslation) {
    FieldConstants.TrenchApproachLine bestLine = null;
    double bestDist = Double.POSITIVE_INFINITY;
    boolean bestBehind = false;
    Translation2d bestClosestPoint = null;

    for (FieldConstants.TrenchApproachLine line : FieldConstants.TRENCH_APPROACH_LINES) {
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
        // Tie-breaker: prefer behind=true
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
    Rotation2d headingRef = alignTargetPose.getRotation();

    double omega =
        headingController.calculate(
            robot.drive.getRotation().getRadians(), headingRef.getRadians());

    robot.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robot.drive.getRotation()));

    Logger.recordOutput("Trench/AlignHeadingRef", headingRef);
    Logger.recordOutput("Trench/AlignAutopilotTargetAngle", out.targetAngle());
    Logger.recordOutput("Trench/AlignVxCmdMps", vx);
    Logger.recordOutput("Trench/AlignVyCmdMps", vy);
    Logger.recordOutput("Trench/AlignOmegaCmdRps", omega);

    Pose2d pose = robot.drive.getPose();
    boolean yOk =
        Math.abs(pose.getY() - alignTargetPose.getY())
            <= Constants.TrenchCommandConstants.ALIGN_Y_TOL_METERS;
    double thetaErr =
        MathUtil.angleModulus(
            pose.getRotation().getRadians() - alignTargetPose.getRotation().getRadians());
    boolean headingOk =
        Math.abs(thetaErr) <= Math.toRadians(Constants.TrenchCommandConstants.ALIGN_THETA_TOL_DEG);
    Logger.recordOutput("Trench/AlignYOk", yOk);
    Logger.recordOutput("Trench/AlignHeadingOk", headingOk);

    if (yOk && headingOk) {
      robot.drive.stop();
      setState(State.RUN);
    }
  }

  private void runState() {
    var out =
        Constants.kAutopilot.calculate(
            robot.drive.getPose(), robot.drive.getChassisSpeeds(), runTarget);

    double vx = out.vx().in(Units.MetersPerSecond);
    double vy = out.vy().in(Units.MetersPerSecond);
    Rotation2d headingRef = runTargetPose.getRotation();

    double omega =
        headingController.calculate(
            robot.drive.getRotation().getRadians(), headingRef.getRadians());

    robot.drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, robot.drive.getRotation()));

    Logger.recordOutput("Trench/RunHeadingRef", headingRef);
    Logger.recordOutput("Trench/RunAutopilotTargetAngle", out.targetAngle());
    Logger.recordOutput("Trench/RunVxCmdMps", vx);
    Logger.recordOutput("Trench/RunVyCmdMps", vy);
    Logger.recordOutput("Trench/RunOmegaCmdRps", omega);

    Translation2d u = chosenLine.dirUnit();
    Translation2d delta =
        robot.drive.getPose().getTranslation().minus(runStartPose.getTranslation());
    double pushedMeters = delta.getX() * u.getX() + delta.getY() * u.getY();
    Logger.recordOutput("Trench/RunPushedMeters", pushedMeters);

    if (pushedMeters
        >= (Constants.TrenchCommandConstants.RUN_PUSH_DISTANCE_METERS
            - Constants.TrenchCommandConstants.RUN_PUSH_DONE_TOL_METERS)) {
      robot.drive.stop();
    }
  }
}
