package frc.robot.commands.Auto;

import static edu.wpi.first.units.Units.Degrees;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.MegaTrackIterativeCommand;
import frc.robot.commands.SmashBumpCommand;
import frc.robot.commands.SmashTrenchCommand;
import frc.robot.subsystems.intake.Intake;

public class UpOut extends SequentialCommandGroup {

  public UpOut(RobotContainer robotContainer) {
    PathPlannerPath Up1, Up2;
    try {
      Up1 = PathPlannerPath.fromChoreoTrajectory("Up1");
      Up2 = PathPlannerPath.fromChoreoTrajectory("Up2");
      addCommands(
          new InstantCommand(
              () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
      addCommands(AutoBuilder.followPath(Up1));
      addCommands(new SmashBumpCommand(robotContainer).withTimeout(2.7));
      addCommands(new MegaTrackIterativeCommand(robotContainer, false).withTimeout(4));
      addCommands(new SmashTrenchCommand(robotContainer).withTimeout(2.7));
      addCommands(
          new InstantCommand(
              () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
      addCommands(AutoBuilder.followPath(Up2));
      addCommands(new SmashBumpCommand(robotContainer).withTimeout(3.2));
      addCommands(new MegaTrackIterativeCommand(robotContainer, false));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  public static Pose2d getStartPose(Alliance alliance) {
    Pose2d bluePose2d = new Pose2d(4.465, 7.353, new Rotation2d(Degrees.of(-90)));
    if (alliance == Alliance.Blue) return bluePose2d;
    else return bluePose2d.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.k180deg);
  }
}
