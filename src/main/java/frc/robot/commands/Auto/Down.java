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

public class Down extends SequentialCommandGroup {

  public Down(RobotContainer robotContainer) {
    PathPlannerPath Down1, Down2;
    try {
      Down1 = PathPlannerPath.fromChoreoTrajectory("Down1");
      Down2 = PathPlannerPath.fromChoreoTrajectory("Down2");
      addCommands(
          new InstantCommand(
              () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
      addCommands(AutoBuilder.followPath(Down1));
      addCommands(new SmashBumpCommand(robotContainer).withTimeout(3.2));
      addCommands(new MegaTrackIterativeCommand(robotContainer, false).withTimeout(4));
      addCommands(new SmashTrenchCommand(robotContainer).withTimeout(3.2));
      addCommands(
          new InstantCommand(
              () -> robotContainer.intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
      addCommands(AutoBuilder.followPath(Down2));
      addCommands(new SmashBumpCommand(robotContainer).withTimeout(3.2));
      addCommands(new MegaTrackIterativeCommand(robotContainer, false));
    } catch (Exception e) {
      DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    }
  }

  public static Pose2d getStartPose(Alliance alliance) {
    Pose2d bluePose2d = new Pose2d(4.59, 0.51, new Rotation2d(Degrees.of(90)));
    if (alliance == Alliance.Blue) return bluePose2d;
    else return bluePose2d.rotateAround(FieldConstants.FIELD_CENTER, Rotation2d.k180deg);
  }
}
