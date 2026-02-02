package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.RobotContainer;

public class ShootCommand extends Command {
  RobotContainer m_RobotContainer;
  Translation2d m_targetTranslation2d;
  Timer m_Timer = new Timer();

  public ShootCommand(RobotContainer _RobotContainer, Translation2d targetTranslation) {
    m_RobotContainer = _RobotContainer;

    m_targetTranslation2d = targetTranslation;
  }

  @Override
  public void initialize() {
    m_RobotContainer.shooter.setVelocity(30);
    m_Timer.reset();
    m_Timer.start();
  }

  Translation2d getFixedTarget(
      Translation2d robotTranslation2d,
      Translation2d targetTranslation2d,
      ChassisSpeeds fieldRelativeRobotSpeeds) {
    Translation2d m_robotTelativeTargetTranslation = targetTranslation2d.minus(robotTranslation2d);
    Translation2d m_velocityContributionTranslation =
        new Translation2d(
                fieldRelativeRobotSpeeds.vxMetersPerSecond,
                fieldRelativeRobotSpeeds.vyMetersPerSecond)
            .times(AutoShootConstants.FlyTime);
    Translation2d fixedTranslation2d =
        m_robotTelativeTargetTranslation.minus(m_velocityContributionTranslation);
    return fixedTranslation2d;
  }

  @Override
  public boolean isFinished() {
    double a = m_Timer.get();
    if (a > 1.) {
      return true;
    } else return false;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    m_RobotContainer.shooter.setVelocity(0);
  }
}
