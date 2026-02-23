package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.AutoShootConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.Intake.WantedState;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Test shooting command: spin up shooter + move hood to setpoint, and only run feeder when right
 * trigger is pressed.
 */
public class TestShootCommand extends Command {
  private final RobotContainer robotContainer;
  private final DoubleSupplier distanceMetersSupplier;
  private final DoubleSupplier shooterRpsSupplier;
  private final DoubleSupplier hoodDegSupplier;
  private final double feederRps;
  private final double indexerVolts;
  private final double triggerThreshold;

  public TestShootCommand(
      RobotContainer robotContainer,
      DoubleSupplier distanceMetersSupplier,
      double feederRps,
      double indexerVolts,
      double triggerThreshold) {
    this.robotContainer = robotContainer;
    this.distanceMetersSupplier = distanceMetersSupplier;
    this.shooterRpsSupplier = null;
    this.hoodDegSupplier = null;
    this.feederRps = feederRps;
    this.indexerVolts = indexerVolts;
    this.triggerThreshold = triggerThreshold;
    addRequirements(
        robotContainer.shooter,
        robotContainer.hood,
        robotContainer.feeder,
        robotContainer.indexer,
        robotContainer.intake);
  }

  /** Direct setpoint mode (no interpolation). */
  public TestShootCommand(
      RobotContainer robotContainer,
      DoubleSupplier shooterRpsSupplier,
      DoubleSupplier hoodDegSupplier,
      double feederRps,
      double indexerVolts,
      double triggerThreshold) {
    this.robotContainer = robotContainer;
    this.distanceMetersSupplier = null;
    this.shooterRpsSupplier = shooterRpsSupplier;
    this.hoodDegSupplier = hoodDegSupplier;
    this.feederRps = feederRps;
    this.indexerVolts = indexerVolts;
    this.triggerThreshold = triggerThreshold;
    addRequirements(
        robotContainer.shooter,
        robotContainer.hood,
        robotContainer.feeder,
        robotContainer.indexer,
        robotContainer.intake);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double shooterRps;
    double hoodDeg;
    if (distanceMetersSupplier != null) {
      // Distance input is assumed to be from robot center; convert to distance from shooter exit.
      double distanceFromCenter = distanceMetersSupplier.getAsDouble();
      double distanceFromShooter =
          Math.max(0.0, distanceFromCenter - Constants.ShooterConstants.FLYWHEEL_OFFSET_X_METERS);

      shooterRps = AutoShootConstants.shooterSpeedMap.get(distanceFromShooter);
      hoodDeg = AutoShootConstants.hoodAngleMap.get(distanceFromShooter);

      Logger.recordOutput("TestShoot/Mode", "Interpolated");
      Logger.recordOutput("TestShoot/DistanceFromCenterMeters", distanceFromCenter);
      Logger.recordOutput("TestShoot/DistanceFromShooterMeters", distanceFromShooter);
    } else {
      shooterRps = shooterRpsSupplier.getAsDouble();
      hoodDeg = hoodDegSupplier.getAsDouble();

      Logger.recordOutput("TestShoot/Mode", "Direct");
    }

    Logger.recordOutput("TestShoot/ShooterRpsSetpoint", shooterRps);
    Logger.recordOutput("TestShoot/HoodDegSetpoint", hoodDeg);

    robotContainer.shooter.setVelocity(shooterRps);
    robotContainer.hood.setAngle(hoodDeg);

    if (robotContainer.getRightTriggerAxisSupplier().getAsDouble() > triggerThreshold) {
      // Shooting: stow intake and link stow depth to shot count
      int totalShots = robotContainer.shooter.getShots1() + robotContainer.shooter.getShots2();
      robotContainer.intake.setWantedState(WantedState.SHOT_LINKED_STOW);
      robotContainer.intake.setShotCount(totalShots);

      robotContainer.feeder.setVelocity(feederRps);
      robotContainer.indexer.setVoltage(indexerVolts);
    } else {
      robotContainer.intake.setWantedState(WantedState.UP_STOW_STOP);
      robotContainer.feeder.stop();
      robotContainer.indexer.stop();
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    robotContainer.feeder.stop();
    robotContainer.indexer.stop();
    robotContainer.intake.setWantedState(WantedState.UP_STOW_STOP);
    robotContainer.hood.stop();
    robotContainer.shooter.stop();
  }
}
