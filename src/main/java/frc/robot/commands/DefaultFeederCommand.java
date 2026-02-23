// Copyright 2026
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.feeder.Feeder;

/** Default command for feeder: slowly reverse when idle. */
public class DefaultFeederCommand extends Command {
  private final Feeder feeder;

  public DefaultFeederCommand(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
  }

  @Override
  public void execute() {
    feeder.setVelocity(Constants.FeederConstants.DEFAULT_REVERSE_RPS);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    feeder.stop();
  }
}
