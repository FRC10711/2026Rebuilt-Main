// Copyright 2026
package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.Indexer;

/** Default command for indexer: slowly reverse when idle. */
public class DefaultIndexerCommand extends Command {
  private final Indexer indexer;

  public DefaultIndexerCommand(Indexer indexer) {
    this.indexer = indexer;
    addRequirements(indexer);
  }

  @Override
  public void execute() {
    indexer.setVoltage(Constants.IndexerConstants.DEFAULT_REVERSE_VOLTS);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    indexer.stop();
  }
}
