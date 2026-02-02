package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  private final IndexerIO io;
  private final IndexerIO.IndexerIOInputs inputs = new IndexerIO.IndexerIOInputs();

  private double voltsSetpoint = 0.0;

  public Indexer(IndexerIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Indexer", inputs);
    Logger.recordOutput("Indexer/VoltsSetpoint", voltsSetpoint);
  }

  public void setVoltage(double volts) {
    voltsSetpoint = volts;
    io.setVoltage(volts);
  }

  public void stop() {
    voltsSetpoint = 0.0;
    io.stop();
  }
}
