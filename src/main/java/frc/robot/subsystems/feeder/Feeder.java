// Copyright 2025
package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem for dual feeders. */
public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIO.FeederIOInputs inputs = new FeederIO.FeederIOInputs();

  private double velocitySetpointRps = 0.0;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    Logger.recordOutput("Feeder/VelocitySetpointRPS", velocitySetpointRps);
  }

  /** Sets both feeders to the same velocity (RPS). */
  public void setVelocity(double rps) {
    velocitySetpointRps = rps;
    io.setVelocity(rps);
  }

  public void stop() {
    velocitySetpointRps = 0.0;
    io.stop();
  }
}
