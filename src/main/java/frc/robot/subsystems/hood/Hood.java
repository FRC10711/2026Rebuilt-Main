// Copyright 2025
package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem for the shared hood. Both shooters use this single hood. */
public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIO.HoodIOInputs inputs = new HoodIO.HoodIOInputs();

  private double hoodSetpointDeg = 0.0;
  private double hoodVelSetpointDegPerSec = 0.0;

  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/SetpointDeg", hoodSetpointDeg);
    Logger.recordOutput("Hood/VelSetpointDegPerSec", hoodVelSetpointDegPerSec);
  }

  /** Sets hood/backplate angle (degrees). */
  public void setAngle(double deg) {
    hoodSetpointDeg = deg;
    hoodVelSetpointDegPerSec = 0.0;
    io.setHoodAngleDeg(deg);
  }

  /** Sets hood/backplate angle with velocity feedforward (deg and deg/s). */
  public void setAngle(double deg, double velDegPerSec) {
    hoodSetpointDeg = deg;
    hoodVelSetpointDegPerSec = velDegPerSec;
    io.setHoodAngleDeg(deg, velDegPerSec);
  }

  public void stop() {
    hoodVelSetpointDegPerSec = 0.0;
    io.stop();
  }

  /** Returns hood/backplate angle (mechanism degrees). */
  public double getAngleDeg() {
    return inputs.hoodAngleDeg;
  }

  /** Returns whether hood IO is connected/healthy. */
  public boolean isConnected() {
    return inputs.connected;
  }
}
