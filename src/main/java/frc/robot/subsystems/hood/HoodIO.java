// Copyright 2025
package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** IO interface for the shared hood (angle adjustment). */
public interface HoodIO {
  class HoodIOInputs implements LoggableInputs {
    public boolean connected = false;
    public double hoodAngleDeg = 0.0;
    public double hoodVelocityDegPerSec = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodCurrentAmps = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("HoodAngleDeg", hoodAngleDeg);
      table.put("HoodVelocityDegPerSec", hoodVelocityDegPerSec);
      table.put("HoodAppliedVolts", hoodAppliedVolts);
      table.put("HoodCurrentAmps", hoodCurrentAmps);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      hoodAngleDeg = table.get("HoodAngleDeg", hoodAngleDeg);
      hoodVelocityDegPerSec = table.get("HoodVelocityDegPerSec", hoodVelocityDegPerSec);
      hoodAppliedVolts = table.get("HoodAppliedVolts", hoodAppliedVolts);
      hoodCurrentAmps = table.get("HoodCurrentAmps", hoodCurrentAmps);
    }
  }

  default void updateInputs(HoodIOInputs inputs) {}

  /** Sets hood/backplate angle in degrees (mechanism degrees). */
  default void setHoodAngleDeg(double deg) {}

  /** Sets hood/backplate angle with velocity feedforward (mechanism degrees and deg/s). */
  default void setHoodAngleDeg(double deg, double velDegPerSec) {
    setHoodAngleDeg(deg);
  }

  default void stop() {}
}
