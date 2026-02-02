// Copyright 2025
package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** IO interface for dual feeders. */
public interface FeederIO {
  class FeederIOInputs implements LoggableInputs {
    public boolean connected = false;

    // Feeder 1 (mechanism units after SensorToMechanismRatio)
    public double velocity1RadPerSec = 0.0;
    public double appliedVolts1 = 0.0;
    public double current1Amps = 0.0;

    // Feeder 2
    public double velocity2RadPerSec = 0.0;
    public double appliedVolts2 = 0.0;
    public double current2Amps = 0.0;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("Velocity1RadPerSec", velocity1RadPerSec);
      table.put("AppliedVolts1", appliedVolts1);
      table.put("Current1Amps", current1Amps);
      table.put("Velocity2RadPerSec", velocity2RadPerSec);
      table.put("AppliedVolts2", appliedVolts2);
      table.put("Current2Amps", current2Amps);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      velocity1RadPerSec = table.get("Velocity1RadPerSec", velocity1RadPerSec);
      appliedVolts1 = table.get("AppliedVolts1", appliedVolts1);
      current1Amps = table.get("Current1Amps", current1Amps);
      velocity2RadPerSec = table.get("Velocity2RadPerSec", velocity2RadPerSec);
      appliedVolts2 = table.get("AppliedVolts2", appliedVolts2);
      current2Amps = table.get("Current2Amps", current2Amps);
    }
  }

  default void updateInputs(FeederIOInputs inputs) {}

  /** Sets both feeders to the same velocity (mechanism RPS). */
  default void setVelocity(double rps) {}

  default void stop() {}
}
