// Copyright 2026
package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** IO interface for hopper sensing (no motors, detection only). */
public interface HopperIO {
  class HopperIOInputs implements LoggableInputs {
    public boolean connected = false;

    /** CANrange 1 distance in meters (best-effort, 0 if unknown). */
    public double distance1Meters = 0.0;
    /** CANrange 2 distance in meters (best-effort, 0 if unknown). */
    public double distance2Meters = 0.0;

    /** Raw detection booleans computed from distance thresholds. */
    public boolean detected1 = false;

    public boolean detected2 = false;

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("Distance1Meters", distance1Meters);
      table.put("Distance2Meters", distance2Meters);
      table.put("Detected1", detected1);
      table.put("Detected2", detected2);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      distance1Meters = table.get("Distance1Meters", distance1Meters);
      distance2Meters = table.get("Distance2Meters", distance2Meters);
      detected1 = table.get("Detected1", detected1);
      detected2 = table.get("Detected2", detected2);
    }
  }

  /** Refresh sensor readings and fill inputs. */
  default void updateInputs(HopperIOInputs inputs) {}
}
