// Copyright 2026
package frc.robot.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Hopper subsystem (detection only).
 *
 * <p>Uses two CANrange sensors and a Debouncer to determine whether the hopper is "full".
 */
public class Hopper extends SubsystemBase {
  private final HopperIO io;
  private final HopperIO.HopperIOInputs inputs = new HopperIO.HopperIOInputs();

  private final Debouncer fullDebouncer =
      new Debouncer(HopperConstants.FULL_DEBOUNCE_SEC, Debouncer.DebounceType.kBoth);

  private boolean full = false;

  public Hopper(HopperIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    boolean fullRaw = inputs.detected1 && inputs.detected2;
    full = fullDebouncer.calculate(fullRaw);

    Logger.recordOutput("Hopper/FullRaw", fullRaw);
    Logger.recordOutput("Hopper/Full", full);
  }

  /** True if hopper is full (debounced). */
  public boolean isFull() {
    return full;
  }

  /** True if the CANrange sensors are healthy. */
  public boolean isConnected() {
    return inputs.connected;
  }
}
