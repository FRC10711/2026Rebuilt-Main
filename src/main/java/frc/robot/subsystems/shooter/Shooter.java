// Copyright 2025
package frc.robot.subsystems.shooter;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Subsystem for dual shooter flywheels. Hood is in {@link frc.robot.subsystems.hood.Hood}. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double flywheelSetpointRPS = 0.0;
  private double flywheelAccelSetpointRpsPerSec = 0.0;

  // Shot counting (per shooter)
  private final Debouncer shot1Debouncer =
      new Debouncer(
          Constants.ShooterConstants.SHOT_COUNT_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);
  private final Debouncer shot2Debouncer =
      new Debouncer(
          Constants.ShooterConstants.SHOT_COUNT_DEBOUNCE_SEC, Debouncer.DebounceType.kRising);

  private boolean lastShot1Debounced = false;
  private boolean lastShot2Debounced = false;
  private int shots1 = 0;
  private int shots2 = 0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/FlywheelSetpointRPS", flywheelSetpointRPS);
    Logger.recordOutput("Shooter/FlywheelAccelSetpointRpsPerSec", flywheelAccelSetpointRpsPerSec);

    updateShotCounting();
    Logger.recordOutput("Shooter/Shots1", shots1);
    Logger.recordOutput("Shooter/Shots2", shots2);
  }

  private void updateShotCounting() {
    // Arm only when commanded to spin (avoid counting while idle)
    boolean armed = flywheelSetpointRPS >= Constants.ShooterConstants.SHOT_COUNT_MIN_SETPOINT_RPS;

    if (!armed) {
      // Reset debouncers/edges while not armed
      shot1Debouncer.calculate(false);
      shot2Debouncer.calculate(false);
      lastShot1Debounced = false;
      lastShot2Debounced = false;
      return;
    }

    double cur1 = Math.max(inputs.flywheel1LeaderCurrentAmps, inputs.flywheel1FollowerCurrentAmps);
    double cur2 = Math.max(inputs.flywheel2LeaderCurrentAmps, inputs.flywheel2FollowerCurrentAmps);

    boolean low1Raw = cur1 >= Constants.ShooterConstants.SHOT_COUNT_LOW_CURRENT_THRESHOLD_AMPS;
    boolean low2Raw = cur2 >= Constants.ShooterConstants.SHOT_COUNT_LOW_CURRENT_THRESHOLD_AMPS;

    boolean low1Debounced = shot1Debouncer.calculate(low1Raw);
    boolean low2Debounced = shot2Debouncer.calculate(low2Raw);

    // Rising edge: false -> true means a "shot" event
    if (!lastShot1Debounced && low1Debounced) {
      shots1++;
    }
    if (!lastShot2Debounced && low2Debounced) {
      shots2++;
    }

    lastShot1Debounced = low1Debounced;
    lastShot2Debounced = low2Debounced;

    Logger.recordOutput("Shooter/ShotCount/Armed", armed);
    Logger.recordOutput("Shooter/ShotCount/Cur1MaxA", cur1);
    Logger.recordOutput("Shooter/ShotCount/Cur2MaxA", cur2);
    Logger.recordOutput("Shooter/ShotCount/Low1Raw", low1Raw);
    Logger.recordOutput("Shooter/ShotCount/Low2Raw", low2Raw);
    Logger.recordOutput("Shooter/ShotCount/Low1Debounced", low1Debounced);
    Logger.recordOutput("Shooter/ShotCount/Low2Debounced", low2Debounced);
  }

  /** Sets both flywheels to the same velocity (RPS). */
  public void setVelocity(double rps) {
    flywheelSetpointRPS = rps;
    flywheelAccelSetpointRpsPerSec = 0.0;
    io.setFlywheelVelocity(rps);
  }

  /** Sets both flywheels with acceleration feedforward (RPS and RPS/s). */
  public void setVelocity(double rps, double accelRpsPerSec) {
    flywheelSetpointRPS = rps;
    flywheelAccelSetpointRpsPerSec = accelRpsPerSec;
    io.setFlywheelVelocity(rps, accelRpsPerSec);
  }

  public void stop() {
    flywheelSetpointRPS = 0.0;
    flywheelAccelSetpointRpsPerSec = 0.0;
    io.stop();
  }

  /** Returns shots counted for shooter 1. */
  public int getShots1() {
    return shots1;
  }

  /** Returns shots counted for shooter 2. */
  public int getShots2() {
    return shots2;
  }

  /** Reset both shot counters to zero. */
  public void resetShotCounts() {
    shots1 = 0;
    shots2 = 0;
    lastShot1Debounced = false;
    lastShot2Debounced = false;
    shot1Debouncer.calculate(false);
    shot2Debouncer.calculate(false);
  }

  /**
   * Returns the minimum of both flywheel velocities (for shoot-ready checks: both must be at
   * setpoint).
   */
  public double getFlywheelVelocityRps() {
    double v1 = inputs.flywheel1LeaderVelocityRotationPerSec;
    double v2 = inputs.flywheel2LeaderVelocityRotationPerSec;
    return Math.min(v1, v2);
  }

  /** Returns whether shooter IO is connected/healthy. */
  public boolean isConnected() {
    return inputs.connected;
  }
}
