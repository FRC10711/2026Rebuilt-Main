// Copyright 2025
package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** Subsystem for dual shooter flywheels. Hood is in {@link frc.robot.subsystems.hood.Hood}. */
public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private double flywheelSetpointRPS = 0.0;
  private double flywheelAccelSetpointRpsPerSec = 0.0;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    Logger.recordOutput("Shooter/FlywheelSetpointRPS", flywheelSetpointRPS);
    Logger.recordOutput("Shooter/FlywheelAccelSetpointRpsPerSec", flywheelAccelSetpointRpsPerSec);
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
