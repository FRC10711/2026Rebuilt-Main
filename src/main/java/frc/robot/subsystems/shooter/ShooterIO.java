// Copyright 2025
package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

/** IO interface for dual-shooter flywheels (hood is in Hood subsystem). */
public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public boolean connected = false;

    // Shooter 1 (mechanism units after SensorToMechanismRatio)
    public double flywheel1LeaderVelocityRotationPerSec = 0.0;
    public double flywheel1FollowerVelocityRotationPerSec = 0.0;
    public double flywheel1LeaderAppliedVolts = 0.0;
    public double flywheel1FollowerAppliedVolts = 0.0;
    public double flywheel1LeaderCurrentAmps = 0.0;
    public double flywheel1FollowerCurrentAmps = 0.0;

    // Shooter 2
    public double flywheel2LeaderVelocityRotationPerSec = 0.0;
    public double flywheel2FollowerVelocityRotationPerSec = 0.0;
    public double flywheel2LeaderAppliedVolts = 0.0;
    public double flywheel2FollowerAppliedVolts = 0.0;
    public double flywheel2LeaderCurrentAmps = 0.0;
    public double flywheel2FollowerCurrentAmps = 0.0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets both flywheels to the same velocity (RPS). */
  default void setFlywheelVelocity(double rps) {}

  /** Sets both flywheels with acceleration feedforward (RPS and RPS/s). */
  default void setFlywheelVelocity(double rps, double accelRpsPerSec) {
    setFlywheelVelocity(rps);
  }

  default void stop() {}
}
