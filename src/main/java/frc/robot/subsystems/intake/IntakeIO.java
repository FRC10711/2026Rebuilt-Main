// Copyright 2025
package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = false;

    // Roller motors (mechanism units after SensorToMechanismRatio config)
    public double leaderVelocityRadPerSec = 0.0;
    public double followerVelocityRadPerSec = 0.0;
    public double leaderAppliedVolts = 0.0;
    public double followerAppliedVolts = 0.0;
    public double leaderCurrentAmps = 0.0;
    public double followerCurrentAmps = 0.0;

    // Deploy/arm motor (mechanism rotations after SensorToMechanismRatio config)
    public double deployPositionRot = 0.0;
    public double deployVelocityRotPerSec = 0.0;
    public double deployAppliedVolts = 0.0;
    public double deployCurrentAmps = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets roller voltage output (volts). */
  public default void setRollerVoltage(double volts) {}

  /** Set deploy/arm position target (mechanism rotations). */
  public default void setDeployPositionRot(double rotations) {}

  public default void stop() {}
}
