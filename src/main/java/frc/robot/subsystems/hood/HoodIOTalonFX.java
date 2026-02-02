// Copyright 2025
package frc.robot.subsystems.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.HoodConstants;

/** TalonFX implementation of HoodIO. Shared by both shooters. */
public class HoodIOTalonFX implements HoodIO {
  private final TalonFX hoodMotor;
  private final MotionMagicTorqueCurrentFOC hoodMotionMagicReq =
      new MotionMagicTorqueCurrentFOC(0.0);

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodCurrent;

  public HoodIOTalonFX() {
    hoodMotor = new TalonFX(HoodConstants.MOTOR_ID);

    var hoodCfg = new TalonFXConfiguration();
    hoodCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    hoodCfg.MotorOutput.Inverted =
        HoodConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    hoodCfg.Feedback.SensorToMechanismRatio = HoodConstants.SENSOR_TO_MECH_RATIO;
    hoodCfg.Slot0 =
        new Slot0Configs()
            .withKP(HoodConstants.KP)
            .withKI(HoodConstants.KI)
            .withKD(HoodConstants.KD)
            .withKS(HoodConstants.KS)
            .withKV(HoodConstants.KV)
            .withKA(HoodConstants.KA)
            .withKG(HoodConstants.KG);
    hoodCfg.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(HoodConstants.MM_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(HoodConstants.MM_ACCELERATION)
            .withMotionMagicJerk(HoodConstants.MM_JERK);
    hoodMotor.getConfigurator().apply(hoodCfg);
    hoodMotor.setPosition(0);

    hoodPosition = hoodMotor.getPosition();
    hoodVelocity = hoodMotor.getVelocity();
    hoodAppliedVolts = hoodMotor.getMotorVoltage();
    hoodCurrent = hoodMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(hoodPosition, hoodVelocity, hoodAppliedVolts, hoodCurrent);
    inputs.hoodAngleDeg = hoodPosition.getValueAsDouble() * 360.0;
    inputs.hoodVelocityDegPerSec = hoodVelocity.getValueAsDouble() * 360.0;
    inputs.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    inputs.hoodCurrentAmps = hoodCurrent.getValueAsDouble();
    inputs.connected = status.isOK();
  }

  @Override
  public void setHoodAngleDeg(double deg) {
    double rotations = deg / 360.0;
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void setHoodAngleDeg(double deg, double velDegPerSec) {
    double rotations = deg / 360.0;
    hoodMotor.setControl(hoodMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void stop() {
    hoodMotor.stopMotor();
  }
}
