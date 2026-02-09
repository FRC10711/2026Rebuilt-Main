// Copyright 2025
package frc.robot.subsystems.feeder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.FeederConstants;

/** TalonFX implementation for dual feeders. */
public class FeederIOTalonFX implements FeederIO {
  // Feeder 1
  private final TalonFX motor1;
  private final TalonFX followerMotor1;
  private final VelocityTorqueCurrentFOC velocityReq1 = new VelocityTorqueCurrentFOC(0.0);
  private final Follower followerReq1;

  // Feeder 2
  private final TalonFX motor2;
  private final TalonFX followerMotor2;
  private final VelocityTorqueCurrentFOC velocityReq2 = new VelocityTorqueCurrentFOC(0.0);
  private final Follower followerReq2;

  private final StatusSignal<AngularVelocity> velocity1;
  private final StatusSignal<Voltage> appliedVolts1;
  private final StatusSignal<Current> current1;

  private final StatusSignal<AngularVelocity> velocity2;
  private final StatusSignal<Voltage> appliedVolts2;
  private final StatusSignal<Current> current2;

  public FeederIOTalonFX() {
    motor1 = new TalonFX(FeederConstants.MOTOR_1_ID);
    followerMotor1 = new TalonFX(FeederConstants.FOLLOWER_1_ID);
    motor2 = new TalonFX(FeederConstants.MOTOR_2_ID);
    followerMotor2 = new TalonFX(FeederConstants.FOLLOWER_2_ID);

    var cfg = new TalonFXConfiguration();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    cfg.MotorOutput.Inverted =
        FeederConstants.INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    cfg.Feedback.SensorToMechanismRatio = FeederConstants.SENSOR_TO_MECH_RATIO;
    cfg.Slot0 =
        new Slot0Configs()
            .withKP(FeederConstants.KP)
            .withKI(FeederConstants.KI)
            .withKD(FeederConstants.KD)
            .withKV(FeederConstants.KV)
            .withKS(FeederConstants.KS);
    cfg.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    cfg.CurrentLimits.SupplyCurrentLimitEnable = FeederConstants.ENABLE_SUPPLY_CURRENT_LIMIT;
    cfg.CurrentLimits.SupplyCurrentLimit = FeederConstants.SUPPLY_CURRENT_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLowerLimit = FeederConstants.SUPPLY_CURRENT_LOWER_LIMIT_AMPS;
    cfg.CurrentLimits.SupplyCurrentLowerTime = FeederConstants.SUPPLY_CURRENT_LOWER_TIME_SEC;
    cfg.CurrentLimits.StatorCurrentLimitEnable = FeederConstants.ENABLE_STATOR_CURRENT_LIMIT;
    cfg.CurrentLimits.StatorCurrentLimit = FeederConstants.STATOR_CURRENT_LIMIT_AMPS;
    motor1.getConfigurator().apply(cfg);
    motor2.getConfigurator().apply(cfg);

    var followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg.Feedback.SensorToMechanismRatio = FeederConstants.SENSOR_TO_MECH_RATIO;
    followerCfg.Slot0 = cfg.Slot0;
    followerCfg.CurrentLimits = cfg.CurrentLimits;
    followerMotor1.getConfigurator().apply(followerCfg);
    followerMotor2.getConfigurator().apply(followerCfg);

    followerReq1 = new Follower(motor1.getDeviceID(), FeederConstants.FOLLOWER_1_ALIGNMENT);
    followerMotor1.setControl(followerReq1);
    followerReq2 = new Follower(motor2.getDeviceID(), FeederConstants.FOLLOWER_2_ALIGNMENT);
    followerMotor2.setControl(followerReq2);

    velocity1 = motor1.getVelocity();
    appliedVolts1 = motor1.getMotorVoltage();
    current1 = motor1.getStatorCurrent();

    velocity2 = motor2.getVelocity();
    appliedVolts2 = motor2.getMotorVoltage();
    current2 = motor2.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity1, appliedVolts1, current1, velocity2, appliedVolts2, current2);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var status1 = BaseStatusSignal.refreshAll(velocity1, appliedVolts1, current1);
    var status2 = BaseStatusSignal.refreshAll(velocity2, appliedVolts2, current2);

    inputs.velocity1RadPerSec = Units.rotationsToRadians(velocity1.getValueAsDouble());
    inputs.appliedVolts1 = appliedVolts1.getValueAsDouble();
    inputs.current1Amps = current1.getValueAsDouble();

    inputs.velocity2RadPerSec = Units.rotationsToRadians(velocity2.getValueAsDouble());
    inputs.appliedVolts2 = appliedVolts2.getValueAsDouble();
    inputs.current2Amps = current2.getValueAsDouble();

    inputs.connected = status1.isOK() && status2.isOK();
  }

  @Override
  public void setVelocity(double rps) {
    motor1.setControl(velocityReq1.withVelocity(rps));
    followerMotor1.setControl(followerReq1);
    motor2.setControl(velocityReq2.withVelocity(rps));
    followerMotor2.setControl(followerReq2);
  }

  @Override
  public void stop() {
    motor1.stopMotor();
    followerMotor1.stopMotor();
    motor2.stopMotor();
    followerMotor2.stopMotor();
  }
}
