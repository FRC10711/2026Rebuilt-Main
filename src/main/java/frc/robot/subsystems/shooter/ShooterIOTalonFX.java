// Copyright 2025
package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterConstants;

/** TalonFX implementation for dual shooter flywheels. */
public class ShooterIOTalonFX implements ShooterIO {
  // Shooter 1
  private final TalonFX flywheel1Leader;
  private final TalonFX flywheel1Follower;
  private final VelocityTorqueCurrentFOC flywheel1VelocityReq = new VelocityTorqueCurrentFOC(0.0);
  private final Follower flywheel1FollowerReq;

  // Shooter 2
  private final TalonFX flywheel2Leader;
  private final TalonFX flywheel2Follower;
  private final VelocityTorqueCurrentFOC flywheel2VelocityReq = new VelocityTorqueCurrentFOC(0.0);
  private final Follower flywheel2FollowerReq;

  // Shooter 1 signals
  private final StatusSignal<AngularVelocity> leader1Velocity;
  private final StatusSignal<AngularVelocity> follower1Velocity;
  private final StatusSignal<Voltage> leader1AppliedVolts;
  private final StatusSignal<Voltage> follower1AppliedVolts;
  private final StatusSignal<Current> leader1Current;
  private final StatusSignal<Current> follower1Current;

  // Shooter 2 signals
  private final StatusSignal<AngularVelocity> leader2Velocity;
  private final StatusSignal<AngularVelocity> follower2Velocity;
  private final StatusSignal<Voltage> leader2AppliedVolts;
  private final StatusSignal<Voltage> follower2AppliedVolts;
  private final StatusSignal<Current> leader2Current;
  private final StatusSignal<Current> follower2Current;

  public ShooterIOTalonFX() {
    flywheel1Leader = new TalonFX(ShooterConstants.FLYWHEEL_1_LEADER_ID);
    flywheel1Follower = new TalonFX(ShooterConstants.FLYWHEEL_1_FOLLOWER_ID);
    flywheel2Leader = new TalonFX(ShooterConstants.FLYWHEEL_2_LEADER_ID);
    flywheel2Follower = new TalonFX(ShooterConstants.FLYWHEEL_2_FOLLOWER_ID);

    var flywheelCfg = new TalonFXConfiguration();
    flywheelCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    flywheelCfg.MotorOutput.Inverted =
        ShooterConstants.FLYWHEEL_LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    flywheelCfg.TorqueCurrent.PeakReverseTorqueCurrent = 0;
    flywheelCfg.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    flywheelCfg.Slot0 =
        new Slot0Configs()
            .withKP(ShooterConstants.FLYWHEEL_KP)
            .withKI(ShooterConstants.FLYWHEEL_KI)
            .withKD(ShooterConstants.FLYWHEEL_KD)
            .withKV(ShooterConstants.FLYWHEEL_KV)
            .withKS(ShooterConstants.FLYWHEEL_KS);

    // Configure shooter 1
    flywheel1Leader.getConfigurator().apply(flywheelCfg);
    var followerCfg = new TalonFXConfiguration();
    followerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    followerCfg.Feedback.SensorToMechanismRatio = ShooterConstants.FLYWHEEL_SENSOR_TO_MECH_RATIO;
    followerCfg.Slot0 = flywheelCfg.Slot0;
    flywheel1Follower.getConfigurator().apply(followerCfg);
    flywheel1FollowerReq =
        new Follower(flywheel1Leader.getDeviceID(), ShooterConstants.FLYWHEEL_1_FOLLOWER_INVERTED);
    flywheel1Follower.setControl(flywheel1FollowerReq);

    // Configure shooter 2 (same config)
    flywheel2Leader.getConfigurator().apply(flywheelCfg);
    flywheel2Follower.getConfigurator().apply(followerCfg);
    flywheel2FollowerReq =
        new Follower(flywheel2Leader.getDeviceID(), ShooterConstants.FLYWHEEL_2_FOLLOWER_INVERTED);
    flywheel2Follower.setControl(flywheel2FollowerReq);

    // Acquire status signals
    leader1Velocity = flywheel1Leader.getVelocity();
    follower1Velocity = flywheel1Follower.getVelocity();
    leader1AppliedVolts = flywheel1Leader.getMotorVoltage();
    follower1AppliedVolts = flywheel1Follower.getMotorVoltage();
    leader1Current = flywheel1Leader.getStatorCurrent();
    follower1Current = flywheel1Follower.getStatorCurrent();

    leader2Velocity = flywheel2Leader.getVelocity();
    follower2Velocity = flywheel2Follower.getVelocity();
    leader2AppliedVolts = flywheel2Leader.getMotorVoltage();
    follower2AppliedVolts = flywheel2Follower.getMotorVoltage();
    leader2Current = flywheel2Leader.getStatorCurrent();
    follower2Current = flywheel2Follower.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leader1Velocity,
        follower1Velocity,
        leader1AppliedVolts,
        follower1AppliedVolts,
        leader1Current,
        follower1Current,
        leader2Velocity,
        follower2Velocity,
        leader2AppliedVolts,
        follower2AppliedVolts,
        leader2Current,
        follower2Current);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var status1 =
        BaseStatusSignal.refreshAll(
            leader1Velocity,
            follower1Velocity,
            leader1AppliedVolts,
            follower1AppliedVolts,
            leader1Current,
            follower1Current);
    var status2 =
        BaseStatusSignal.refreshAll(
            leader2Velocity,
            follower2Velocity,
            leader2AppliedVolts,
            follower2AppliedVolts,
            leader2Current,
            follower2Current);

    inputs.flywheel1LeaderVelocityRotationPerSec = leader1Velocity.getValueAsDouble();
    inputs.flywheel1FollowerVelocityRotationPerSec = follower1Velocity.getValueAsDouble();
    inputs.flywheel1LeaderAppliedVolts = leader1AppliedVolts.getValueAsDouble();
    inputs.flywheel1FollowerAppliedVolts = follower1AppliedVolts.getValueAsDouble();
    inputs.flywheel1LeaderCurrentAmps = leader1Current.getValueAsDouble();
    inputs.flywheel1FollowerCurrentAmps = follower1Current.getValueAsDouble();

    inputs.flywheel2LeaderVelocityRotationPerSec = leader2Velocity.getValueAsDouble();
    inputs.flywheel2FollowerVelocityRotationPerSec = follower2Velocity.getValueAsDouble();
    inputs.flywheel2LeaderAppliedVolts = leader2AppliedVolts.getValueAsDouble();
    inputs.flywheel2FollowerAppliedVolts = follower2AppliedVolts.getValueAsDouble();
    inputs.flywheel2LeaderCurrentAmps = leader2Current.getValueAsDouble();
    inputs.flywheel2FollowerCurrentAmps = follower2Current.getValueAsDouble();

    inputs.connected = status1.isOK() && status2.isOK();
  }

  @Override
  public void setFlywheelVelocity(double rps) {
    flywheel1Leader.setControl(flywheel1VelocityReq.withVelocity(rps));
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Leader.setControl(flywheel2VelocityReq.withVelocity(rps));
    flywheel2Follower.setControl(flywheel2FollowerReq);
  }

  @Override
  public void setFlywheelVelocity(double rps, double accelRpsPerSec) {
    flywheel1Leader.setControl(
        flywheel1VelocityReq.withVelocity(rps).withAcceleration(accelRpsPerSec));
    flywheel1Follower.setControl(flywheel1FollowerReq);
    flywheel2Leader.setControl(
        flywheel2VelocityReq.withVelocity(rps).withAcceleration(accelRpsPerSec));
    flywheel2Follower.setControl(flywheel2FollowerReq);
  }

  @Override
  public void stop() {
    flywheel1Leader.stopMotor();
    flywheel1Follower.stopMotor();
    flywheel2Leader.stopMotor();
    flywheel2Follower.stopMotor();
  }
}
