// Copyright 2025
package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

public class IntakeIOTalonFX implements IntakeIO {
  // Roller motors
  private final TalonFX leader;
  private final TalonFX follower;

  // Deploy motor (收放)
  private final TalonFX deploy;

  // Voltage control requests
  private final VoltageOut leaderVoltageReq = new VoltageOut(0.0);
  private final VoltageOut followerVoltageReq = new VoltageOut(0.0);
  private final MotionMagicTorqueCurrentFOC deployMotionMagicReq =
      new MotionMagicTorqueCurrentFOC(0.0);

  // Status signals
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;
  private final StatusSignal<Current> followerCurrent;

  private final StatusSignal<Angle> deployPosition;
  private final StatusSignal<AngularVelocity> deployVelocity;
  private final StatusSignal<Voltage> deployAppliedVolts;
  private final StatusSignal<Current> deployCurrent;

  public IntakeIOTalonFX() {
    leader = new TalonFX(Constants.IntakeConstants.LEADER_MOTOR_ID);
    follower = new TalonFX(Constants.IntakeConstants.FOLLOWER_MOTOR_ID);
    deploy = new TalonFX(Constants.IntakeConstants.DEPLOY_MOTOR_ID);

    // ---------------- Roller config ----------------
    var rollerLeaderCfg = new TalonFXConfiguration();
    rollerLeaderCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerLeaderCfg.MotorOutput.Inverted =
        Constants.IntakeConstants.LEADER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leader.getConfigurator().apply(rollerLeaderCfg);

    var rollerFollowerCfg = new TalonFXConfiguration();
    rollerFollowerCfg.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rollerFollowerCfg.MotorOutput.Inverted =
        Constants.IntakeConstants.FOLLOWER_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    follower.getConfigurator().apply(rollerFollowerCfg);

    // ---------------- Deploy config (Motion Magic + TorqueCurrentFOC) ----------------
    var deployCfg = new TalonFXConfiguration();
    deployCfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    deployCfg.MotorOutput.Inverted =
        Constants.IntakeConstants.DEPLOY_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    deployCfg.Feedback.SensorToMechanismRatio =
        Constants.IntakeConstants.DEPLOY_SENSOR_TO_MECH_RATIO;
    deployCfg.Slot0 =
        new Slot0Configs()
            .withKP(Constants.IntakeConstants.DEPLOY_KP)
            .withKI(Constants.IntakeConstants.DEPLOY_KI)
            .withKD(Constants.IntakeConstants.DEPLOY_KD)
            .withKS(Constants.IntakeConstants.DEPLOY_KS)
            .withKV(Constants.IntakeConstants.DEPLOY_KV)
            .withKA(Constants.IntakeConstants.DEPLOY_KA)
            .withKG(Constants.IntakeConstants.DEPLOY_KG);
    deployCfg.MotionMagic =
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(Constants.IntakeConstants.DEPLOY_MM_CRUISE_VELOCITY)
            .withMotionMagicAcceleration(Constants.IntakeConstants.DEPLOY_MM_ACCELERATION)
            .withMotionMagicJerk(Constants.IntakeConstants.DEPLOY_MM_JERK);
    deploy.getConfigurator().apply(deployCfg);
    // TODO: if you have an absolute reference / homing routine, do NOT blindly zero here.
    // deploy.setPosition(0.0);

    // Acquire status signals
    leaderVelocity = leader.getVelocity();
    followerVelocity = follower.getVelocity();
    leaderAppliedVolts = leader.getMotorVoltage();
    followerAppliedVolts = follower.getMotorVoltage();
    leaderCurrent = leader.getStatorCurrent();
    followerCurrent = follower.getStatorCurrent();

    deployPosition = deploy.getPosition();
    deployVelocity = deploy.getVelocity();
    deployAppliedVolts = deploy.getMotorVoltage();
    deployCurrent = deploy.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        followerVelocity,
        leaderAppliedVolts,
        followerAppliedVolts,
        leaderCurrent,
        followerCurrent,
        deployPosition,
        deployVelocity,
        deployAppliedVolts,
        deployCurrent);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            leaderVelocity,
            followerVelocity,
            leaderAppliedVolts,
            followerAppliedVolts,
            leaderCurrent,
            followerCurrent,
            deployPosition,
            deployVelocity,
            deployAppliedVolts,
            deployCurrent);

    inputs.leaderVelocityRadPerSec = Units.rotationsToRadians(leaderVelocity.getValueAsDouble());
    inputs.followerVelocityRadPerSec =
        Units.rotationsToRadians(followerVelocity.getValueAsDouble());
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.leaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.followerCurrentAmps = followerCurrent.getValueAsDouble();

    inputs.deployPositionRot = deployPosition.getValueAsDouble();
    inputs.deployVelocityRotPerSec = deployVelocity.getValueAsDouble();
    inputs.deployAppliedVolts = deployAppliedVolts.getValueAsDouble();
    inputs.deployCurrentAmps = deployCurrent.getValueAsDouble();

    inputs.connected = status.isOK();
  }

  @Override
  public void setRollerVoltage(double volts) {
    leader.setControl(leaderVoltageReq.withOutput(volts));
    follower.setControl(followerVoltageReq.withOutput(volts));
  }

  @Override
  public void setDeployPositionRot(double rotations) {
    // Phoenix expects mechanism rotations after SensorToMechanismRatio
    deploy.setControl(deployMotionMagicReq.withPosition(rotations));
  }

  @Override
  public void stop() {
    leader.stopMotor();
    follower.stopMotor();
    deploy.stopMotor();
  }
}
