package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public enum WantedState {
    /** 向下放 + 吸球 */
    DOWN_INTAKE,
    /** 向上收回 + 停止吸球 */
    UP_STOW_STOP,
    /** 间歇性收放，用来把球往后拨 */
    FLICK_BACK
  }

  private WantedState wantedState = WantedState.UP_STOW_STOP;

  private double rollerVoltsSetpoint = 0.0;
  private double deployPosRotSetpoint = 0.0;

  // Flick timing
  private boolean flickDownPhase = false;
  private double nextFlickToggleTs = 0.0;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    applyWantedState();

    Logger.recordOutput("Intake/WantedState", wantedState.toString());
    Logger.recordOutput("Intake/RollerVoltsSetpoint", rollerVoltsSetpoint);
    Logger.recordOutput("Intake/DeployPosRotSetpoint", deployPosRotSetpoint);
  }

  public void setWantedState(WantedState state) {
    if (state != wantedState) {
      wantedState = state;
      // Reset flick when entering flick mode
      if (wantedState == WantedState.FLICK_BACK) {
        flickDownPhase = false;
        nextFlickToggleTs = Timer.getFPGATimestamp();
      }
    }
  }

  /** Backwards-compatible helper: directly set roller voltage and keep current deploy state. */
  public void setVoltage(double volts) {
    rollerVoltsSetpoint = volts;
    io.setRollerVoltage(volts);
  }

  public WantedState getWantedState() {
    return wantedState;
  }

  private void applyWantedState() {
    switch (wantedState) {
      case DOWN_INTAKE -> {
        deployPosRotSetpoint = Constants.IntakeConstants.DEPLOY_POS_DOWN_ROT;
        rollerVoltsSetpoint = Constants.IntakeConstants.ROLLER_INTAKE_VOLTS;
      }
      case UP_STOW_STOP -> {
        deployPosRotSetpoint = Constants.IntakeConstants.DEPLOY_POS_UP_ROT;
        rollerVoltsSetpoint = Constants.IntakeConstants.ROLLER_STOP_VOLTS;
      }
      case FLICK_BACK -> {
        double now = Timer.getFPGATimestamp();
        if (now >= nextFlickToggleTs) {
          flickDownPhase = !flickDownPhase;
          nextFlickToggleTs =
              now
                  + (flickDownPhase
                      ? Constants.IntakeConstants.FLICK_ON_SEC
                      : Constants.IntakeConstants.FLICK_OFF_SEC);
        }
        deployPosRotSetpoint =
            flickDownPhase
                ? Constants.IntakeConstants.FLIP_POS_UP
                : Constants.IntakeConstants.DEPLOY_POS_UP_ROT;
        rollerVoltsSetpoint = Constants.IntakeConstants.FLICK_ROLLER_VOLTS;
      }
    }

    io.setDeployPositionRot(deployPosRotSetpoint);
    io.setRollerVoltage(rollerVoltsSetpoint);
  }

  public void stop() {
    setWantedState(WantedState.UP_STOW_STOP);
  }
}
