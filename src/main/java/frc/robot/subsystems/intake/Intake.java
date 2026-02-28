package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
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
    /** 射球时：根据射球数量“越收越回” */
    SHOT_LINKED_STOW,
    UP_DEBUG,
    /** 间歇性收放，用来把球往后拨 */
    FLICK_BACK
  }

  private WantedState wantedState = WantedState.UP_STOW_STOP;

  private double rollerVoltsSetpoint = 0.0;
  private double deployPosRotSetpoint = 0.0;

  // Shot-linked stow
  private int shotCount = 0;

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

  /** Provide total shots fired to drive {@link WantedState#SHOT_LINKED_STOW}. */
  public void setShotCount(int shotCount) {
    this.shotCount = Math.max(0, shotCount);
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
        double baseUp = Constants.IntakeConstants.DEPLOY_POS_UP_ROT;
        deployPosRotSetpoint = baseUp;
        rollerVoltsSetpoint = Constants.IntakeConstants.ROLLER_STOP_VOLTS;
      }
      case UP_DEBUG -> {
        double baseUp = Constants.IntakeConstants.DEPLOY_POS_DEBUG_ROT;
        deployPosRotSetpoint = baseUp;
        rollerVoltsSetpoint = Constants.IntakeConstants.ROLLER_STOP_VOLTS;
      }
      case SHOT_LINKED_STOW -> {
        double baseUp = Constants.IntakeConstants.DEPLOY_POS_UP_ROT;
        double extra =
            MathUtil.clamp(
                shotCount * Constants.IntakeConstants.SHOOT_STOW_EXTRA_PER_SHOT_ROT,
                Constants.IntakeConstants.SHOOT_STOW_EXTRA_MIN_ROT,
                Constants.IntakeConstants.SHOOT_STOW_EXTRA_MAX_ROT);
        deployPosRotSetpoint = baseUp + extra;
        rollerVoltsSetpoint = Constants.IntakeConstants.ROLLER_STOP_VOLTS;

        Logger.recordOutput("Intake/ShotLinkedStow/Shots", shotCount);
        Logger.recordOutput("Intake/ShotLinkedStow/ExtraRot", extra);
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
