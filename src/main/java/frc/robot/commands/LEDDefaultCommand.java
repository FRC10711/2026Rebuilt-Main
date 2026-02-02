// Copyright 2026
package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

/**
 * Default LED command.
 *
 * <p>TODO: fill in full state machine + blink patterns. For now this is a minimal placeholder that
 * demonstrates wiring.
 */
public class LEDDefaultCommand extends Command {
  private final RobotContainer robot;

  public LEDDefaultCommand(RobotContainer robot) {
    this.robot = robot;
    addRequirements(robot.led);
  }

  @Override
  public void execute() {
    // TODO: replace with full machine-state logic (enabled/disabled, errors, shooting ready, etc.)
    if (DriverStation.isDisabled()) {
      robot.led.off();
      return;
    }

    if (robot.hopper.isFull()) {
      robot.led.setSolid(0, 255, 0); // green
    } else {
      robot.led.setBlink(255, 80, 0, 0.15, 0.15); // orange blink
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
