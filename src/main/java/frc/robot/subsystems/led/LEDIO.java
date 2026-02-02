// Copyright 2026
package frc.robot.subsystems.led;

/** IO interface for LED control (CANdle). */
public interface LEDIO {
  /** Set LEDs to a solid RGB color (0-255). */
  default void setRGB(int r, int g, int b) {}

  /** Turn LEDs off. */
  default void off() {
    setRGB(0, 0, 0);
  }
}
