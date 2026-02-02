// Copyright 2026
package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/** LED subsystem (CANdle). Provides simple solid/blink behaviors. */
public class LED extends SubsystemBase {
  private enum Mode {
    OFF,
    SOLID,
    BLINK
  }

  private final LEDIO io;

  private Mode mode = Mode.OFF;

  private int r = 0;
  private int g = 0;
  private int b = 0;

  private double blinkOnSec = 0.2;
  private double blinkOffSec = 0.2;
  private boolean blinkLit = false;
  private double nextToggleTs = 0.0;

  public LED(LEDIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();

    switch (mode) {
      case OFF -> io.off();
      case SOLID -> io.setRGB(r, g, b);
      case BLINK -> {
        if (now >= nextToggleTs) {
          blinkLit = !blinkLit;
          nextToggleTs = now + (blinkLit ? blinkOnSec : blinkOffSec);
        }
        if (blinkLit) {
          io.setRGB(r, g, b);
        } else {
          io.off();
        }
      }
    }

    Logger.recordOutput("LED/Mode", mode.toString());
    Logger.recordOutput("LED/R", r);
    Logger.recordOutput("LED/G", g);
    Logger.recordOutput("LED/B", b);
  }

  public void off() {
    mode = Mode.OFF;
  }

  public void setSolid(int r, int g, int b) {
    this.r = clamp255(r);
    this.g = clamp255(g);
    this.b = clamp255(b);
    mode = Mode.SOLID;
  }

  public void setBlink(int r, int g, int b, double onSec, double offSec) {
    this.r = clamp255(r);
    this.g = clamp255(g);
    this.b = clamp255(b);
    this.blinkOnSec = Math.max(0.01, onSec);
    this.blinkOffSec = Math.max(0.01, offSec);
    this.blinkLit = false;
    this.nextToggleTs = Timer.getFPGATimestamp();
    mode = Mode.BLINK;
  }

  private static int clamp255(int x) {
    return Math.max(0, Math.min(255, x));
  }
}
