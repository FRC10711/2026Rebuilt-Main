// Copyright 2026
package frc.robot.subsystems.led;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;

/** LEDIO implementation using CTRE CANdle. */
public class LEDIOCANdle implements LEDIO {
  private final CANdle candle = new CANdle(frc.robot.Constants.LEDConstants.CANDLE_ID);
  private final SolidColor solid =
      new SolidColor(
          frc.robot.Constants.LEDConstants.LED_START_INDEX,
          frc.robot.Constants.LEDConstants.LED_END_INDEX);

  @Override
  public void setRGB(int r, int g, int b) {
    candle.setControl(solid.withColor(new RGBWColor(r, g, b, 0)));
  }
}
