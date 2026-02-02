// Copyright 2026
package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.robot.Constants.HopperConstants;

/** HopperIO implementation using two CTRE CANrange sensors. */
public class HopperIOCANrange implements HopperIO {
  private final CANrange range1 = new CANrange(HopperConstants.CANRANGE_1_ID);
  private final CANrange range2 = new CANrange(HopperConstants.CANRANGE_2_ID);

  private final StatusSignal<Distance> distance1 = range1.getDistance();
  private final StatusSignal<Distance> distance2 = range2.getDistance();

  public HopperIOCANrange() {
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, distance1, distance2);
  }

  @Override
  public void updateInputs(HopperIOInputs inputs) {
    var status1 = BaseStatusSignal.refreshAll(distance1);
    var status2 = BaseStatusSignal.refreshAll(distance2);

    double d1 = distance1.getValueAsDouble();
    double d2 = distance2.getValueAsDouble();

    inputs.distance1Meters = d1;
    inputs.distance2Meters = d2;
    inputs.detected1 = d1 > 0.0 && d1 <= HopperConstants.DETECTION_DISTANCE_METERS;
    inputs.detected2 = d2 > 0.0 && d2 <= HopperConstants.DETECTION_DISTANCE_METERS;
    inputs.connected = status1.isOK() && status2.isOK();
  }
}
