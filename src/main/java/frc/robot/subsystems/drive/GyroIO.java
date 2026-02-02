// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface GyroIO {
  public static class GyroIOInputs implements LoggableInputs {
    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public Rotation2d pitchPosition = new Rotation2d();
    public double pitchVelocityRadPerSec = 0.0;
    public Rotation2d rollPosition = new Rotation2d();
    public double rollVelocityRadPerSec = 0.0;
    public double[] odometryYawTimestamps = new double[] {};
    public Rotation2d[] odometryYawPositions = new Rotation2d[] {};

    @Override
    public void toLog(LogTable table) {
      table.put("Connected", connected);
      table.put("YawPositionRad", yawPosition.getRadians());
      table.put("YawVelocityRadPerSec", yawVelocityRadPerSec);
      table.put("PitchPositionRad", pitchPosition.getRadians());
      table.put("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
      table.put("RollPositionRad", rollPosition.getRadians());
      table.put("RollVelocityRadPerSec", rollVelocityRadPerSec);
      table.put("OdometryYawTimestamps", odometryYawTimestamps);
      double[] yawPosRad = new double[odometryYawPositions.length];
      for (int i = 0; i < odometryYawPositions.length; i++) {
        yawPosRad[i] = odometryYawPositions[i].getRadians();
      }
      table.put("OdometryYawPositionsRad", yawPosRad);
    }

    @Override
    public void fromLog(LogTable table) {
      connected = table.get("Connected", connected);
      yawPosition = Rotation2d.fromRadians(table.get("YawPositionRad", yawPosition.getRadians()));
      yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", yawVelocityRadPerSec);
      pitchPosition =
          Rotation2d.fromRadians(table.get("PitchPositionRad", pitchPosition.getRadians()));
      pitchVelocityRadPerSec = table.get("PitchVelocityRadPerSec", pitchVelocityRadPerSec);
      rollPosition =
          Rotation2d.fromRadians(table.get("RollPositionRad", rollPosition.getRadians()));
      rollVelocityRadPerSec = table.get("RollVelocityRadPerSec", rollVelocityRadPerSec);
      odometryYawTimestamps = table.get("OdometryYawTimestamps", odometryYawTimestamps);
      double[] yawPosRad = table.get("OdometryYawPositionsRad", new double[] {});
      odometryYawPositions = new Rotation2d[yawPosRad.length];
      for (int i = 0; i < yawPosRad.length; i++) {
        odometryYawPositions[i] = Rotation2d.fromRadians(yawPosRad[i]);
      }
    }
  }

  public default void updateInputs(GyroIOInputs inputs) {}
}
