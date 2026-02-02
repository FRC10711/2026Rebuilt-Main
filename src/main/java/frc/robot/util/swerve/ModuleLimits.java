package frc.robot.util.swerve;

/** Kinematic limits applied by {@link SwerveSetpointGenerator}. */
public record ModuleLimits(
    double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {}
