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

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.LEDDefaultCommand;
import frc.robot.commands.MegaTrackIterativeCommand;
import frc.robot.commands.SmashBumpCommand;
import frc.robot.commands.TestShootCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.util.LoggedTunableNumber;
import java.util.Set;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  public final Drive drive;
  // Dashboard tuning
  private LoggedTunableNumber hoodAngleDegTunable = new LoggedTunableNumber("Hood/AngleDeg", 0);
  private LoggedTunableNumber shooterVelRpsTunable = new LoggedTunableNumber("Shooter/VelRps", 0);
  private LoggedTunableNumber testShooterVel =
      new LoggedTunableNumber("TestShoot/ShooterVelRPS", 26.0);
  private LoggedTunableNumber testHoodAngle =
      new LoggedTunableNumber("TestShoot/HoodAngleDeg", 6.0);
  private LoggedTunableNumber testFeederVel = new LoggedTunableNumber("TestShoot/FeederVelRPS", 20);
  private LoggedTunableNumber testIndexerVolts =
      new LoggedTunableNumber("TestShoot/IndexerVolts", 3.0);

  @SuppressWarnings("unused")
  public final Shooter shooter;

  @SuppressWarnings("unused")
  public final Hood hood;

  @SuppressWarnings("unused")
  public final Feeder feeder;

  @SuppressWarnings("unused")
  public final Intake intake;

  @SuppressWarnings("unused")
  public final Indexer indexer;

  @SuppressWarnings("unused")
  public final Hopper hopper;

  @SuppressWarnings("unused")
  public final LED led;

  // Controller
  public final CommandXboxController controller = new CommandXboxController(0);

  /** Driver X speed supplier (forward/back). */
  public DoubleSupplier getDriveXSupplier() {
    return () -> -controller.getLeftY();
  }

  /** Driver Y speed supplier (left/right). */
  public DoubleSupplier getDriveYSupplier() {
    return () -> -controller.getLeftX();
  }

  /** Right trigger axis supplier in range [0, 1]. */
  public DoubleSupplier getRightTriggerAxisSupplier() {
    return () -> controller.getRightTriggerAxis();
  }

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIOTalonFX());
        hood = new Hood(new HoodIOTalonFX());
        feeder = new Feeder(new FeederIOTalonFX());
        intake = new Intake(new IntakeIOTalonFX());

        indexer = new Indexer(new IndexerIOTalonFX());
        hopper = new Hopper(new HopperIO() {});
        led = new LED(new LEDIO() {});
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hopper = new Hopper(new HopperIO() {});
        led = new LED(new LEDIO() {});
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        shooter = new Shooter(new ShooterIO() {});
        hood = new Hood(new HoodIO() {});
        feeder = new Feeder(new FeederIO() {});
        intake = new Intake(new IntakeIO() {});
        indexer = new Indexer(new IndexerIO() {});
        hopper = new Hopper(new HopperIO() {});
        led = new LED(new LEDIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();

    // Default commands
    led.setDefaultCommand(new LEDDefaultCommand(this));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    controller
        .povUp()
        .whileTrue(
            Commands.run(
                () -> {
                  drive.runVelocity(new ChassisSpeeds(2, 0, 0));
                },
                drive));

    // Debug: log nearest trench pre-align pose
    controller.rightStick().whileTrue(new SmashBumpCommand(this));

    // Manual tuning buttons
    controller
        .a()
        .whileTrue(new InstantCommand(() -> shooter.setVelocity(shooterVelRpsTunable.get())));
    controller.x().whileTrue(new InstantCommand(() -> hood.setAngle(hoodAngleDegTunable.get())));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    controller
        .leftBumper()
        .onTrue(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.DOWN_INTAKE)));
    controller
        .leftBumper()
        .onFalse(new InstantCommand(() -> intake.setWantedState(Intake.WantedState.UP_STOW_STOP)));
    controller
        .povDown()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput(
                      "Bump/AlignPose", FieldConstants.getNearestTrenchPrePose(drive.getPose()));
                }));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(new Translation2d(3.223, 4.030), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .rightBumper()
        .whileTrue(
            new MegaTrackIterativeCommand(this, FieldConstants.getHubLocation(Alliance.Blue)));

    // Hold Y to spin up shooter + aim hood, and press right trigger to run feeder
    controller
        .y()
        .whileTrue(
            Commands.defer(
                () -> {
                  return new TestShootCommand(
                      this,
                      () -> testShooterVel.get(),
                      () -> testHoodAngle.get(),
                      testFeederVel.get(),
                      testIndexerVolts.get(),
                      0.25);
                },
                Set.of(shooter, hood, feeder, indexer)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
