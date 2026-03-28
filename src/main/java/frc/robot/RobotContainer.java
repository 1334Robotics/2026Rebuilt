// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Driving;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDriveCommand;
import frc.robot.commands.SubsystemCommands;
import frc.robot.subsystems.DriverInfo;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Floor;
import frc.robot.subsystems.Hanger;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Rumble;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Led;
import frc.util.SwerveTelemetry;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");
    private final DriverInfo driverInfo = new DriverInfo(() -> swerve.getState().Pose);
    private final Led led = new Led(8);

    private final SwerveTelemetry swerveTelemetry = new SwerveTelemetry(Driving.kMaxSpeed.in(MetersPerSecond));
    
    private final CommandXboxController driver = new CommandXboxController(0);
    private final CommandXboxController operator = new CommandXboxController(1);

    private final Rumble rumble = new Rumble(driver, operator);

    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    );
    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        swerve.registerTelemetry(swerveTelemetry::telemeterize);
        led.turnOn();
    }
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupp lier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());

        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(hanger.homingCommand());

        // Driver controls

        // Operator
        operator.start().onTrue(intake.zeroEncoderCommand());

        operator.leftTrigger().whileTrue(intake.intakeCommand());
        operator.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());

        operator.rightBumper().whileTrue(subsystemCommands.feedAndShoot());
        operator.leftBumper().whileTrue(subsystemCommands.manualShot(0.2, 3100));

        operator.povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        operator.povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));

        // tested and working
        operator.povRight().onTrue(Commands.sequence(
                Commands.waitSeconds(0.5),
                intake.runOnce(() -> {
                    intake.intakePivotRequest = Intake.Position.INTAKE;
                    intake.set(Intake.Position.INTAKE);
                }),
                Commands.waitUntil(() -> intake.isPositionWithinTolerance() || intake.didHitLimitSwitch() || intake.currentHigh()),
                intake.runOnce(() -> intake.setPivotPercentOutput(0))
            ));
        // untested, but should work
        operator.povLeft().onTrue(Commands.sequence(
                Commands.waitSeconds(0.5),
                intake.runOnce(() -> {
                    intake.intakePivotRequest = Intake.Position.HOMED;
                    intake.set(Intake.Position.HOMED);
                }),
                Commands.waitUntil(() -> intake.isPositionWithinTolerance() || intake.didHitLimitSwitch() || intake.currentHigh()),
                intake.runOnce(() -> intake.setPivotPercentOutput(0))
            ));

        // operator.povLeft().onTrue(intake.manualRetractCommand());
        // operator.povRight().onTrue(intake.manualExtendCommand());

        // operator.a().onTrue(intake.runOnce(() -> {
        //             intake.intakePivotRequest = Intake.Position.INTAKE;
        //             intake.set(Intake.Position.INTAKE);
        //         })); 
        
        operator.b().whileTrue(shooter.spinUpCommand(3000));
        operator.a().whileTrue(subsystemCommands.manualShot(0.44, 4500));
        operator.x().whileTrue(subsystemCommands.manualShot(0.99, 4500));

        operator.y().whileTrue(subsystemCommands.manualShot(0.48, 3650));

    }

    private void configureManualDriveBindings() {
        final ManualDriveCommand manualDriveCommand = new ManualDriveCommand(
            swerve, 
            () -> -driver.getLeftY(), 
            () -> -driver.getLeftX(), 
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDriveCommand);

        // driver.a().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.k180deg)));
        // driver.b().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCW_90deg)));
        // driver.x().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kCCW_90deg)));

        // driver.y().onTrue(Commands.runOnce(() -> manualDriveCommand.setLockedHeading(Rotation2d.kZero)));
        driver.back().onTrue(Commands.runOnce(() -> manualDriveCommand.seedFieldCentric())); // the "view" button, center left of xbox logo
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getState().Pose;
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose, 
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }

    public void setIntakePosition() {
        intake.setIntakePos();
    }
}
