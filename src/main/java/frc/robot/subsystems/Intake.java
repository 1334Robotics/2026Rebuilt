package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.KrakenX60;
import frc.robot.Ports;
import frc.util.annotations.MagicNumber;
import frc.util.annotations.ManuallySet;

public class Intake extends SubsystemBase {
    public enum Speed {
        @MagicNumber STOP(0),
        @MagicNumber INTAKE(0.8);

        private final double percentOutput;

        private Speed(double percentOutput) {
            this.percentOutput = percentOutput;
        }

        public Voltage voltage() {
            return Volts.of(percentOutput * 12.0);
        }
    }

    public enum Position {
        @MagicNumber HOMED(110),
        @MagicNumber STOWED(100),
        @MagicNumber INTAKE(-4+10), // Add 10 to these since intake keeps trying to force down
        @MagicNumber AGITATE(20+10);

        private final double degrees;

        private Position(double degrees) {
            this.degrees = degrees;
        }

        public Angle angle() {
            return Degrees.of(degrees);
        }
    }

    @ManuallySet private static final double kPivotReduction = 50.0;
    private static final AngularVelocity kMaxPivotSpeed = KrakenX60.kFreeSpeed.div(kPivotReduction).times(0.10);
    private static final Angle kPositionTolerance = Degrees.of(5);

    private final TalonFX pivotMotor, rollerMotor;
    private final VoltageOut pivotVoltageRequest = new VoltageOut(0);
    private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0).withSlot(0);
    private final VoltageOut rollerVoltageRequest = new VoltageOut(0);

    private final DigitalInput intakeSwitchDown;
    private final DigitalInput intakeSwitchUp;

    private boolean isHomed = false;
    public Position intakePivotRequest = Position.STOWED; // where we last set the intake to

    public Intake() {
        pivotMotor = new TalonFX(Ports.kIntakePivot, Ports.kRoboRioCANBus);
        rollerMotor = new TalonFX(Ports.kIntakeRollers, Ports.kRoboRioCANBus);

        intakeSwitchDown = new DigitalInput(Ports.kIntakeDownSwitch);
        intakeSwitchUp = new DigitalInput(Ports.kIntakeUpSwitch);

        configurePivotMotor();
        configureRollerMotor();
        SmartDashboard.putData(this);
        SmartDashboard.putNumber("Intake Set Angle", -180);
    }

    private void configurePivotMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.CounterClockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            )
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor)
                    .withSensorToMechanismRatio(kPivotReduction)
            )
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(kMaxPivotSpeed)
                    .withMotionMagicAcceleration(kMaxPivotSpeed.per(Second))
            )
            .withSlot0(
                new Slot0Configs()
                    .withKP(300)
                    .withKI(0)
                    .withKD(0)
                    .withKV(12.0 / kMaxPivotSpeed.in(RotationsPerSecond)) // 12 volts when requesting max RPS
            );

        pivotMotor.getConfigurator().apply(config);
        pivotMotor.setPosition(Position.HOMED.angle());
    }

    private void configureRollerMotor() {
        final TalonFXConfiguration config = new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(InvertedValue.Clockwise_Positive)
                    .withNeutralMode(NeutralModeValue.Brake)
            )
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(Amps.of(120))
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(Amps.of(70))
                    .withSupplyCurrentLimitEnable(true)
            );
        rollerMotor.getConfigurator().apply(config);
    }

    public boolean isPositionWithinTolerance() {
        final Angle currentPosition = pivotMotor.getPosition().getValue();
        final Angle targetPosition = pivotMotionMagicRequest.getPositionMeasure();
        return currentPosition.isNear(targetPosition, kPositionTolerance);
    }

    public boolean didHitLimitSwitch() {
        return ((intakePivotRequest == Position.INTAKE) 
                    ? isPivotDown()
                    : isPivotUp());
    }

    public void setPivotPercentOutput(double percentOutput) {
        pivotMotor.setControl(
            pivotVoltageRequest
                .withOutput(Volts.of(percentOutput * 12.0))
        );
    }

    public void set(Position position) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(position.angle())
        );
    }

    public void set(Angle angle) {
        pivotMotor.setControl(
            pivotMotionMagicRequest
                .withPosition(angle)
        );
    }

    public void set(Speed speed) {
        rollerMotor.setControl(
            rollerVoltageRequest
                .withOutput(speed.voltage())
        );
    }

    public boolean currentHigh() {
        return pivotMotor.getSupplyCurrent().getValue().in(Amps) > 6;
    }

    public Command intakeCommand() {
        return startEnd(
            () -> set(Speed.INTAKE),
            () -> set(Speed.STOP)
        );
    }

    public Command agitateCommand() {
        return runOnce(() -> set(Speed.INTAKE))
            .andThen(
                Commands.sequence(
                    runOnce(() -> {
                        setPivotPercentOutput(0);
                        set(Position.AGITATE);
                    }),
                    Commands.waitUntil(() -> isPositionWithinTolerance() || currentHigh()),
                    runOnce(() -> {
                        setPivotPercentOutput(0);
                        set(Position.INTAKE);
                    }),
                    Commands.waitUntil(() -> isPositionWithinTolerance() || currentHigh())
                )
                .repeatedly()
            )
            .handleInterrupt(() -> {
                set(Speed.STOP);
                set(Position.INTAKE);
            });
    }

    public Command homingCommand() { // don't use
        return Commands.sequence(
            runOnce(() -> setPivotPercentOutput(0.025)),
            Commands.waitUntil(() -> currentHigh()),
            runOnce(() -> {
                pivotMotor.setPosition(Position.HOMED.angle());
                isHomed = true;
                set(Position.STOWED);
            })
        )
        .unless(() -> isHomed)
        .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    public Command manualExtendCommand() {
        return Commands.runEnd(
            () -> setPivotPercentOutput(-0.1),
            () -> setPivotPercentOutput(0),
            this
        );
        // return runOnce(() -> set(Position.AGITATE));
    }

    public Command manualRetractCommand() {
        return Commands.runEnd(
            () -> setPivotPercentOutput(0.1),
            () -> setPivotPercentOutput(0),
            this
        );
        // return runOnce(() -> set(Position.INTAKE));
    }

    public Command testCommand() {
        /**
         * This method is a testing command, and will probably be removed. If it isnt removed, then yap
         */
        return Commands.sequence(
            runOnce(() -> {
                setIntakePos();
                intakePivotRequest = (intakePivotRequest == Position.STOWED) ? Position.INTAKE : Position.STOWED;
                set(intakePivotRequest);
            }),
            Commands.waitUntil(() -> 
                isPositionWithinTolerance() || 
                didHitLimitSwitch()
            ),
            runOnce(() -> setPivotPercentOutput(0))
        );
    }

    public Command zeroEncoderCommand() {
        return runOnce(() -> {
            pivotMotor.setPosition(Position.HOMED.angle());
        });
    }

    public boolean isPivotUp() {
        return !intakeSwitchUp.get() && intakeSwitchDown.get();
    }

    public boolean isPivotDown() {
        return !intakeSwitchDown.get() && intakeSwitchUp.get();
    }

    public void setIntakePos() {
        if(isPivotUp()) {
            intakePivotRequest = Position.STOWED;
        } else if (isPivotDown()) {
            intakePivotRequest = Position.INTAKE;
        } else {
            DriverStation.reportWarning("The intake pivot position could not be read!", false);
        }
    }


    @Override
    public void periodic() {
        SmartDashboard.putNumber("Intake Angle", pivotMotor.getPosition().getValueAsDouble());
        SmartDashboard.putBoolean("Intake Down", isPivotDown());
        SmartDashboard.putBoolean("Intake Up", isPivotUp());
        SmartDashboard.putBoolean("Intake Stowed", (intakePivotRequest == Position.STOWED) ? true : false);
    }

    

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addStringProperty("Command", () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "null", null);
        builder.addDoubleProperty("Angle (degrees)", () -> pivotMotor.getPosition().getValue().in(Degrees), null);
        builder.addDoubleProperty("RPM", () -> rollerMotor.getVelocity().getValue().in(RPM), null);
        builder.addDoubleProperty("Pivot Supply Current", () -> pivotMotor.getSupplyCurrent().getValue().in(Amps), null);
        builder.addDoubleProperty("Roller Supply Current", () -> rollerMotor.getSupplyCurrent().getValue().in(Amps), null);
    }
}
