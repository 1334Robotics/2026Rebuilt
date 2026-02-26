package frc.robot;

import com.ctre.phoenix6.CANBus;

import frc.util.annotations.MagicNumber;

public final class Ports {
    // CAN Buses
    @MagicNumber("Okay")
    public static final CANBus kRoboRioCANBus = new CANBus("");
    public static final CANBus kCANivoreCANBus = new CANBus("CANivore");

    // Talon FX IDs
    @MagicNumber("From the robot's wiring as of 2026-02-24")
    public static final int kIntakePivot = 41;
    public static final int kIntakeRollers = 40;
    public static final int kFloor = 43;
    public static final int kFeeder = 45;
    public static final int kShooterLeft = 49;
    public static final int kShooterMiddle = 47;
    public static final int kShooterRight = 46;
    public static final int kHanger = 50;

    // PWM Ports
    @MagicNumber("Manually set as of 2026-02-25")
    public static final int kHoodLeftServo = 0;
    public static final int kHoodRightServo = 9;
}
