package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Ports {
    // CAN Buses
    public static final CANBus kRoboRioCANBus = new CANBus("");
    public static final CANBus kCANivoreCANBus = new CANBus("");

    // Talon FX IDs
    public static final int kIntakePivot = 0;
    public static final int kIntakeRollers = 0;
    public static final int kFloor = 0;
    public static final int kFeeder = 0;
    public static final int kShooterLeft = 0;
    public static final int kShooterMiddle = 0;
    public static final int kShooterRight = 0;
    public static final int kHanger = 0;

    // PWM Ports
    public static final int kHoodLeftServo = 3;
    public static final int kHoodRightServo = 4;
}
