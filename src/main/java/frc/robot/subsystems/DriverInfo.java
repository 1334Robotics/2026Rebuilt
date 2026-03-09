package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Landmarks;

public class DriverInfo extends SubsystemBase {
    final Supplier<Pose2d> robotPoseSupplier;

    public DriverInfo(Supplier<Pose2d> robotPoseSupplier) {
        this.robotPoseSupplier = robotPoseSupplier;
    }

    private Distance getDistanceToTarget() {
        final Translation2d robotPos = robotPoseSupplier.get().getTranslation();
        final Translation2d hubPos = Landmarks.hubPosition();
        return Meters.of(robotPos.getDistance(hubPos));
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Distance from hub (Inches)", getDistanceToTarget().in(Inches));
    }
}
