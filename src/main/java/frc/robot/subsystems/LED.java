package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.SolidColor;

import edu.wpi.first.wpilibj.simulation.StadiaControllerSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// TODO (I guess)
// Make it flash when robot turns on

public class LED extends SubsystemBase{
    private final CANdle m_Led  = new CANdle(Constants.CANdle.CANdleID, CANBus.roboRIO());
    private CANdleConfiguration ledCFG = new CANdleConfiguration();
    /* Define some colors here 
     * Example code provided by CTRE (allows creation of colors from multiple formats):
     */
    public final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);

    private enum Animations {
        EMPTY,
        COLOR_FLOW,
        FIRE,
        LARSON,
        RAINBOW,
        RGB_FADE,
        SINGLE_FADE,
        STROBE,
        TWINKLE,
        TWINKLE_OFF,
    }

    public LED(){
        // Configure the CANdle here
        ledCFG.LED.StripType = StripTypeValue.GRB;
        ledCFG.LED.BrightnessScalar = 0.5;
        m_Led.getConfigurator().apply(ledCFG);

        for (int i = 0; i < 8; i++) { // Clear all LED animations
            m_Led.setControl(new EmptyAnimation(i));
        }
        m_Led.setControl(new SolidColor(0,  7).withColor(new RGBWColor())); // Sets all LEDs to off
    }

    // TODO: add setAnimatedColor functions to introduce animations
    public void setSolidColor(RGBWColor color, int index) {
        m_Led.setControl(new SolidColor(index, index).withColor(color));
    }

    public void setSolidColor(RGBWColor color, int first, int last) {
        m_Led.setControl(new SolidColor(first, last).withColor(color));
    }
}
