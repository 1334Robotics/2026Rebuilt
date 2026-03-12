package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANled;

// TODO: Make it flash when robot turns on (likely not in this subsystem)

public class LED extends SubsystemBase{

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

    private final CANdle m_Led  = new CANdle(CANled.ID, CANBus.roboRIO());
    private CANdleConfiguration ledCFG = new CANdleConfiguration();

    /* Define some colors here 
     * Example code provided by CTRE (allows creation of colors from multiple formats):
     */
    public final RGBWColor kGreen = new RGBWColor(0, 217, 0, 0);


    public LED(){
        // Configure the CANdle here
        ledCFG.LED.StripType = StripTypeValue.GRB;
        ledCFG.LED.BrightnessScalar = 0.5;
        m_Led.getConfigurator().apply(ledCFG);
    }

    // TODO: add setAnimatedColor functions to introduce animations
    // TODO: Should i turn these into commands? 

    public void clearAll() {
        for (int i = 0; i < 8; i++) { // Clear all LED animations
            m_Led.setControl(new EmptyAnimation(i));
        }
        m_Led.setControl(new SolidColor(CANled.Start, CANled.End).withColor(new RGBWColor())); // Sets all LEDs to off
    }

    public void setSolidColor(RGBWColor color, int index) {
        m_Led.setControl(new SolidColor(index, index).withColor(color));
    }

    public void setSolidColor(RGBWColor color, int first, int last) {
        m_Led.setControl(new SolidColor(first, last).withColor(color));
    }

    public void setAnimatedColor(int slot, RGBWColor color, Animations animation, int startIndex, int endIndex){
        switch(animation) {
            case COLOR_FLOW:
                m_Led.setControl(new ColorFlowAnimation(startIndex, endIndex)
                                    .withSlot(slot)
                                    .withColor(color));
            case FIRE:
                m_Led.setControl(new FireAnimation(startIndex, endIndex)
                                    .withSlot(slot));
            case LARSON:
                m_Led.setControl(new LarsonAnimation(startIndex, endIndex)
                                    .withSlot(slot)
                                    .withColor(color));
            case RAINBOW:
                m_Led.setControl(new RainbowAnimation(startIndex, endIndex)
                                    .withSlot(slot));
            case RGB_FADE:
                m_Led.setControl(new RgbFadeAnimation(startIndex, endIndex)
                                    .withSlot(slot));
            case SINGLE_FADE:
                m_Led.setControl(new SingleFadeAnimation(startIndex, endIndex)
                                    .withSlot(slot)
                                    .withColor(color));
            case STROBE:
                m_Led.setControl(new StrobeAnimation(startIndex, endIndex)
                                    .withSlot(slot)
                                    .withColor(color));
            case TWINKLE:
                m_Led.setControl(new TwinkleAnimation(startIndex, endIndex)
                                    .withSlot(slot)
                                    .withColor(color));
            case TWINKLE_OFF:
                m_Led.setControl(new TwinkleOffAnimation(startIndex, endIndex)
                                    .withSlot(slot)
                                    .withColor(color));
            default:
                m_Led.setControl(new EmptyAnimation(slot));
        }
    }
    public void setAnimatedColor(int slot, RGBWColor color, Animations animation){
        switch(animation) {
            case COLOR_FLOW:
                m_Led.setControl(new ColorFlowAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot)
                                    .withColor(color));
            case FIRE:
                m_Led.setControl(new FireAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot));
            case LARSON:
                m_Led.setControl(new LarsonAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot)
                                    .withColor(color));
            case RAINBOW:
                m_Led.setControl(new RainbowAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot));
            case RGB_FADE:
                m_Led.setControl(new RgbFadeAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot));
            case SINGLE_FADE:
                m_Led.setControl(new SingleFadeAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot)
                                    .withColor(color));
            case STROBE:
                m_Led.setControl(new StrobeAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot)
                                    .withColor(color));
            case TWINKLE:
                m_Led.setControl(new TwinkleAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot)
                                    .withColor(color));
            case TWINKLE_OFF:
                m_Led.setControl(new TwinkleOffAnimation(CANled.Start, CANled.End)
                                    .withSlot(slot)
                                    .withColor(color));
            default:
                m_Led.setControl(new EmptyAnimation(slot));
        }
    }
}
