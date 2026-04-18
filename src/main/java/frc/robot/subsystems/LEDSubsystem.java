// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.EmptyControl;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.Animation0TypeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

    private final CANdle m_candle;

    private final CANdleConfiguration m_config;

    private static final RGBWColor kWhite = new RGBWColor(255, 255, 255, 0);
    private static final RGBWColor kRed = new RGBWColor(255, 0, 0, 0);
    private static final RGBWColor kGreen = new RGBWColor(0, 255, 0, 0);
    private static final RGBWColor kBlue = new RGBWColor(0, 0, 255, 0);
    private static final RGBWColor kPurple = new RGBWColor(255, 0, 255, 0);

    public static final SolidColor kSolidWhite = 
        new SolidColor(0, 0)
        .withColor(kWhite);
        
    public static final SolidColor kSolidRed = 
        new SolidColor(0, 0).withColor(kRed);

    public static final SolidColor kSolidGreen = 
        new SolidColor(0, 0).withColor(kGreen);

    public static final SolidColor kSolidBlue = 
        new SolidColor(0, 0).withColor(kBlue);
    
    public static final StrobeAnimation kStrobeFastPurple =
        new StrobeAnimation(0, 0).withColor(kPurple).withFrameRate(5);

    public static final SingleFadeAnimation kFadePurple =
        new SingleFadeAnimation(0, 0).withColor(kPurple).withFrameRate(200);

    public static final SingleFadeAnimation kFadeRed = 
        new SingleFadeAnimation(0, 0).withColor(kRed).withFrameRate(50);

    public static final SingleFadeAnimation kFadeBlue = 
        new SingleFadeAnimation(0, 0).withColor(kBlue).withFrameRate(50);

    public static final TwinkleAnimation kTwinkleGreen = 
        new TwinkleAnimation(0, 0).withColor(kGreen).withFrameRate(200);


    /** Creates a new LEDSubsystem. */
    public LEDSubsystem() {
        m_candle = new CANdle(0, "kachow"); // Create CANdle with ID 0
        // CANdleConfiguration config = new CANdleConfiguration();

        m_config = new CANdleConfiguration();
        m_config.LED.StripType = StripTypeValue.RGB;
        m_config.LED.BrightnessScalar = 1;
        m_candle.getConfigurator().apply(m_config);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * Sets the LED pattern based on a start index, end index, and a control request.
     * Control requests are public and can be accessed in the LEDSubsystem.
     *
     * @param startIndex LED to start at.
     * @param endIndex LED to end at.
     * @param slot The animation slot to use.
     * @param pattern Control Request to use.
     */
    public void set(int startIndex, int endIndex, int slot, SolidColor pattern) {
        m_candle.setControl(new EmptyAnimation(slot));
        m_candle.setControl(pattern.withLEDStartIndex(startIndex).withLEDEndIndex(endIndex));
    }

    /**
     * Sets the LED pattern based on a start index, end index, and a control request.
     * Control requests are public and can be accessed in the LEDSubsystem.
     *
     * @param startIndex LED to start at.
     * @param endIndex LED to end at.
     * @param slot The animation slot to use.
     * @param pattern Control Request to use.
     */
    public void set(int startIndex, int endIndex, int slot, StrobeAnimation pattern) {
        m_candle.setControl(pattern.withLEDStartIndex(startIndex).withLEDEndIndex(endIndex).withSlot(slot));
    }

    /**
     * Sets the LED pattern based on a start index, end index, and a control request.
     * Control requests are public and can be accessed in the LEDSubsystem.
     *
     * @param startIndex LED to start at.
     * @param endIndex LED to end at.
     * @param slot The animation slot to use.
     * @param pattern Control Request to use.
     */
    public void set(int startIndex, int endIndex, int slot, SingleFadeAnimation pattern) {
        m_candle.setControl(pattern.withLEDStartIndex(startIndex).withLEDEndIndex(endIndex).withSlot(slot));
    }

    /**
     * Sets the LED pattern based on a start index, end index, and a control request.
     * Control requests are public and can be accessed in the LEDSubsystem.
     *
     * @param startIndex LED to start at.
     * @param endIndex LED to end at.
     * @param slot The animation slot to use.
     * @param pattern Control Request to use.
     */
    public void set(int startIndex, int endIndex, int slot, TwinkleAnimation pattern) {
        m_candle.setControl(pattern.withLEDStartIndex(startIndex).withLEDEndIndex(endIndex).withSlot(slot));
    }
}
