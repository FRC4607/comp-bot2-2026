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

    private ControlRequest m_slot0;
    private ControlRequest m_slot1;
    private ControlRequest m_slot2;
    private ControlRequest m_slot3;

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

    public static final StrobeAnimation kStrobeGreen = 
        new StrobeAnimation(0, 0).withColor(kGreen).withFrameRate(10);


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
    public void set(int slot, SolidColor pattern) {

        if (slot == 0) {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7));
                m_slot0 = pattern;
            }
        } else if (slot == 1) {
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12));
                m_slot1 = pattern;
            }
        } else if (slot == 2) {
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15));
                m_slot2 = pattern;
            }
        } else if (slot == 3) {
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20));
                m_slot3 = pattern;
            }
        } else {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7));
                m_slot0 = pattern;
            }
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12));
                m_slot1 = pattern;
            }
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15));
                m_slot2 = pattern;
            }
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20));
                m_slot3 = pattern;
            }
        }
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
    public void set(int slot, StrobeAnimation pattern) {
        if (slot == 0) {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7).withSlot(0));
                m_slot0 = pattern;
            }
        } else if (slot == 1) {
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12).withSlot(1));
                m_slot1 = pattern;
            }
        } else if (slot == 2) {
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15).withSlot(2));
                m_slot2 = pattern;
            }
        } else if (slot == 3) {
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20).withSlot(3));
                m_slot3 = pattern;
            }
        } else {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7).withSlot(0));
                m_slot0 = pattern;
            }
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12).withSlot(1));
                m_slot1 = pattern;
            }
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15).withSlot(2));
                m_slot2 = pattern;
            }
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20).withSlot(3));
                m_slot3 = pattern;
            }
        }
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
    public void set(int slot, SingleFadeAnimation pattern) {
        if (slot == 0) {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7).withSlot(0));
                m_slot0 = pattern;
            }
        } else if (slot == 1) {
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12).withSlot(1));
                m_slot1 = pattern;
            }
        } else if (slot == 2) {
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15).withSlot(2));
                m_slot2 = pattern;
            }
        } else if (slot == 3) {
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20).withSlot(3));
                m_slot3 = pattern;
            }
        } else {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7).withSlot(0));
                m_slot0 = pattern;
            }
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12).withSlot(1));
                m_slot1 = pattern;
            }
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15).withSlot(2));
                m_slot2 = pattern;
            }
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20).withSlot(3));
                m_slot3 = pattern;
            }
        }
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
    public void set(int slot, TwinkleAnimation pattern) {
        if (slot == 0) {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7).withSlot(0));
                m_slot0 = pattern;
            }
        } else if (slot == 1) {
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12).withSlot(1));
                m_slot1 = pattern;
            }
        } else if (slot == 2) {
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15).withSlot(2));
                m_slot2 = pattern;
            }
        } else if (slot == 3) {
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20).withSlot(3));
                m_slot3 = pattern;
            }
        } else {
            if (m_slot0 != pattern) {
                m_candle.setControl(new EmptyAnimation(0));
                m_candle.setControl(pattern.withLEDStartIndex(0).withLEDEndIndex(7).withSlot(0));
                m_slot0 = pattern;
            }
            if (m_slot1 != pattern) {
                m_candle.setControl(new EmptyAnimation(1));
                m_candle.setControl(pattern.withLEDStartIndex(8).withLEDEndIndex(12).withSlot(1));
                m_slot1 = pattern;
            }
            if (m_slot2 != pattern) {
                m_candle.setControl(new EmptyAnimation(2));
                m_candle.setControl(pattern.withLEDStartIndex(13).withLEDEndIndex(15).withSlot(2));
                m_slot2 = pattern;
            }
            if (m_slot3 != pattern) {
                m_candle.setControl(new EmptyAnimation(3));
                m_candle.setControl(pattern.withLEDStartIndex(16).withLEDEndIndex(20).withSlot(3));
                m_slot3 = pattern;
            }
        }
    }

    public void clearAnimation(int slot) {
        m_candle.setControl(new EmptyAnimation(slot));
    }
}
