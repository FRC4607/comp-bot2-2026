// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Field;
import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;

import edu.wpi.first.hal.AddressableLEDJNI;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class LEDSubsystem extends SubsystemBase {

    private final CANdle m_candle = new CANdle(Constants.LEDConstants.kCANdleID, "kachow");
    //private final CANdle m_candle = new CANdle(Constants.LEDConstants.kCANdleID);
    
    private static enum LEDSubsystemState {
        DISABLED,
        NEUTRAL,
        INTAKE,
        MANIPULATOR_NOT_READY,
        MANIPULATOR_READY,
        CLIMB,
        ERROR
    }

    private Alliance m_alliance = null;

    // private final AddressableLED m_led = new AddressableLED(1);
    // private final SK6811RGBWBuffer m_buffer = new SK6811RGBWBuffer(64);

    private static LEDSubsystemState m_currentState = LEDSubsystemState.DISABLED;
    private static LEDSubsystemState m_pastState = null;

    private final StrobeAnimation m_noAlliance =        new StrobeAnimation(        255,      0,    255,    0,  0.5,    Constants.LEDConstants.kRGBCount);
    private final SingleFadeAnimation m_redDisabled =   new SingleFadeAnimation(    255,      0,    0,      0,  0.3,    Constants.LEDConstants.kRGBCount);
    private final SingleFadeAnimation m_blueDisabled =  new SingleFadeAnimation(    0,      0,      255,    0,  0.3,    Constants.LEDConstants.kRGBCount);
    private final LarsonAnimation m_intake =            new LarsonAnimation(        165,    255,    0,      0,  0.25,   Constants.LEDConstants.kRGBCount, LarsonAnimation.BounceMode.Back, 3);
    private final StrobeAnimation m_error =             new StrobeAnimation(        255,      0,    0,      0,  0.5,    Constants.LEDConstants.kRGBCount);
    private final TwinkleOffAnimation m_manipulatorNotReady = new TwinkleOffAnimation(    0,      255,    0,      0,  1,      Constants.LEDConstants.kRGBCount, TwinkleOffAnimation.TwinkleOffPercent.Percent64);
    private final StrobeAnimation m_manipulatorReady =        new StrobeAnimation(        255,    0,      0,      0,  1,      Constants.LEDConstants.kRGBCount);
    private final LarsonAnimation m_climbLEFT=          new LarsonAnimation(        165,    255,    0,      0,  .1,   Constants.LEDConstants.kRGBSection1.length, LarsonAnimation.BounceMode.Back, 4, Constants.LEDConstants.kRGBSection1.start);
    private final LarsonAnimation m_climbTOP=         new LarsonAnimation(        165,    255,    0,      0,  .1,   Constants.LEDConstants.kRGBSection2.length, LarsonAnimation.BounceMode.Center, 4, Constants.LEDConstants.kRGBSection2.start);
    private final LarsonAnimation m_climbRIGHT=         new LarsonAnimation(        165,    255,    0,      0,  .1,   Constants.LEDConstants.kRGBSection3.length, LarsonAnimation.BounceMode.Back, 4, Constants.LEDConstants.kRGBSection3.start);
    
   

    public LEDSubsystem() {
        CANdleConfiguration config = new CANdleConfiguration();
        config.stripType = LEDStripType.RGB; // The chips we use seem to be RGB
        config.brightnessScalar = 1; // Avoid drawing too much current
        // m_buffer.fillRGBW(255, 0, 0, 0);
        // m_led.setBitTiming(300, 900, 600, 600);
        // m_led.setSyncTime(100);
        // m_led.setLength(m_buffer.getFakeLength());
        // setBuf(m_led, m_buffer);
        // m_led.start();
        m_candle.configAllSettings(config);
        m_candle.configLEDType(LEDStripType.RGB);
        m_candle.clearAnimation(0);
        m_candle.clearAnimation(1); 
        m_candle.clearAnimation(2);
        m_candle.clearAnimation(3);
        
        SmartDashboard.putBoolean("Animation", false);
    }

    @Override
    public void periodic() {

        boolean colorUpdate = false;
        if (m_alliance == null) {
            Optional<Alliance> value = DriverStation.getAlliance();
            if (value.isPresent()) {
                m_alliance = value.get();
                colorUpdate = true;
            }
        }

        if ((m_currentState != m_pastState) || colorUpdate) {
            switch (m_currentState) {
                case DISABLED:
                    if (m_alliance == Alliance.Blue) {
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_blueDisabled,0);
                    } else if (m_alliance == Alliance.Red) {
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_redDisabled,0);
                    } else {
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.clearAnimation(0);
                        m_candle.animate(m_noAlliance,0);
                    }
                    break;
                case NEUTRAL:
                    if (m_alliance == Alliance.Blue) {
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.clearAnimation(0);
                        m_candle.setLEDs(0, 0, 255, 0, 0, Constants.LEDConstants.kRGBCount);
                    } else if (m_alliance == Alliance.Red) {
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.clearAnimation(0);
                        m_candle.setLEDs(0, 255, 0, 0, 0, Constants.LEDConstants.kRGBCount);
                    } else {
                        m_candle.clearAnimation(2);
                        m_candle.clearAnimation(1);
                        m_candle.animate(m_noAlliance,0);
                    }
                    break;
                case INTAKE:
                    m_candle.clearAnimation(2);
                    m_candle.clearAnimation(1);
                    m_candle.animate(m_intake,0);
                    break;
                case MANIPULATOR_NOT_READY:
                    m_candle.clearAnimation(2);
                    m_candle.clearAnimation(1);
                    m_candle.animate(m_manipulatorNotReady,0);
                    break;
                case MANIPULATOR_READY:
                    m_candle.clearAnimation(2);
                    m_candle.clearAnimation(1);
                    m_candle.animate(m_manipulatorReady,0);
                    break;
                case CLIMB:
                    m_candle.animate(m_climbRIGHT, 2);
                    m_candle.animate(m_climbTOP, 1);
                    m_candle.animate(m_climbLEFT, 0);
                    break;
                case ERROR:
                    m_candle.clearAnimation(2);
                    m_candle.clearAnimation(1);
                    m_candle.animate(m_error,0);
                    //m_candle.setLEDs(255, 0, 0, 0, 0, Constants.LEDConstants.kRGBCount);
                    break;
            }
            System.out.println(m_currentState);
        }
        m_pastState = m_currentState;
    }

    public static void setDisabled() {
        m_currentState = LEDSubsystemState.DISABLED;
    }

    public static void setNeutral() {
        m_currentState = LEDSubsystemState.NEUTRAL;
    }

    public static void setIntake() {
        m_currentState = LEDSubsystemState.INTAKE;
    }

    public static void setClimb() {
        m_currentState = LEDSubsystemState.CLIMB;
    }

    public static void setManipulatorNotReady() {
        m_currentState = LEDSubsystemState.MANIPULATOR_NOT_READY;
    }

    public static void setManipulatorReady() {
        m_currentState = LEDSubsystemState.MANIPULATOR_READY;
    }

    public static void setError() {
        m_currentState = LEDSubsystemState.ERROR;
    }

}
