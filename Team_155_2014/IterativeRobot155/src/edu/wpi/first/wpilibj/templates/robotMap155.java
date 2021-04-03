/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author kevin
 */
public class robotMap155 {

    //The constants to define robot connections
    //PWMs
    public final int DRIVE_LEFT = 1;
    public final int DRIVE_RIGHT = 2;
    public final int LAUNCHMOTOR = 3;
    public final int ARMMOTOR = 4;
    public final int GRIPPERMOTOR = 5;
    public final int PWM6 = 6;
    public final int PWM7 = 7;
    public final int PWM8 = 8;
    public final int PWM9 = 9;
    public final int PWM10 = 10;
    //solenoids
    public final int SOL_1 = 1;
    public final int SOL_2 = 2;
    public final int SOL_3 = 3;
    public final int SOL_4 = 4;
    public final int GRIPPEROPEN = 5;
    public final int GRIPPERCLOSED = 6;
    public final int WINCH_A = 7;
    public final int WINCH_B = 8;
    //digital I/O
    public final int LEFTENCODER_A = 2;
    public final int LEFTENCODER_B = 1;
    public final int RIGHTENCODER_A = 4;
    public final int RIGHTENCODER_B = 3;
    public final int DIG_IO_5 = 5;
    public final int DIG_IO_6 = 6;
    public final int ARMLOWLIMIT = 7;
    public final int ARMHIGHLIMIT = 8;
    public final int DIG_IO_9 = 9;
    public final int BALLSWITCH = 10;
    public final int LAUNCHLIMIT = 11;
    public final int DIG_IO_12 = 12;
    public final int DIG_IO_13 = 13;
    public final int COMPRESSOR_SWITCH = 14;
    //relays
    public final int COMPRESSOR_RELAY = 1;
    public final int RELAY_2 = 2;
    public final int RELAY_3 = 3;
    public final int RELAY_4 = 4;
    public final int RELAY_5 = 5;
    public final int RELAY_6 = 6;
    public final int RELAY_7 = 7;
    public final int RELAY_8 = 8;
    //ANALOGS
    public final int RANGEFINDER = 1;
    public final int ARMPOT = 2;
    public final int BALLSENSOR = 3;
    public final int LOADSENSOR = 4;
    public final int ANA_5 = 5;
    public final int ANA_6 = 6;
    public final int ANA_7 = 7;
    //public final int ANA_8 = 8; DO NOT USE Reserve for battery power
    //joysticks
    DigitalInput ballSwitch;
    DigitalInput highLimit;
    DigitalInput lowLimit;
    DigitalInput launchLimit;
    public AnalogChannel armPot;
    public AnalogChannel loadSensor;
    AnalogChannel rangeFinder;

    AnalogChannel ana5;
    AnalogChannel ana6;
    AnalogChannel ana7;

    robotMap155() {

        ballSwitch = new DigitalInput(BALLSWITCH);
        highLimit = new DigitalInput(ARMHIGHLIMIT);
        lowLimit = new DigitalInput(ARMLOWLIMIT);
        launchLimit = new DigitalInput(LAUNCHLIMIT);
        armPot = new AnalogChannel(ARMPOT);
        loadSensor = new AnalogChannel(LOADSENSOR);
        rangeFinder = new AnalogChannel(RANGEFINDER);
        ana5 = new AnalogChannel(ANA_5);
        ana6 = new AnalogChannel(ANA_6);
        ana7 = new AnalogChannel(ANA_7);

    }

    public void sensorOutput() {

        SmartDashboard.putBoolean("Ball Switch = ", ballSwitch.get());
        SmartDashboard.putBoolean("Arm HighLimit Switch = ", highLimit.get());
        SmartDashboard.putBoolean("Arm LowLimit Switch= ", lowLimit.get());
        SmartDashboard.putBoolean("kpuncher low limit = ", launchLimit.get());
        SmartDashboard.putNumber("Arm Pot voltage =", armPot.getAverageVoltage());
        SmartDashboard.putNumber("Load Sendor Voltage =", loadSensor.getAverageVoltage());
        SmartDashboard.putNumber("Rangefinder voltage =", rangeFinder.getAverageVoltage());

        SmartDashboard.putNumber("Ana 5 voltage =", ana5.getAverageVoltage());
        SmartDashboard.putNumber("Ana 6 voltage =", ana6.getAverageVoltage());
        SmartDashboard.putNumber("Ana 7 voltage =", ana7.getAverageVoltage());
    }
}
