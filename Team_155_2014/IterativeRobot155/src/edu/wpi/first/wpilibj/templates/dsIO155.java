/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author kevin
 */
public class dsIO155 {

    //Driver Station inputs outputs  
    public Joystick leftStick;
    public Joystick rightStick;
    public Joystick manualStick;
    public final int NUM_JOYSTICK_BUTTONS = 16;
    public final int HIGH_GEAR = 1; //shift to high gear 
    public final int LS_2 = 2;
    public final int LS_3 = 3;
    public final int LS_4 = 4;  //use camera to find target, dist to target, and hot or not
    public final int LS_5 = 5;
    public final int LS_6 = 6;
    public final int LS_7 = 7;
    public final int LS_8 = 8;
    public final int LS_9 = 9;
    public final int LS_10 = 10;
    public final int LS_11 = 11;
    public final int LS_12 = 12;
    public final int LS_13 = 13;
    public final int LS_14 = 14;
    public final int LS_15 = 15;
    public final int LS_16 = 16;
    public final int LOW_GEAR = 1; //shift to low gear
    public final int CENTER_CALCULATE = 2;     //center Calculate (capture image and determine hot or not; distance; and target found
    public final int RS_3 = 3;  // shift to high gear
    public final int RS_4 = 4;  //shift to low gear
    public final int RS_5 = 5;  // drive with PID
    public final int RS_6 = 6;
    public final int RS_7 = 7;
    public final int RS_8 = 8;
    public final int RS_9 = 9;
    public final int RS_10 = 10;
    public final int RS_11 = 11;
    public final int RS_12 = 12;
    public final int RS_13 = 13;
    public final int RS_14 = 14;
    public final int RS_15 = 15;
    public final int RS_16 = 16;
    
    public final int CLOSEGRABBER = 1;
    public final int OPENGRABBER = 2;
    public final int BALLSWITCHOVERIDE = 3;
    public final int BALLSWITCHENABLED = 4;
    private final DriverStationEnhancedIO dsIO;
    // digital switches 
    //public boolean tuskSwitch;
    public boolean READYLAUNCH;
    public boolean READYGRAB;
    public boolean INITGRIP;
    public boolean HALFRETRACT;
    public boolean FULLRETRACT;
    public boolean KPOW;
    public boolean LAUNCHEROVERRIDE;
    public boolean launcherAutoMan;
    public boolean hangSwitch;
    public boolean grabberAutoMan;
    public boolean dig12;
    public boolean dig13;
    public boolean dig14;
    public boolean dig15;
    public boolean dig16;

    dsIO155() {
        dsIO = DriverStation.getInstance().getEnhancedIO();
        leftStick = new Joystick(1);
        rightStick = new Joystick(2);
        manualStick = new Joystick(3);
    }

    public int update() {
        try {
            //LAUNCHEROVERRIDE = dsIO.getDigital(2);           
            KPOW = dsIO.getDigital(4);
            launcherAutoMan = dsIO.getDigital(6);
            grabberAutoMan = dsIO.getDigital(8);
            //hangSwitch = dsIO.getDigital(10);
            dig12 = dsIO.getDigital(12);
            HALFRETRACT = dsIO.getDigital(14);
            FULLRETRACT = dsIO.getDigital(16);
            
            //tuskSwitch = dsIO.getDigital(1);
            //READYLAUNCH = dsIO.getDigital(3);
            //READYGRAB = dsIO.getDigital(5);
            //INITGRIP = dsIO.getDigital(7);
            //dig13 = dsIO.getDigital(9);
            //dig14 = dsIO.getDigital(11);
            //dig15 = dsIO.getDigital(13);
            //dig16 = dsIO.getDigital(15);
            SmartDashboard.putBoolean("FIRE Button = ", KPOW);
            SmartDashboard.putBoolean("launcherAutoMan = ", launcherAutoMan);
            SmartDashboard.putBoolean("grabberAutoMan = ", grabberAutoMan);
            SmartDashboard.putBoolean("HALFRETRACT = ", HALFRETRACT);
            SmartDashboard.putBoolean("FULLRETRACT = ", FULLRETRACT);
           
            return 0;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
            KPOW = true;
            
            
            
            return -1;
             
        }
        
        
    }

    public int setDigitalOutput(int channel, boolean value) {
        try {
            dsIO.setDigitalOutput(channel, value);
            return 0;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
            return -1;
        }
    }

}
