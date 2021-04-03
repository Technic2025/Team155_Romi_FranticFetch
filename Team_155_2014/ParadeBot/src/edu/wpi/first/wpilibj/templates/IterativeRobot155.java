/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class IterativeRobot155 extends IterativeRobot {

    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);

    Compressor mainCompressor = new Compressor(14, 1);

    mainDrive155 driveSystem;
        robotMap155 robotSystem;

//Minimum area of particles to be considered
    final int AREA_MINIMUM = 150;
    double Distance;
    //double startTime;
    boolean readyToFire;
    boolean winchRetracted = false;
    boolean armOutOfWay = false;
    boolean teleOpOrNot;
    boolean onTarget = false;
    boolean targetHot = false;
    String gripperMode;
    String puncherMode;

    boolean puncherAuto;
    boolean gripperAuto;
    boolean cameraOn = false;
    boolean cameraSweetspot = false;
    double startTimePuncher;
    public final int DEFAULT_ARM_MOVE = 0;
    public final int MOVE_ROBOT = 1;
    public final int PREP_FIRE = 2;
    public final int WAIT_TO_FIRE = 3;
    public final int FIRING_KPUNCHER = 4;
    public final int GOTOBALL2 = 5;
    public final int PICKUPBALL2 = 6;
    double shootDelay;

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
        puncherAuto = false;
        gripperAuto = false;
        robotSystem = new robotMap155();

        robotSystem.sensorOutput();
        driveSystem = new mainDrive155(leftStick, rightStick, robotSystem);
        mainCompressor.start();
        driveSystem.startEncoders();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void disableInit() {
        robotSystem.sensorOutput();
    }

    public void disablePeriodic() {
        robotSystem.sensorOutput();
    }

    //* This function is called periodically during operator control
    public void teleopInit() {
        mainCompressor.start();
        driveSystem.reset();
        driveSystem.disablePID();

    }

    public void teleopPeriodic() {
        robotSystem.sensorOutput();

        mainCompressor.start();
        driveSystem.driveWithJoysticks();  //Competition Robot Drive

        SmartDashboard.putString("Puncher Mode = ", puncherMode);

        /**
         * *****************Sensor CODE***********************
         */
        //SmartDashboard.putNumber("range finder distance = ", Distance);
        //System.out.println("driveSystem.countTicks = " + driveSystem.countTicks());
        //System.out.println("driveSystem.distance = " + driveSystem.measureDistance());
        //System.out.println("Leftmotor =" + driveSystem.leftDrive.get());
        //System.out.println("Left Encoder = " + left_encoder);
        //System.out.println("Button pressed: " + leftStick.getRawButton(1));
        /**
         * *****************Camera Distance CODE***********************
         */
        if (rightStick.getRawButton(2)) {
            cameraOn = true;

            System.out.println("We're in!");
            SmartDashboard.putBoolean("Sweet Spot = ", cameraSweetspot);
            SmartDashboard.putBoolean("Camera On = ", cameraOn);

        }

    }
}
