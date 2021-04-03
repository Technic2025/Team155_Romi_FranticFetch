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

    private int autonomousState = 0;

    AxisCamera camera = AxisCamera.getInstance("10.1.55.11");
    CriteriaCollection cc;
    double distAdjust = 0;
    boolean autoHotOrNot = false;

    DriverStationEnhancedIO dsIO = DriverStation.getInstance().getEnhancedIO();
    Compressor mainCompressor = new Compressor(14, 1);

    mainDrive155 driveSystem;
    mainVision155 cameraSystem;
    mainSensors155 sensorSystem;
    //demoShooter155 shooterSystem;
    dsIO155 driverStationSystem;
    robotMap155 robotSystem;
    mainCombinedGripperPuncher combinedGripperPuncherSystem;

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
        driverStationSystem = new dsIO155();
        robotSystem.sensorOutput();
        driverStationSystem.update();
        cc = new CriteriaCollection();      // create the criteria for the particle filter
        cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, AREA_MINIMUM, 65535, false);

        driveSystem = new mainDrive155(leftStick, rightStick, robotSystem);
        cameraSystem = new mainVision155(camera, cc);
        sensorSystem = new mainSensors155(robotSystem);
        combinedGripperPuncherSystem = new mainCombinedGripperPuncher(driverStationSystem, robotSystem);

        mainCompressor.start();
        driveSystem.startEncoders();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void disableInit() {
        robotSystem.sensorOutput();
        driverStationSystem.update();
        combinedGripperPuncherSystem.resetStateMachine();

    }

    public void disablePeriodic() {
        robotSystem.sensorOutput();
        driverStationSystem.update();
        combinedGripperPuncherSystem.resetStateMachine();
        combinedGripperPuncherSystem.armPID.disable();
        combinedGripperPuncherSystem.armPID.reset();
        combinedGripperPuncherSystem.launchPID.disable();
        combinedGripperPuncherSystem.launchPID.reset();
    }

    public void autonomousInit() {
        //startTime = Timer.getFPGATimestamp();
        mainCompressor.start();
        driveSystem.reset();
        driveSystem.disablePID();
        robotSystem.sensorOutput();
        combinedGripperPuncherSystem.resetStateMachine();
        autonomousState = DEFAULT_ARM_MOVE;
        driverStationSystem.update();
    }

    public void autonomousPeriodic() {

        robotSystem.sensorOutput();
        driverStationSystem.update();
        mainCompressor.start();

        if (!driveSystem.leftPID.isEnable()) {
            driveSystem.leftPID.enable();
        }

        if (!driveSystem.rightPID.isEnable()) {
            driveSystem.rightPID.enable();
        }

        //If statement to determine which autonomous code to run 
        // Decision 1: ball or no ball
        // Decision 2: 1 ball or 2 ball attempt
        if (driverStationSystem.grabberAutoMan) {  //if launcherAutoMan (dig out 6) true for 1 or 2 balls
            auto1();
        } else {
            auto2();

        }
        /*
         System.out.println("drive left setpoint = " + driveSystem.leftPID.getSetpoint()
         + "drive right setpoint = " + driveSystem.rightPID.getSetpoint());
         System.out.println("left ticks " + driveSystem.left_encoder.get() + "right ticks "
         + driveSystem.right_encoder.get());
         */
    }

    public void auto1() {
        //default autonomous code to drive forward and shoot one ball during autonomous
        switch (autonomousState) {
            case DEFAULT_ARM_MOVE:
                combinedGripperPuncherSystem.armMotor.set(-.5); //start arm motor lifting arm
                // if arm at high position (either by arm pot or limit switch) go to next state
                if ((robotSystem.highLimit.get())) {
                    autonomousState = MOVE_ROBOT;
                    startTimePuncher = Timer.getFPGATimestamp();
                }
                break;

            case MOVE_ROBOT:
                combinedGripperPuncherSystem.armMotor.set(0);
                driveSystem.driveDistance(11.0);

                if (Timer.getFPGATimestamp() - startTimePuncher > 1.5) {//wait to move forward
                    autonomousState = PREP_FIRE;
                    startTimePuncher = Timer.getFPGATimestamp();
                    //System.out.println("Start time for puncher = " + startTimePuncher);
                }
                break;

            case PREP_FIRE:
              
                combinedGripperPuncherSystem.releaseBall();
                if (Timer.getFPGATimestamp() - startTimePuncher > .5) {
                    cameraSystem.centerCalculate();
                    if (cameraSystem.foundTarget) {
                        if (cameraSystem.target.Hot) {
                            autonomousState = FIRING_KPUNCHER;
                        } else {
                            autonomousState = WAIT_TO_FIRE;
                            startTimePuncher = Timer.getFPGATimestamp();
                        }
                    } else {
                        autonomousState = WAIT_TO_FIRE;
                        startTimePuncher = Timer.getFPGATimestamp();
                    }
                }
                break;

            case WAIT_TO_FIRE:
                if (Timer.getFPGATimestamp() - startTimePuncher > 3) {
                    autonomousState = FIRING_KPUNCHER;
                }
                break;

            case FIRING_KPUNCHER:
                combinedGripperPuncherSystem.punch();
                combinedGripperPuncherSystem.isRetracted = false;
                break;
        }
    }

    public void auto2() {
        //autonomous code drive forward during autonomous, assuming no ball
        switch (autonomousState) {
            case DEFAULT_ARM_MOVE:
                combinedGripperPuncherSystem.armMotor.set(0);
                driveSystem.driveDistance(4);
                break;
        }
    }

    public void auto3() {
        //default autonomous code to drive forward, shoot one ball, back up retrieve second ball, 
        // drive forward and shoot second ball during autonomous
        switch (autonomousState) {
            case DEFAULT_ARM_MOVE:
                combinedGripperPuncherSystem.armMotor.set(-.5); //start arm motor lifting arm
                // if arm at high position (either by arm pot or limit switch) go to next state
                if ((robotSystem.highLimit.get())) {
                    autonomousState = MOVE_ROBOT;
                    startTimePuncher = Timer.getFPGATimestamp();
                }
                break;

            case MOVE_ROBOT:
                combinedGripperPuncherSystem.armMotor.set(0);
                driveSystem.driveDistance(6.5);

                if (Timer.getFPGATimestamp() - startTimePuncher > 1.5) {//wait to move forward
                    autonomousState = PREP_FIRE;
                    startTimePuncher = Timer.getFPGATimestamp();
                    //System.out.println("Start time for puncher = " + startTimePuncher);
                }
                break;

            case PREP_FIRE:
                combinedGripperPuncherSystem.releaseBall();
                autonomousState = FIRING_KPUNCHER;
                break;

            case FIRING_KPUNCHER:
                combinedGripperPuncherSystem.punch();
                combinedGripperPuncherSystem.isRetracted = false;
                break;

            case GOTOBALL2:
                //retract kpuncher, lower arm and drive back to second ball
                // and not fully retracted
                combinedGripperPuncherSystem.launchMotor.set(combinedGripperPuncherSystem.MANUALRETRACTSPEED);
                if ((robotSystem.loadSensor.getAverageVoltage()
                        >= combinedGripperPuncherSystem.FULL) // winch pulled back to desired position?
                        || (robotSystem.launchLimit.get())) {
                    combinedGripperPuncherSystem.launchMotor.set(0);
                    combinedGripperPuncherSystem.isRetracted = true;
                    combinedGripperPuncherSystem.armMotor.set(0.6);
                    if (robotSystem.lowLimit.get()) {
                        driveSystem.driveDistance(-6.5);
                        autonomousState = PICKUPBALL2;
                    }
                }
                break;

            case PICKUPBALL2:
                //pickup second ball
                //if arm limit is high then move to next state to move the robot up to shoot
                combinedGripperPuncherSystem.armMotor.set(-.5); //start arm motor lifting arm
                if ((robotSystem.highLimit.get())) {
                    autonomousState = MOVE_ROBOT;
                    startTimePuncher = Timer.getFPGATimestamp();
                }
                break;
        }
    }

    //* This function is called periodically during operator control
    public void teleopInit() {
        mainCompressor.start();
        driveSystem.reset();
        driveSystem.disablePID();
        driverStationSystem.update();
        if (combinedGripperPuncherSystem.armPID.isEnable()) {
            combinedGripperPuncherSystem.armPID.disable();
        }
        combinedGripperPuncherSystem.resetStateMachine();
    }

    public void teleopPeriodic() {
        robotSystem.sensorOutput();
        driverStationSystem.update();
        mainCompressor.start();
        driveSystem.driveWithJoysticks();  //Competition Robot Drive

        if (!driverStationSystem.launcherAutoMan) {
            puncherMode = "Auto Puncher";
        } else {
            puncherMode = "Manual Puncher";
        }

        combinedGripperPuncherSystem.run(true, false);
        //System.out.println("winch retracted = " + winchRetracted);

        SmartDashboard.putString("Puncher Mode = ", puncherMode);

        /**
         * *****************Sensor CODE***********************
         */
        Distance = sensorSystem.getDistance(); //get distance from the range finder
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
            cameraSystem.centerCalculate();
            if ((cameraSystem.camera_dist > 8) && (cameraSystem.camera_dist < 12)) {
                cameraSweetspot = true;
            } else {
                cameraSweetspot = false;
            }

        } else {
            cameraOn = false;
            cameraSweetspot = false;
        }
        SmartDashboard.putBoolean("Sweet Spot = ", cameraSweetspot);
        SmartDashboard.putNumber("camera distance = ", cameraSystem.camera_dist);
        SmartDashboard.putBoolean("target found = ", cameraSystem.foundTarget);

        SmartDashboard.putBoolean("Camera On = ", cameraOn);

    }

    public void testPeriodic() {
        combinedGripperPuncherSystem.armPID.startLiveWindowMode();
        driveSystem.right_encoder.startLiveWindowMode();
        driveSystem.left_encoder.startLiveWindowMode();
        System.out.println("Left encoder count: " + driveSystem.left_encoder.get());
        System.out.println("Left encoder distance: " + driveSystem.left_encoder.getDistance());
        System.out.println("Right encoder count: " + driveSystem.right_encoder.get());
        System.out.println("Right encoder distance: " + driveSystem.right_encoder.getDistance());
        LiveWindow.addActuator("Arm System", "Arm PID", combinedGripperPuncherSystem.armPID);
    }
}
