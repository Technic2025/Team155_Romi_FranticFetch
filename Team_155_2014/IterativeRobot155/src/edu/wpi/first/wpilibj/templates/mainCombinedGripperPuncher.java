/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.templates.dsIO155;
import edu.wpi.first.wpilibj.templates.robotMap155;

/**
 *
 * @author Paul
 */
public class mainCombinedGripperPuncher {

    //common variables
    Joystick rightStick;
    Joystick leftStick;
    robotMap155 robotSystem;
    dsIO155 driveSystem;

    //from mainpuncher
    Solenoid pistonFireExtract;
    Solenoid pistonFireRetract;
    public double startTimePuncher;
    Talon launchMotor;             //was a talon
    AnalogChannel loadsensor;
    DigitalInput launchLimit;
    PIDController launchPID;
    DigitalInput ballSwitch;
    private final double KpPuncher = 0.0;
    private final double KiPuncher = 0.01;
    private final double KdPuncher = 0.0;

    public final double HALF = 3.0; //I NEED TO BE UPDATED..... MAYBER 3.0?
    public final double FULL = 3.8; //
    //private final double START = 2.0;
    //private final double PASTARMVALUE = 5.5;
    private int gripperState;
    String stateNamePuncher;
    private final int DEFAULT_PUNCHER = 0;
    private final int RETRACTING_PUNCHER = 1;
    private final int RETRACTED_PUNCHER = 2;
    private final int PRE_FIRE_PUNCHER = 3;
    private final int FIRING_PUNCHER = 4;
    public double retractPosition;

    public final double MANUALRETRACTSPEED = -0.75;

    //from maingripper
    Solenoid pistonGripperExtract;
    Solenoid pistonGripperRetract;
    public double startTimeGripper;
    Talon armMotor;                //was a talon
    Victor gripperMotor;
    PIDController armPID;
    private final double KpArm = -0.12;
    private final double KiArm = 0.0;
    private final double KdArm = 0.0;
    public final double Hi = 2.8; // real bot = 3.2
    public final double Lo = .5;//real bot = 1.5
    private final double ARMOUTOFWAY = 1.75; // real bot = 2
    private final double Start = 2.0;
    public final double DOWN_SCALE = .75;
    public final double SLOW_SCALE = .25;

    public final double UP_SCALE = 1.0;
    private int puncherState;
    private String stateNameGripper;
    private final int DEFAULT_GRIPPER = 0;
    //private final int GRABPREP_GRIPPER = 1;
    //private final int GRAB_GRIPPER = 2;
    //private final int LAUNCHPREP_GRIPPER = 3;
    //private final int LOADLAUNCHER_GRIPPER = 4;
    //private final int RETREAT_GRIPPER = 5;
    //private final int LAUNCH_GRIPPER = 6;
    public boolean armOutOfWay;
    public final double RETRACTED = 8;
    public boolean ballswitchEnable = true;
    public boolean isRetracted = false;
    private final boolean gripperMotorAssist;
    public boolean motorForward = true;

    //prepping for the filtering
    int length = 3;
    double[] a = new double[length];
    double[] b = new double[length];
    double[] history = new double[length];
    double input;
    double loadValue;
    double gain = 2.452606427;

    //array constants
    mainCombinedGripperPuncher(dsIO155 dsIO, robotMap155 robot) {
        //common portions
        robotSystem = robot;
        driveSystem = dsIO;
        leftStick = driveSystem.leftStick;
        rightStick = driveSystem.rightStick;

        //from mainpuncher
        pistonFireExtract = new Solenoid(robotSystem.WINCH_A);
        pistonFireRetract = new Solenoid(robotSystem.WINCH_B);
        launchMotor = new Talon(robotSystem.LAUNCHMOTOR);
        launchPID = new PIDController(KpPuncher, KiPuncher, KdPuncher, robotSystem.loadSensor, launchMotor);

        gripperState = DEFAULT_GRIPPER;
        launchPID.setOutputRange(-.75, 0);

        //from maingripper
        pistonGripperExtract = new Solenoid(robotSystem.GRIPPEROPEN);
        pistonGripperRetract = new Solenoid(robotSystem.GRIPPERCLOSED);
        armMotor = new Talon(robotSystem.ARMMOTOR);

        gripperMotor = new Victor(robotSystem.GRIPPERMOTOR);
        armPID = new PIDController(KpArm, KiArm, KdArm, robotSystem.armPot, armMotor);

        armPID.setOutputRange(-.25, .25);     //WAS -.25,.5
        gripperMotorAssist = false;

        puncherState = DEFAULT_PUNCHER;

        //clear the history array
        for (int i = 0; i < length; i++) {
            history[i] = 0;
        }
        
        //initialize the 'b' and 'a' vectors
        a[0] = 1;
        a[1] = -0.4269374068;
        a[2] = -0.2039806062;
        b[0] = 1;
        b[1] = 2;
        b[2] = 1;

        SmartDashboard.putNumber("kpuncher motor speed = ", launchMotor.getSpeed());
        LiveWindow.addActuator("kPuncher System", "kPuncher PID", launchPID);
        LiveWindow.addSensor("kPuncher  System", "Load Sensor", robotSystem.loadSensor);
        LiveWindow.addActuator("kPuncher  System", "kPuncher Motor", (LiveWindowSendable) launchMotor);

        SmartDashboard.putNumber("arm motor speed = ", armMotor.getSpeed());
        LiveWindow.addActuator("Gripper System", "Arm PID", armPID);
        LiveWindow.addSensor("Gripper System", "Arm Pot", robotSystem.armPot);
        LiveWindow.addActuator("Gripper System", "Arm Motor", (LiveWindowSendable) armMotor);

    }

    public void resetStateMachine() {
        gripperState = DEFAULT_GRIPPER;
        puncherState = DEFAULT_PUNCHER;
    }


    /*                    METHODS PERTAINING TO THE PUNCHER                   */
    public void punch() { // if arm in position and 

        pistonFireExtract.set(true);             //     fire
        pistonFireRetract.set(false);
    }

    public void stop() {  // set setpoint and open gripper
        launchMotor.set(0);
        pistonFireExtract.set(false);  //retract winch
        pistonFireRetract.set(true);
    }

    public void retract(double setPoint) {  // set setpoint and open gripper
        launchPID.setSetpoint(setPoint);
        pistonFireExtract.set(false);  //retract winch
        pistonFireRetract.set(true);
    }

    public void releaseBall() { //open gripper
        pistonGripperExtract.set(false);
        pistonGripperRetract.set(true);
    }

    public void grabBall() {    //close gripper
        pistonGripperExtract.set(true);
        pistonGripperRetract.set(false);
    }

    private void moveArmManual() {
        //System.out.println("arm PID is enabled in move arm man?  " + armPID.isEnable());
        if ((((driveSystem.manualStick.getY()) >= 0) && robotSystem.highLimit.get())
                || (((driveSystem.manualStick.getY()) <= 0) && robotSystem.lowLimit.get())) {
            armMotor.set(0);
            //System.out.println("stop arm motor");
        } else {
            if (driveSystem.manualStick.getY() <= 0) {
                if (robotSystem.armPot.getAverageVoltage() < 1.75) {
                    armMotor.set(-driveSystem.manualStick.getY() * SLOW_SCALE);
                    //System.out.println("Go down arm motor"); 
                } else {
                    armMotor.set(-driveSystem.manualStick.getY() * DOWN_SCALE);
                }

                //System.out.println("Go down arm motor");
            } else if (driveSystem.manualStick.getY() > 0) {
                if (robotSystem.armPot.getAverageVoltage() < 2) {

                    armMotor.set(-driveSystem.manualStick.getY() * UP_SCALE); // scaling movement of arm
                    //System.out.println("Go up arm motor");
                } else {
                    armMotor.set(-driveSystem.manualStick.getY() * SLOW_SCALE);
                }
            }
        }

    }

    /*                    THE STATE MACHINE METHOD                   */
    public void run(boolean teleOpOrNot, boolean fire) {
        //current + new - old

        int i;

        input = robotSystem.loadSensor.getVoltage() / gain;

        history[2] = history[1];
        history[1] = history[0];

        history[0] = input - a[1] * history[1] - a[2] * history[2];
        loadValue = b[0] * history[0] + b[1] * history[1] + b[2] * history[2];

        //puncher state machine
        //passed in:     boolean armOutOfWay
        stateNameGripper = "error";
        stateNamePuncher = "error";
        //System.out.println("Arm Motor" + armMotor.get());
        switch (puncherState) {
            case DEFAULT_PUNCHER:
                stateNamePuncher = "DEFAULT";
                launchPID.disable();
                grabBall();
                stop();

                if (!driveSystem.launcherAutoMan) {//True is Auto //I MIGHT BE BACKWARDS
                  
                    if (!driveSystem.FULLRETRACT) {  //move arm all the way?
                        retractPosition = FULL;
                        puncherState = RETRACTING_PUNCHER;
                    }
                } else {
                    if (!driveSystem.FULLRETRACT) {
                        puncherState = RETRACTING_PUNCHER;
                    }
                }

                isRetracted = false;
                break;
            case RETRACTING_PUNCHER: // pull back arm
                stateNamePuncher = "RETRACTING";

                if (!driveSystem.FULLRETRACT) {     // and not fully retracted
                    launchMotor.set(MANUALRETRACTSPEED);
                }

                if ( (robotSystem.launchLimit.get())) {
                    launchMotor.set(0);
                    isRetracted = true;
                    puncherState = RETRACTED_PUNCHER;
                }

                if (!driveSystem.KPOW && robotSystem.highLimit.get()) {     // fire switch selected?
                    puncherState = PRE_FIRE_PUNCHER;
                    startTimePuncher = Timer.getFPGATimestamp();
                }
                //if (isRetracted)
                //    puncherState = RETRACTED_PUNCHER;
                //}
                //System.out.println("Trying to retract");
                break;
            case RETRACTED_PUNCHER:     // arm pulled back
                stateNamePuncher = "RETRACTED";
                launchPID.disable();

                if ((!driveSystem.KPOW || fire) && robotSystem.highLimit.get()) {     // fire switch selected?
                    puncherState = PRE_FIRE_PUNCHER;
                    startTimePuncher = Timer.getFPGATimestamp();
                }
                if ((!driveSystem.FULLRETRACT) // full retract switch selected
                        && (retractPosition == HALF)) { // and only half retracted?
                    retractPosition = FULL;
                    puncherState = RETRACTING_PUNCHER;
                }
                break;
            case PRE_FIRE_PUNCHER:
                releaseBall();
                if (Timer.getFPGATimestamp() - startTimePuncher > .5) {
                    // fire!!
                    puncherState = FIRING_PUNCHER;
                    startTimePuncher = Timer.getFPGATimestamp();//now transition
                }
                break;
            case FIRING_PUNCHER:
                stateNamePuncher = "FIRING";
                launchPID.disable();
                launchMotor.set(0);
                punch();
                isRetracted = false;
                if (Timer.getFPGATimestamp() - startTimePuncher > .5) {
                    puncherState = DEFAULT_PUNCHER;                             //now transition

                    startTimePuncher = Timer.getFPGATimestamp();//now transition
                }
                break;
        }
        if ((robotSystem.loadSensor.getAverageVoltage() >= FULL) // winch pulled back to desired position
                || (robotSystem.launchLimit.get())) {                          // or winch pulled back to limit         
            isRetracted = true;

        }
        //gripper state machine

        //in Manual Gripper Code*************************
        //run motor only if gripper is low
        if (driveSystem.manualStick.getRawButton(1)) {
            motorForward = true;
        }
        if (driveSystem.manualStick.getRawButton(2)) {
            motorForward = false;
        }

        if ((robotSystem.armPot.getAverageVoltage() < (Lo + .25)) || (robotSystem.lowLimit.get())) {
            if (motorForward) {
                if (robotSystem.ballSwitch.get()) {
                    gripperMotor.set(-0.35);
                } else {
                    gripperMotor.set(0);
                }
            } else {
                gripperMotor.set(0.25);
            }
        } else {
            gripperMotor.set(0);
            motorForward = true;
        }

        if ((isRetracted) || //allow movement when fully retracted
                (robotSystem.armPot.getAverageVoltage() <= ARMOUTOFWAY)) //allow full movement when less than armoutofway
        {
            moveArmManual();
        } else {
            armMotor.set(0);
        }

        if ((robotSystem.loadSensor.getAverageVoltage() >= retractPosition) || (robotSystem.launchLimit.get())) {
            isRetracted = true;
        }

        System.out.println("is it retract" + isRetracted);
        SmartDashboard.putString("gripper state = ", stateNameGripper);
        SmartDashboard.putNumber("arm motor speed = ", armMotor.getSpeed());
        SmartDashboard.putString("kpuncher state = ", stateNamePuncher);
        SmartDashboard.putNumber("kpuncher motor speed = ", launchMotor.getSpeed());
        SmartDashboard.putNumber("filtered loadcell ", loadValue);

    }

}
