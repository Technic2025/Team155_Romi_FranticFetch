/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.SpeedController;
/* import edu.wpi.first.wpilibj.Talon; */
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Paulie
 */
public class mainDrive155 {

    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    //RobotDrive mainDrive = new RobotDrive(1, 2);
    Solenoid piston1extract = new Solenoid(1);
    Solenoid piston1retract = new Solenoid(2);
    Encoder left_encoder;
    Encoder right_encoder;
    robotMap155 robotSystem;
    PIDController leftPID;
    PIDController rightPID;
    private final double Kp = 0.025;   //was .12
    private final double Ki = 0.0;
    private final double Kd = 0.0;
    /* public Talon leftDrive;
     public Talon rightDrive; */
    public Jaguar leftDrive;
    public Jaguar rightDrive;

    private boolean highSpeed;

    public mainDrive155(Joystick left, Joystick right, robotMap155 robot) {
        robotSystem = robot;
        leftStick = left;
        rightStick = right;
        left_encoder = new Encoder(robotSystem.LEFTENCODER_A, robotSystem.LEFTENCODER_B, false, CounterBase.EncodingType.k1X);
        right_encoder = new Encoder(robotSystem.RIGHTENCODER_A, robotSystem.RIGHTENCODER_B, false, CounterBase.EncodingType.k1X);
        /*      leftDrive = new Talon(robotSystem.DRIVE_LEFT);
         rightDrive = new Talon(robotSystem.DRIVE_RIGHT); */
        leftDrive = new Jaguar(robotSystem.DRIVE_LEFT);
        rightDrive = new Jaguar(robotSystem.DRIVE_RIGHT);
        leftPID = new PIDController(Kp, Ki, Kd, left_encoder, leftDrive);
        rightPID = new PIDController(Kp, Ki, Kd, right_encoder, rightDrive); //temporary fix until we have correct encoder
        left_encoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        right_encoder.setPIDSourceParameter(Encoder.PIDSourceParameter.kDistance);
        leftPID.setOutputRange(-0.35, 0.35);
        rightPID.setOutputRange(-0.35, 0.35);
        left_encoder.setDistancePerPulse(0.037);
        right_encoder.setDistancePerPulse(0.037);
        leftPID.setPercentTolerance(50);
        rightPID.setPercentTolerance(50);
        LiveWindow.addActuator("Drive System", "Left PID", leftPID);
        LiveWindow.addActuator("Drive System", "Reft PID", rightPID);
        LiveWindow.addSensor("Drive System", "Left Encoder", left_encoder);
        LiveWindow.addSensor("Drive System", "Right Encoder", right_encoder);
        LiveWindow.addActuator("Drive System", "Right Drive", (LiveWindowSendable) rightDrive);
        LiveWindow.addActuator("Drive System", "Left Drive", (LiveWindowSendable) leftDrive);

        highSpeed = false;

    }

    public void startEncoders() {
        right_encoder.start();
        left_encoder.start();
    }

    public void stopEncoders() {
        right_encoder.stop();
        left_encoder.stop();
    }

    public void enablePID() {
        leftPID.enable();
        rightPID.enable();
    }

    public void disablePID() {
        leftPID.disable();
        rightPID.disable();
    }

    public void reset() {
        leftPID.reset();
        rightPID.reset();
        right_encoder.reset();
        left_encoder.reset();
    }

    public void driveWithJoysticks() {
        //mainDrive.tankDrive(leftStick, rightStick);  //we are using tank drive to drive the robot
        leftDrive.set(-leftStick.getY());
        rightDrive.set(rightStick.getY());
        //System.out.println("in Drive with Joysticks");
        System.out.println("LeftStick =" + leftStick.getY());
        //System.out.println("RightStick =" + rightStick.getY());

        //this code shifts gears when certain buttons are pressed
        if (leftStick.getRawButton(1)) { //button 1 on right is equal to shift to low gear.
            piston1extract.set(true);
            piston1retract.set(false);
        }
        if (rightStick.getRawButton(1)) { //button 1 on left is equal to high gear
            piston1extract.set(false);
            piston1retract.set(true);
        }
    }

    public double measureDistance() {
        return left_encoder.getDistance();
    }

    public int countTicks() {
        return left_encoder.get();
    }

    public void driveDistance(double distance) {
        leftPID.setSetpoint(-distance * 12);//right constant?
        rightPID.setSetpoint(distance * 12);
        //rightDrive.set(leftDrive.get());
        System.out.println("Left motor =" + left_encoder.get());
        System.out.println("Right motor =" + right_encoder.get());
    }

    public void driveWithJoysticksfake() {
        //mainDrive.tankDrive(leftStick, rightStick);  //we are using tank drive to drive the robot

        System.out.println("in Drive with Joysticks");
        System.out.println("LeftStick =" + leftStick.getY());
        System.out.println("RightStick =" + rightStick.getY());

        //this code shifts gears when certain buttons are pressed
        if (leftStick.getRawButton(1)) { //button 1 on right is equal to shift to low gear.
            piston1extract.set(true);
            piston1retract.set(false);
            highSpeed = false;

        }
        if (rightStick.getRawButton(1)) { //button 1 on left is equal to high gear
            piston1extract.set(false);
            piston1retract.set(true);
            highSpeed = true;

        }

        if (highSpeed) {
            leftDrive.set(leftStick.getY());
            rightDrive.set(-rightStick.getY());
        } else {
            leftDrive.set(.75 * leftStick.getY());
            rightDrive.set(-.75 * rightStick.getY());

        }

    }
}
