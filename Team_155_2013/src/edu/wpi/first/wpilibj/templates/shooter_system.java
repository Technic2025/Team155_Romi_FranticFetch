/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author kevin
 */
//test commit 2
public class shooter_system {

    public double rpmSetPoint;
    public double rpm;
    double SPEED;
    protected int m_enable_shooter;
    Solenoid fireA;
    Solenoid fireB;
    int newCount;
    int oldCount;
    //Encoder encoder;
    //Team155_PIDcontroller shooterPID;
    Team155PIDcontrollerThreaded shooterTiltPID;
    Team155PIDcontrollerThreaded shooterPID;
    //Victor shooterMotorA;
    // Victor shooterMotorB;
    Victor tiltMotor;
    AnalogChannel pot;
    double newtime;
    double oldtime;
    int fireState;
    double startTime;
    //static final double RPMCONVERT = 4.25*57.142857;      For Drive Motors
    static final double RPMCONVERT = 4.25 * 57.142857;
    static final int AVERAGESAMLPES = 20;
    double[] prevRPM;
    int prevRPMpointer;
    double averageRPM;
    public int TILTHIGHLIMIT = 1000;
    DriverStationLCD dsLCD = DriverStationLCD.getInstance();
    boolean upToSpeed;
    String tiltModeString;
    private double linPot_prev;

    //the constructor
    shooter_system(int A, int B, int encoderSlotA, int encoderSlotB, int shooterMotorSlotA, int shooterMotorSlotB, int tiltMotorSlot, int potSlot, int tiltEncoderA, int tiltEncoderB, int low_switch, int high_switch) {

        rpm = 0;
        rpmSetPoint = 0;
        m_enable_shooter = 0;
        fireA = new Solenoid(A);
        fireB = new Solenoid(B);

        //tiltMotor = new Victor(tiltMotorSlot);
        pot = new AnalogChannel(potSlot);
        //shooter tilt PID stuffs
        shooterTiltPID = new Team155PIDcontrollerThreaded(-.02, 0, 0,
                30, tiltMotorSlot, tiltEncoderA, tiltEncoderB, true,
                low_switch, high_switch, TILTHIGHLIMIT);
        shooterTiltPID.setTolerance(5);

        // shooterTiltPID = new Team155_PIDcontroller(.25, 0, 0);
        //shooter PID stuffs
        shooterPID = new Team155PIDcontrollerThreaded(0, .05, 0.0, 40, shooterMotorSlotA, shooterMotorSlotB, encoderSlotA, encoderSlotB, false);

        startTime = Timer.getFPGATimestamp();
        fireState = 1;

        //book keeping to calculate the average
        prevRPM = new double[AVERAGESAMLPES];
        int i;
        for (i = 0; i < AVERAGESAMLPES; i++) {
            prevRPM[i] = 0;
        }
        prevRPMpointer = 0;
        averageRPM = 0;
        linPot_prev = 0;
    }

    /**
     * * PUBLIC FUNCTIONS - FOR EXTERNAL CONTROL ***
     */
    public void init() {
        new Thread(shooterPID).start();
        shooterPID.enable();
        new Thread(shooterTiltPID).start();
        shooterTiltPID.enable();
    }

    //PID CONTROL OR SHOOTER
    public void setShooterRPM(double RPM) {
        if (m_enable_shooter == 1) {
            shooterPID.enable();
        } else {
            shooterPID.disable();
        }
        shooterPID.setSetpoint(RPM / RPMCONVERT);
    }

    public double getGoalRPM() {
        return shooterPID.setPoint * RPMCONVERT;
    }

    public void setTiltPos(double analog) {
        shooterTiltPID.setSetpoint(analog * 5 / 3.318 * TILTHIGHLIMIT);
    }

    //Now is not the time for fear.  That comes later.
    //
    public void runShooter(double slider) {

        //for PID slider control of shooter
        /*
         if (m_enable_shooter == 1) {
         if (slider < .25) {
         shooterPID.disable();
         } else if (slider > 3.18) {
         shooterPID.manual();
         shooterPID.set(1);

         } else {
         shooterPID.enable();
         }
         //shooterPID.set(-slider / 3.318);
         //shooterPID.setShooterRPM(-slider/3.318*4000/RPMCONVERT);     //get a [0,1] * max_rpm /rpmconvert
         } else {
         shooterPID.set(0);
         }*/
        shooterPID.manual();
        if (m_enable_shooter == 1) {

            //if ((shooterPID.status() == 2) && fireState==1)
            //    shooterPID.set(-1);
            //else
            shooterPID.set(-slider / 3.318);
        } else {
            shooterPID.set(0);
        }

        //dsLCD.println(DriverStationLCD.Line.kUser3, 1, "motor = " + shooterPID.motor_input);
        //shooterPID.manual();
        //shooterPID.set(1);
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        SmartDashboard.putNumber("Shooter Motor", shooterPID.motor_out1.get());

    }

    public void autoShooter() {
        shooterPID.enable();

    }

    public void runTilt(boolean manualMode, boolean rangeFinderMode, boolean buttonUp, boolean buttonDown, double linPot) {
        double diff;
        tiltModeString = "NULL";

        dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Slider = " + linPot);

        if (buttonUp) {
            shooterTiltPID.manual();
            shooterTiltPID.set(1);
            tiltModeString = "manual move up";
            linPot_prev = linPot;
        }
        if (buttonDown) {
            shooterTiltPID.manual();
            shooterTiltPID.set(-1);
            tiltModeString = "manual move down";
            linPot_prev = linPot;
        }

        if ((!buttonDown) && (!buttonUp) && (shooterTiltPID.status() == 2)) {  //no button presses and in manual mode?
            shooterTiltPID.set(0);
        }

        if (shooterTiltPID.status() == 1) {  //PID control enabled?
            shooterTiltPID.enable();
            shooterTiltPID.setSetpoint(linPot / 3.318 * TILTHIGHLIMIT);
            tiltModeString = "PID control";
        }

        diff = linPot - linPot_prev;
        if (((diff > .05) || (diff < -.05)) && (shooterTiltPID.status() == 2)) {  //did it change more that threshold AND in manual mode
            shooterTiltPID.enable();
        }
        //linPot_prev=linPot;

        dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Tilt" + shooterTiltPID.getPosition());
        //}

        SmartDashboard.putNumber("Tilt", shooterTiltPID.getPosition());
        SmartDashboard.putBoolean("High Limit", shooterTiltPID.highLimit.get());
        SmartDashboard.putBoolean("Low Limit", shooterTiltPID.lowLimit.get());
        SmartDashboard.putNumber("Tilt Motor", shooterTiltPID.motor_out1.get());
        SmartDashboard.putString("Tilt Mode", tiltModeString);
    }

    public void fire(boolean leftTrigger, boolean rightTrigger) {
        double shooterRPM = getRPM();
        double totalRPM = 0;

        double setPoint = getGoalRPM();
        double oldRPM;

        oldRPM = prevRPM[prevRPMpointer];
        prevRPM[prevRPMpointer] = shooterRPM / AVERAGESAMLPES;
        averageRPM = averageRPM + shooterRPM / AVERAGESAMLPES - oldRPM;

        prevRPMpointer++;
        if (prevRPMpointer >= AVERAGESAMLPES) {
            prevRPMpointer = 0;
        }

        switch (fireState) {
            case 1: //Not Ready to Fire
                fireA.set(false);
                fireB.set(true);
                if ((shooterPID.status() == 2)
                        && ((Timer.getFPGATimestamp() - startTime) >= 1)
                        || ((shooterPID.status() == 1)
                        && (averageRPM <= (setPoint * 1.05) && averageRPM >= (setPoint * .95))
                        && ((Timer.getFPGATimestamp() - startTime) >= 1.5))) {

                    fireState = 2;

                }
                //System.out.println("Not Ready to Fire");
                break;
            case 2:  //Ready to Fire
                if (leftTrigger && rightTrigger) {
                    startTime = Timer.getFPGATimestamp();
                    fireState = 3;

                }
                //System.out.println("Ready to Fire");
                break;
            case 3: //Firing
                fireA.set(true);
                fireB.set(false);

                if ((Timer.getFPGATimestamp() - startTime) >= .5) {
                    startTime = Timer.getFPGATimestamp();
                    fireState = 1;
                }
                //System.out.println("Fireing");
                break;
        }
        if ((shooterPID.status() == 2) || (averageRPM <= (setPoint * 1.05) && averageRPM >= (setPoint * .95))) {
            upToSpeed = true;
        } else {
            upToSpeed = false;
        }

        SmartDashboard.putBoolean("Up to speed?", upToSpeed);
    }

    //disable shooter
    public void disable() {
        m_enable_shooter = 0;
    }

    //enable shooter
    public void enable() {
        m_enable_shooter = 1;
    }

    //shooter status
    //  -return 1 for enable
    //  -return 0 for disable
    //  -return -1 for error/fault in shooter
    public int status() {
        return m_enable_shooter;
    }

    //perhaps he's wondering why you would shoot a man before throwing him off of a plane
    /**
     * * PROTECTED FUNCTIONS - FOR INTERNAL USE ONLY ***
     */
    public double getRPM() {

        return shooterPID.rpm * RPMCONVERT;

    }
}
