/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 20XX. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;
//You merely adopted the code, I gave birth to it
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationEnhancedIO;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.templates.*;
import edu.wpi.first.wpilibj.GearTooth.*;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Servo;
//hello
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Main_robot extends IterativeRobot {
    //our constants to define robot connections
    //PWMs

    static final int DRIVE_LEFT = 1;
    static final int DRIVE_RIGHT = 2;
    static final int SHOOTER_MOTOR_1 = 3;
    static final int SHOOTER_MOTOR_2 = 4;
    static final int FEEDER_MOTOR = 5;
    static final int SHOOTER_TILT_SLOT = 6;
    static final int PWM7 = 7;
    static final int PWM8 = 8;
    static final int CAMERA_TILT_SERVO = 9;
    static final int CAMERA_PAN_SERVO = 10;
    //solenoids
    static final int SHOOTER_FIRERER_A = 7;
    static final int SHOOTER_FIRERER_B = 8;
    static final int CLIMBER_SOL_HANG_A = 3;
    static final int CLIMBER_SOL_HANG_B = 4;
    static final int SOL_5 = 5;
    static final int SOL_6 = 6;
    static final int SOL_7 = 7;
    static final int SOL_8 = 8;
    //digital I/O
    static final int DIG_IO_1 = 1;
    static final int DIG_IO_2 = 2;
    static final int DIG_IO_3 = 3;
    static final int DIG_IO_4 = 4;
    static final int ENCODER_SLOT_A = 5;
    static final int ENCODER_SLOT_B = 6;
    static final int SHOOTER_LIMIT_LOW = 8;
    static final int SHOOTER_LIMIT_HIGH = 7;
    static final int CLIMBER_SWITCH = 9;
    static final int SHOOTER_TILT_B = 10;
    static final int SHOOTER_TILT_A = 11;
    static final int DIG_IO_7 = 12;
    static final int DIG_IO_8 = 13;
    static final int COMPRESSOR_SWITCH = 14;
    //relays
    static final int COMPRESSOR_RELAY = 1;
    static final int RELAY_2 = 2;
    static final int RELAY_3 = 3;
    static final int RELAY_4 = 4;
    static final int RELAY_5 = 5;
    static final int RELAY_6 = 6;
    static final int RELAY_7 = 7;
    static final int RELAY_8 = 8;
    //ANALOGS
    static final int SHOOTER_TILT_POT = 4;
    static final int TILT_RANGEFINDER = 1;
    static final int FEEDER_RANGEFINDER = 2;
    // Declare variable for the robot drive system  --stuff from their 
    int m_autoPeriodicLoops;
    int m_disabledPeriodicLoops;
    int m_telePeriodicLoops;
    int m_dsPacketsReceivedInCurrentSecond;
    static int printSec;
    static int startSec;
    //climbing variables
    DigitalInput climbLimitSwitch = new DigitalInput(CLIMBER_SWITCH);
    int climbing;
    //prep up our objects, to be initialized in an init() function
    DriverStation ds;
    DriveSystem drive;
    //Custom_camera MyCamera;
    Climb_system Climb;
    Feeder_system feed;
    shooter_system shooter;
    compressorSystem compressor;
    DriverStationLCD dsLCD = DriverStationLCD.getInstance();
    DriverStationEnhancedIO dsIO = DriverStation.getInstance().getEnhancedIO();
    Joystick leftStick = new Joystick(1);       //our drive joysticks
    Joystick rightStick = new Joystick(2);
    AnalogChannel tiltRangeFinder;
    static final int NUM_JOYSTICK_BUTTONS = 16;
    boolean[] m_rightStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS + 1)];
    boolean[] m_leftStickButtonState = new boolean[(NUM_JOYSTICK_BUTTONS + 1)];
    boolean climbSwitch;
    boolean fSwitch;
    boolean rSwitch;
    boolean faceSwitcher;
    //adaptivePIDcontroller mymotor;
    int acquire;
    boolean fireSwitch;
    double rpmSlider;
    //double maxShooterMotorSpeed = 1500;   For Drive Motors
    double maxShooterMotorSpeed = 2500;
    double autonomousRPM;
    //double autoShootSpeed = 1000;         For Drive Motors
    double autoShootSpeed = 4000;
    double shooterThreadConstant = 4.25;
    double thyme;
    boolean fireBool;
    double tiltSlider;
    double tiltInput;
    Timer timer;
    boolean tiltSwitchA;
    boolean tiltSwitchB;
    double arbitraryNumber;
    double rangeFinderScaler = (100);
    int lightCase;
    AnalogChannel rangeFinder;
    double degrees;
    double encoderCount;
    double oldEncoderCount;
    double newEncoderCount;
    Servo cameraTservo = new Servo(CAMERA_TILT_SERVO);
    double beginningCameraAngle = 90;                                                //90? 0? 
    double countModifier = 10;                                                    //Find through testing
    short shooter_enable;
    double last_auton_disabled;
    boolean Automode1;
    boolean Automode2;
    double autotimedelay;
    double camera_angle;

    /**
     * ********************************************************************
     *
     * CONSTRUCTOR FOR MAIN ROBOT *
     *
     * ********************************************************************
     */
    public Main_robot() {    //question: why put stuff in here versus the <robotINIT()> function????
        m_dsPacketsReceivedInCurrentSecond = 0;

        //define joysticks

        // Iterate over all the buttons on each joystick, setting state to false for each
        int buttonNum = 1;						// start counting buttons at button 1
        for (buttonNum = 1; buttonNum <= NUM_JOYSTICK_BUTTONS; buttonNum++) {
            m_rightStickButtonState[buttonNum] = false;
            m_leftStickButtonState[buttonNum] = false;
        }

        // Initialize counters to record the number of loops completed in autonomous and teleop modes
        m_autoPeriodicLoops = 0;
        m_disabledPeriodicLoops = 0;
        m_telePeriodicLoops = 0;

        System.out.println("BuiltinDefaultCode Constructor Completed\n");
    }

    /**
     * *********************************(asteric)***********************************
     */
    /*            INITIALIZATION FUNCTIONS                                 */
    /**
     * ********************************************************************
     */
    public void robotInit() {
        // Actions which would be performed once (and only once) upon initialization of the
        // robot would be put here.
        Watchdog.getInstance().feed();
        //connect everything up
        //MyCamera = new Custom_camera();  //I CAUSE ISSUES -> ROBOT FREEZES OVER
        drive = new DriveSystem(DRIVE_LEFT, DRIVE_RIGHT, leftStick, rightStick, 1, 1);  //tested 2/11, appears to work
        Climb = new Climb_system(CLIMBER_SOL_HANG_A, CLIMBER_SOL_HANG_B);           //tested 2/11, appears to work
        climbing = 0;
        feed = new Feeder_system(FEEDER_MOTOR);                                     //tested 2/11, appears to work
        compressor = new compressorSystem(COMPRESSOR_RELAY, COMPRESSOR_SWITCH);     //tested 2/11, appears to work
        shooter = new shooter_system(SHOOTER_FIRERER_A, SHOOTER_FIRERER_B, ENCODER_SLOT_A, ENCODER_SLOT_B,
                SHOOTER_MOTOR_1, SHOOTER_MOTOR_2, SHOOTER_TILT_SLOT,
                SHOOTER_TILT_POT, SHOOTER_TILT_A, SHOOTER_TILT_B,
                SHOOTER_LIMIT_LOW, SHOOTER_LIMIT_HIGH);

        acquire = 0;

        tiltRangeFinder = new AnalogChannel(TILT_RANGEFINDER);
        rangeFinder = new AnalogChannel(FEEDER_RANGEFINDER);
        System.out.println("RobotInit() completed.\n");
        LiveWindow.addSensor("Stuff", "Range Finder", rangeFinder);

        //newEncoderCount = 0;
        //oldEncoderCount = 0;
        //encoderCount = 0;

        //newEncoderCount = shooter.shooterPID.m_encoder.get();
        //oldEncoderCount = newEncoderCount;

        shooter_enable = 0;
        shooter.init();
    }

    public void disabledInit() {
        m_disabledPeriodicLoops = 0;			// Reset the loop counter for disabled mode
        last_auton_disabled = Timer.getFPGATimestamp();
        Watchdog.getInstance().feed();
    }

    public void autonomousInit() {
        m_autoPeriodicLoops = 0;				// Reset the loop counter for autonomous mode
        last_auton_disabled = Timer.getFPGATimestamp();
        Watchdog.getInstance().feed();
    }

    public void disabledAutonomous() {
        last_auton_disabled = Timer.getFPGATimestamp();
        Watchdog.getInstance().feed();
    }

    public void teleopInit() {
        m_telePeriodicLoops = 0;				// Reset the loop counter for teleop mode
        m_dsPacketsReceivedInCurrentSecond = 0;	// Reset the number of dsPackets in current second
        cameraTservo.setAngle(beginningCameraAngle);
        Watchdog.getInstance().feed();
        shooter.shooterTiltPID.enable();
    }

    /**
     * ********************************************************************
     */
    /*                  PERIODIC FUNCTIONS                                 */
    /**
     * ********************************************************************
     */
    public void disabledPeriodic() {
        Watchdog.getInstance().feed();      // feed the user watchdog at every period when disabled
        m_disabledPeriodicLoops++;          // increment the number of disabled periodic loops completed
        cameraTservo.setAngle(90);
        //<COMPRESSOR CODE>
        compressor.run();
    }

    public void autonomousPeriodic() {
        Watchdog.getInstance().feed();
        try {
            tiltSlider = dsIO.getAnalogIn(1);
            rpmSlider = dsIO.getAnalogIn(2);
            Automode1 = dsIO.getDigital(6);
            Automode2 = dsIO.getDigital(8);
            tiltSwitchA = false;
            tiltSwitchB = false;
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
            Automode1 = true;
            Automode2 = true;
        }
        //SHOOTER CODE
        //shooter.setShooterRPM(autoShootSpeed);
        //shooter.autoShooter();
        System.out.println("in autonomous");
        cameraTservo.setAngle(90);
        shooter.runShooter(rpmSlider);

        shooter.shooterTiltPID.enable();
        shooter.runTilt(tiltSwitchA, tiltSwitchB, rightStick.getRawButton(3), rightStick.getRawButton(2), tiltSlider);
        
        cameraTilt();
        if (Automode1) {
            shooter.enable();
            autotimedelay = 3;

        } else if (Automode2) {
            shooter.enable();
            autotimedelay = 5;
        } else {
            shooter.disable();
        }
        if (((Timer.getFPGATimestamp() - last_auton_disabled) > autotimedelay) && (shooter.shooterTiltPID.isSettled())) {
            System.out.println("shooting!!!!!");
            fireBool = true;
            shooter.fire(fireBool, fireBool);
        }
        //<COMPRESSOR CODE>
        compressor.run();

    }

    public void teleopPeriodic() {
        Watchdog.getInstance().feed();          // feed the user watchdog at every period when in teleop

        //clear the LCD to clear up the left overs
        /*dsLCD.println(DriverStationLCD.Line.kUser1, 1, "                                    ");
         dsLCD.println(DriverStationLCD.Line.kUser2, 1, "                                    ");
         dsLCD.println(DriverStationLCD.Line.kUser3, 1, "                                    ");
         dsLCD.println(DriverStationLCD.Line.kUser4, 1, "                                    ");
         dsLCD.println(DriverStationLCD.Line.kUser5, 1, "                                    ");
         dsLCD.println(DriverStationLCD.Line.kUser6, 1, "                                    ");
         dsLCD.updateLCD();*/

        //Pretty Lights
        testRange();
        SmartDashboard.putNumber("Range Finder Voltage", rangeFinder.getVoltage());
        SmartDashboard.putNumber("Range Finder Range", rangeFinder.getVoltage() * 100);
        
        try {
            faceSwitcher = dsIO.getDigital(12);
            fSwitch = dsIO.getDigital(2);
            rSwitch = dsIO.getDigital(4);
            tiltSwitchA = false;
            tiltSwitchB = false;
            //fireSwitch = dsIO.getDigital(10);
            climbSwitch = dsIO.getDigital(10);
            rpmSlider = dsIO.getAnalogIn(2);
            tiltSlider = dsIO.getAnalogIn(1);

            switch (lightCase) {
                case 0:
                    dsIO.setDigitalOutput(9, false);
                    dsIO.setDigitalOutput(11, false);
                    dsIO.setDigitalOutput(13, false);
                    SmartDashboard.putBoolean("5 Feet", false);
                    SmartDashboard.putBoolean("3 Feet", false);
                    SmartDashboard.putBoolean("1 Foot", false);
                    break;
                case 1:
                    dsIO.setDigitalOutput(9, true);
                    dsIO.setDigitalOutput(11, false);
                    dsIO.setDigitalOutput(13, false);
                    SmartDashboard.putBoolean("5 Feet", true);
                    SmartDashboard.putBoolean("3 Feet", false);
                    SmartDashboard.putBoolean("1 Foot", false);
                    break;
                case 2:
                    dsIO.setDigitalOutput(9, true);
                    dsIO.setDigitalOutput(11, true);
                    dsIO.setDigitalOutput(13, false);
                    SmartDashboard.putBoolean("5 Feet", true);
                    SmartDashboard.putBoolean("3 Feet", true);
                    SmartDashboard.putBoolean("1 Foot", false);
                    break;
                case 3:
                    dsIO.setDigitalOutput(9, true);
                    dsIO.setDigitalOutput(11, true);
                    dsIO.setDigitalOutput(13, true);
                    SmartDashboard.putBoolean("5 Feet", true);
                    SmartDashboard.putBoolean("3 Feet", true);
                    SmartDashboard.putBoolean("1 Foot", true);
                    break;
            }

        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
            fSwitch = true;
            rSwitch = true;
            climbSwitch = true;
            fireSwitch = true;
        }

        //DRIVE CODE
        drive.run(faceSwitcher);

        //SHOOTER CODE
        //shooter.setShooterRPM(rpmSlider / 3.5 * maxShooterMotorSpeed);

        if (leftStick.getRawButton(2)) {
            shooter.disable();
        }
        if (leftStick.getRawButton(3)) {
            shooter.enable();
        }

        shooter.runShooter((.5 + rpmSlider * (.5 / 3.318)) * 3.318);//runs motor half to full speed

        //FIRERER CODE
        shooter.fire(leftStick.getRawButton(1), rightStick.getRawButton(1));

        //shooter.setTiltPos(tiltSlider);
        if (climbSwitch) {
            tiltSlider = 0;
        }
        shooter.runTilt(tiltSwitchA, tiltSwitchB, rightStick.getRawButton(3), rightStick.getRawButton(2), tiltSlider);
        cameraTilt();

        //dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Slider = " + rpmSlider / 3.318 * maxShooterMotorSpeed);

        //COMPRESSOR CODE
        compressor.run();

        //FEEDING CODE
        feed.run(fSwitch, rSwitch);

        //climbing code
        if (climbSwitch && shooter.shooterTiltPID.isSettled()) {
            //System.out.println("preparing to climb....");
            switch (climbing) {
                case 0:
                    //System.out.println("waiting on robot to be ready....");
                    if (!climbLimitSwitch.get()) {
                        climbing = 1;
                        //System.out.println("robot is ready to climb");
                    }

                    break;
                case 1:
                    //System.out.println("hanging now");
                    Climb.hang();
                    break; //Winter is coming.
            }
        } else {
            //System.out.println("not climbing");
            Climb.release();
            climbing = 0;
        }


        //now make the "LCD" panel display our message -- keep me at the end
        dsLCD.updateLCD();

        //That's no moon
        m_telePeriodicLoops++;                  // increment the number of teleop periodic loops completed
        m_dsPacketsReceivedInCurrentSecond++;	// icrement DS packets received
    }

    public void testPeriodic() {
        Watchdog.getInstance().feed();
        //Pretty Lights
        if (rangeFinder.getVoltage() * rangeFinderScaler > 5) {
            lightCase = 0;
        } else if (rangeFinder.getVoltage() * rangeFinderScaler < 5 && rangeFinder.getVoltage() * rangeFinderScaler >= 3) {
            lightCase = 1;
        } else if (rangeFinder.getVoltage() * rangeFinderScaler <= 3 && rangeFinder.getVoltage() * rangeFinderScaler > 1) {
            lightCase = 2;
        } else if (rangeFinder.getVoltage() * rangeFinderScaler <= 1) {
            lightCase = 3;
        }

        try {
            switch (lightCase) {
                case 0:
                    dsIO.setDigitalOutput(9, false);
                    dsIO.setDigitalOutput(11, false);
                    dsIO.setDigitalOutput(13, false);
                    SmartDashboard.putBoolean("5 Feet", false);
                    SmartDashboard.putBoolean("3 Feet", false);
                    SmartDashboard.putBoolean("1 Foot", false);
                    break;
                case 1:
                    dsIO.setDigitalOutput(9, true);
                    dsIO.setDigitalOutput(11, false);
                    dsIO.setDigitalOutput(13, false);
                    SmartDashboard.putBoolean("5 Feet", true);
                    SmartDashboard.putBoolean("3 Feet", false);
                    SmartDashboard.putBoolean("1 Foot", false);
                    break;
                case 2:
                    dsIO.setDigitalOutput(9, true);
                    dsIO.setDigitalOutput(11, true);
                    dsIO.setDigitalOutput(13, false);
                    SmartDashboard.putBoolean("5 Feet", true);
                    SmartDashboard.putBoolean("3 Feet", true);
                    SmartDashboard.putBoolean("1 Foot", false);
                    break;
                case 3:
                    dsIO.setDigitalOutput(9, true);
                    dsIO.setDigitalOutput(11, true);
                    dsIO.setDigitalOutput(13, true);
                    SmartDashboard.putBoolean("5 Feet", true);
                    SmartDashboard.putBoolean("3 Feet", true);
                    SmartDashboard.putBoolean("1 Foot", true);

                    break;
            }
        } catch (DriverStationEnhancedIO.EnhancedIOException ex) {
        }
    }

    public void testRange() {
        if (rangeFinder.getVoltage() * rangeFinderScaler > 60) {
            lightCase = 0;
        } else if (rangeFinder.getVoltage() * rangeFinderScaler < 60 && rangeFinder.getVoltage() * rangeFinderScaler >= 36) {
            lightCase = 1;
        } else if (rangeFinder.getVoltage() * rangeFinderScaler <= 36 && rangeFinder.getVoltage() * rangeFinderScaler > 12) {
            lightCase = 2;
        } else if (rangeFinder.getVoltage() * rangeFinderScaler <= 12) {
            lightCase = 3;
        }
    }

    public void cameraTilt() {
        
        //resetCount();
        //setCount();
        double temp;
        temp=shooter.shooterTiltPID.getPosition();
        //degrees = encoderCount / countModifier;
        degrees = 0.022123893805*temp+10;
        camera_angle=beginningCameraAngle - degrees;
        cameraTservo.setAngle(camera_angle);

        dsLCD.println(DriverStationLCD.Line.kUser3, 1, "titl angle  " + degrees);
        //dsLCD.println(DriverStationLCD.Line.kUser4, 1, "encoderCount  " +encoderCount);
        dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Camera angle  " + (camera_angle));
        dsLCD.println(DriverStationLCD.Line.kUser6, 1, "PID tilt " + temp);
    }

    public void setCount() {
        newEncoderCount = shooter.shooterTiltPID.m_encoder.get();
        encoderCount += newEncoderCount - oldEncoderCount;
        oldEncoderCount = newEncoderCount;
    }

    public void resetCount() {
        if (shooter.shooterTiltPID.lowLimit.get()) {
            encoderCount = 0;
            degrees = 0;
        }
        if (shooter.shooterTiltPID.highLimit.get()) {
            encoderCount = 0;
            degrees = 20;
        }
    }
}