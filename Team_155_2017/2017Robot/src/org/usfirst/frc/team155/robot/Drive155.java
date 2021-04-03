package org.usfirst.frc.team155.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drive155 {
	private static final double Kp = 0;
	robotMap155 robotSystem;
	SmartDashboard sDash;
	PixyVision155 camera;
	public RobotDrive myrobot;

	LiveWindow lw;

	// DRIVE MODE SELECTION
	public int driveMode;
	public int driveState;
	public final int TANK_MODE = 0;
	public final int MECANUM_MODE = 1;
	public double startTime;
	public double deltaTime = 0.25;
	public final double THRESHOLD = .1;
	public boolean preTank = false;
	public boolean inMech = false;

	// JOYSTICKS
	Joystick leftStick;
	Joystick rightStick;

	// MOTORS FOR COMPEITION ROBOT

	public CANTalon left_front;
	public CANTalon right_front;
	public CANTalon left_back;
	public CANTalon right_back;

	/*
	 * public Talon left_front; public Talon right_front; public Talon
	 * left_back; public Talon right_back;
	 */
	// DRIVE ENCODERS

	// YAW RATE SENSOR
	public Gyro roboGyro;
	public AnalogInput wallfinder;
	double WALLSCALE = 3.33;

	double motorScale = 1;
	public double drivesetpoint = 0;

	boolean starttime = true;
	double timer;
	double drivesetpointtime;
	double straighttimescale = 1.5;
	double slidescaletime = .5;

	// Encoder Controllers

	double dKf = 0;
	double dKp = 0.1; // For example if you want your mechanism to drive 50%
						// throttle when the error is 360 (one rotation when
						// using CTRE Mag Encoder, see section 17.2.1),
	// then the calculated Proportional Gain would be (0.50 X 1023) / 4096 =
	// ~0.125.
	double dKi = 0;
	double dKd = 0;

	double distanceTol = 10;
	double straightscale = 1040;
	double slidescale = 1300;
	double Front_Right_Kp = .05;
	double Front_Right_Ki = 0;
	double Front_Right_Kd = 0;
	double Front_Left_Kp = .05;
	double Front_Left_Ki = 0;
	double Front_Left_Kd = 0;
	double Rear_Right_Kp = .05;
	double Rear_Right_Ki = 0;
	double Rear_Right_Kd = 0;
	double Rear_Left_Kp = .05;
	double Rear_Left_Ki = 0;
	double Rear_Left_Kd = 0;

	// for the gyro stabilized field oriented drive
	private double foo = 2; // more of an integration scaling factor.
	private double headingSetPoint;
	private double error;
	private double PIDoutput;
	private double Kp_FieldOrientedControl = .007; // not tuned
	private double Kp_Turn = .0125;
	private boolean holdHeading;
	private boolean prevCentered;
	private boolean centered;
	public double distanceS = 0;

	double FORWARDSPEED = .35;

	boolean atHeading = false;

	Boolean reset_trigger;
	Boolean prev_reset_trigger;

	// SOLENOIDS

	DoubleSolenoid sol;
	private final int UP = 0;
	private final int DOWN = 1;
	double start_time = Timer.getFPGATimestamp();
	// navx stuffs
	SerialPort serial_port;
	AHRS ahrs;

	public Drive155(robotMap155 robot, PixyVision155 cameraSystem) {
		ahrs = new AHRS(SerialPort.Port.kMXP);
		robotSystem = robot;
		camera = cameraSystem;
		sDash = new SmartDashboard();
		lw = new LiveWindow();

		// vision = new Vision155();
		leftStick = new Joystick(robotSystem.LEFTSTICK);
		rightStick = new Joystick(robotSystem.RIGHTSTICK);

		// DRIVE STATE
		driveMode = MECANUM_MODE;
		driveState = 4;

		// navx_init();
		// *****************************************************************************

		// MOTORS FOR COMPEITION ROBOT

		left_front = new CANTalon(robotSystem.DRIVE_LEFT_FRONT);
		right_front = new CANTalon(robotSystem.DRIVE_RIGHT_FRONT);
		left_back = new CANTalon(robotSystem.DRIVE_LEFT_BACK);
		right_back = new CANTalon(robotSystem.DRIVE_RIGHT_BACK);

		/*
		 * myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		 * myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
		 * myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		 * myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
		 */
		// ENCODERS

		wallfinder = new AnalogInput(robotSystem.WALL_FINDER);
		
		
		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position
		 */
		int absolutePositionLF = left_front.getPulseWidthPosition()
				& 0xFFF; /*
							 * mask out the bottom12 bits, we don't care about
							 * the wrap arounds
							 */
		/* use the low level API to set the quad encoder signal */
		left_front.setEncPosition(absolutePositionLF);

		/* choose the sensor and sensor direction */
		// left_front.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		left_front.reverseSensor(true);
		left_front.configEncoderCodesPerRev(360); // if using
													// FeedbackDevice.QuadEncoder
		// _talon.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		left_front.configNominalOutputVoltage(+0f, -0f);
		left_front.configPeakOutputVoltage(+12f, -12f);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		left_front.setAllowableClosedLoopErr(0); /* always servo */
		/* set closed loop gains in slot0 */
		left_front.setProfile(0);
		left_front.setF(dKf);
		left_front.setP(dKp);
		left_front.setI(dKi);
		left_front.setD(dKd);

		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position
		 */
		int absolutePositionLR = left_back.getPulseWidthPosition()
				& 0xFFF; /*
							 * mask out the bottom12 bits, we don't care about
							 * the wrap arounds
							 */
		/* use the low level API to set the quad encoder signal */
		left_back.setEncPosition(absolutePositionLR);

		/* choose the sensor and sensor direction */
		// left_front.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		left_back.reverseSensor(true);
		left_back.configEncoderCodesPerRev(360); // if using
													// FeedbackDevice.QuadEncoder
		// _talon.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		left_back.configNominalOutputVoltage(+0f, -0f);
		left_back.configPeakOutputVoltage(+12f, -12f);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		left_back.setAllowableClosedLoopErr(0); /* always servo */
		/* set closed loop gains in slot0 */
		left_back.setProfile(0);
		left_back.setF(dKf);
		left_back.setP(dKp);
		left_back.setI(dKi);
		left_back.setD(dKd);
		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position
		 */
		int absolutePositionRF = left_front.getPulseWidthPosition()
				& 0xFFF; /*
							 * mask out the bottom12 bits, we don't care about
							 * the wrap arounds
							 */
		/* use the low level API to set the quad encoder signal */
		right_front.setEncPosition(absolutePositionRF);

		/* choose the sensor and sensor direction */
		// left_front.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		right_front.reverseSensor(false);
		right_front.configEncoderCodesPerRev(360); // if using
													// FeedbackDevice.QuadEncoder
		// _talon.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		right_front.configNominalOutputVoltage(+0f, -0f);
		right_front.configPeakOutputVoltage(+12f, -12f);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		right_front.setAllowableClosedLoopErr(0); /* always servo */
		/* set closed loop gains in slot0 */
		right_front.setProfile(0);
		right_front.setF(dKf);
		right_front.setP(dKp);
		right_front.setI(dKi);
		right_front.setD(dKd);

		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position
		 */
		int absolutePositionRR = right_back.getPulseWidthPosition()
				& 0xFFF; /*
							 * mask out the bottom12 bits, we don't care about
							 * the wrap arounds
							 */
		/* use the low level API to set the quad encoder signal */
		right_back.setEncPosition(absolutePositionRR);

		/* choose the sensor and sensor direction */
		// right_front.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		right_back.reverseSensor(false);
		right_back.configEncoderCodesPerRev(250); // if using
													// FeedbackDevice.QuadEncoder
		// _talon.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		right_back.configNominalOutputVoltage(+0f, -0f);
		right_back.configPeakOutputVoltage(+12f, -12f);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		right_back.setAllowableClosedLoopErr(0); /* always servo */
		/* set closed loop gains in slot0 */
		right_back.setProfile(0);
		right_back.setF(dKf);
		right_back.setP(dKp);
		right_back.setI(dKi);
		right_back.setD(dKd);

		// CONTROLLER

		// PIDGyro = new PIDController(Kp,Ki,Kd,roboGyro,PIDGyroOut);

		headingSetPoint = 0;
		holdHeading = true;
		prevCentered = false;
		centered = false;

		myrobot = new RobotDrive(left_front, left_back, right_front, right_back);

		// YAW RATE (GYRO)
		// roboGyro = new AnalogGyro(robotSystem.GYRO);

		sol = new DoubleSolenoid(robot.DRIVE_SOL_A, robot.DRIVE_SOL_B);
	}

	/*
	 * private void mecanum_fieldOriented() {
	 * 
	 * myrobot.mecanumDrive_Cartesian(leftStick.getX(), leftStick.getY(),
	 * rightStick.getX(), ahrs.getAngle());
	 * //myrobot.mecanumDrive_Cartesian(leftStick.getX(), leftStick.getY(),
	 * rightStick.getX(), roboGyro.getAngle());
	 * 
	 * }
	 */
	public void driveStraight(double heading, double speed) {
		// double error = heading - roboGyro.getAngle();
		double error = heading - ahrs.getAngle();
		double turnRate = error * Kp;
		double maxturnRate = .33;
		if (turnRate > maxturnRate)
			turnRate = maxturnRate;
		else if (turnRate < -maxturnRate)
			turnRate = -maxturnRate;
		myrobot.arcadeDrive(speed, turnRate);
		//
		// SmartDashboard.putNumber("Gyro", roboGyro.getAngle());

	}

	public void driveMecanum(double heading, double speed, double direction) {
		System.out.println("In driveMecanum");
		System.out.println("heading = " + heading);
		System.out.println("speed = " + speed);
		System.out.println("direction = " + direction);

		// System.out.println("roboGyro.getAngle() = " + roboGyro.getAngle());
		// double error = heading - roboGyro.getAngle();

		double error = heading - ahrs.getAngle();
		System.out.println("error = " + error);

		PowerDistributionPanel pdp = new PowerDistributionPanel();
		double turnRate = error * Kp;
		System.out.println("turnRate = " + turnRate);

		pdp.clearStickyFaults(); // clear the brownouts from last run, if there
									// are any

		double maxturnRate = .25;

		if (turnRate > maxturnRate)
			turnRate = maxturnRate;
		else if (turnRate < -maxturnRate)
			turnRate = -maxturnRate;
		System.out.println("turnRate(after max compare) = " + turnRate);
		System.out.println("Going to mecanumDrive_Polar");
		System.out.println("speed = " + speed);
		System.out.println("direction = " + direction);
		System.out.println("turnRate(after max compare) = " + turnRate);

		myrobot.mecanumDrive_Polar(speed, direction, turnRate);
		// SmartDashboard.putNumber("Gyro", roboGyro.getAngle());
		// SmartDashboard.putNumber("Motor1 current", pdp.getCurrent(12));
	}

	public void centerYellowTote(double goalposition, double speed, double toteposition) {

		System.out.println("In centerYellowTote");
		System.out.println("goalposition = " + goalposition);
		System.out.println("speed = " + speed);
		System.out.println("toteposition = " + toteposition);

		double error = goalposition - toteposition;
		System.out.println("error(= goalposition - toteposition) = " + error);

		double camKp = 0;
		System.out.println("camKp = " + camKp);
		double slideRate = -error * camKp;
		System.out.println("slideRate(= -error * camKp) = " + slideRate);

		double maxslideRate = .5;
		// double minslideRate = .1;

		if (slideRate > maxslideRate)
			slideRate = maxslideRate;
		else if (slideRate < -maxslideRate)
			slideRate = -maxslideRate;

		System.out.println("slideRate(adj for maxslideRate) = " + slideRate);

		// if ((slideRate < minslideRate)&&(slideRate > -minslideRate))
		// slideRate = 0;
		// Timer.delay(0.005);

		System.out.println("Going to mecanumDrive_Cartesian");
		System.out.println("x = " + slideRate);
		System.out.println("y = 0");
		System.out.println("speed = " + speed);
		// System.out.println("GyroAngle = " + roboGyro.getAngle());

		myrobot.mecanumDrive_Cartesian(slideRate, speed, 0, ahrs.getAngle());

		// myrobot.mecanumDrive_Cartesian(slideRate, speed, 0,
		// roboGyro.getAngle());
		// SmartDashboard.putNumber("toteposition=", toteposition);
		// SmartDashboard.putNumber("error=", error);
		// SmartDashboard.putNumber("sliderate = ", slideRate);
	}

	public double getGyro() {
		// return roboGyro.getAngle();
		return ahrs.getAngle();
	}

	public void GyroReset() {
		// roboGyro.reset();
		ahrs.reset();
		headingSetPoint = 0;
		holdHeading = true;
		prevCentered = false;
		centered = false;

	}

	// drive selector
	public void mecanumstop() {
		// myrobot.mecanumDrive_Polar(Speed, direction, rotation);
		myrobot.mecanumDrive_Polar(0, 0, 0);
	}

	public int DriveMode() {

		return driveMode;
	}

	public void run() {
		left_front.enableBrakeMode(false);
		right_front.enableBrakeMode(false);
		left_back.enableBrakeMode(false);
		right_back.enableBrakeMode(false);

		// EncoderDistance();
		PowerDistributionPanel pdp = new PowerDistributionPanel();
		// / SmartDashboard.putNumber(key, value);
		// SmartDashboard.putNumber("Motor0 current", pdp.getCurrent(0));
		// SmartDashboard.putNumber("Motor1 current", pdp.getCurrent(1));
		// SmartDashboard.putNumber("Motor12 current", pdp.getCurrent(12));
		// SmartDashboard.putNumber("Motor13 current", pdp.getCurrent(13));
		// SmartDashboard.putNumber("pdp voltage", pdp.getVoltage());

		if (leftStick.getRawButton(robotSystem.GYRORESET))
			GyroReset();

		if (driveMode == 1)
			inMech = true;
		else
			inMech = false;

		sDash.putBoolean("IN MECHANUM", inMech);
		/*
		 * left_front.changeControlMode(TalonControlMode.Voltage);
		 * left_back.changeControlMode(TalonControlMode.Voltage);
		 * right_front.changeControlMode(TalonControlMode.Voltage);
		 * right_back.changeControlMode(TalonControlMode.Voltage);
		 */
		switch (driveState) {
		// tank mode - button press
		case 0: {
			robotSystem.ledMecanumMode(false);
			driveMode = TANK_MODE;
			if (rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
				driveState = 1;
				startTime = Timer.getFPGATimestamp();
			}

			break;
		}
		// mecanum mode - second button press release
		case 1: {
			if (driveMode == MECANUM_MODE) {
				if (!rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
					driveState = 0;
				}
				// If there's joystick input, the drive mode is set to mecanum
				if ((Math.abs(leftStick.getX()) > THRESHOLD) || (Math.abs(leftStick.getY()) > THRESHOLD)
						|| (Math.abs(rightStick.getX()) > THRESHOLD) || (Math.abs(rightStick.getY()) > THRESHOLD)) {
					driveState = 4;
				}
				// timeout
				if (Timer.getFPGATimestamp() - startTime > deltaTime) {
					driveState = 4;
				}
				// tank mode - button press release
			} else {
				if (!rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
					driveState = 2;
					startTime = Timer.getFPGATimestamp();
				}
				// If there's joystick input, the drive mode is set to tank
				if ((Math.abs(leftStick.getX()) > THRESHOLD) || (Math.abs(leftStick.getY()) > THRESHOLD)
						|| (Math.abs(rightStick.getX()) > THRESHOLD) || (Math.abs(rightStick.getY()) > THRESHOLD)) {
					driveState = 0;
				}
				// timeout
				if (Timer.getFPGATimestamp() - startTime > deltaTime) {
					driveState = 0;
				}

			}

			break;
		}
		// mecanum - second button press
		case 2: {
			if (driveMode == MECANUM_MODE) {
				if (rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
					driveState = 1;
					startTime = Timer.getFPGATimestamp();
				}
				// If there's joystick input, the drive mode is set to mecanum
				if ((Math.abs(leftStick.getX()) > THRESHOLD) || (Math.abs(leftStick.getY()) > THRESHOLD)
						|| (Math.abs(rightStick.getX()) > THRESHOLD) || (Math.abs(rightStick.getY()) > THRESHOLD)) {
					driveState = 4;
				}
				// timeout
				if (Timer.getFPGATimestamp() - startTime > deltaTime) {
					driveState = 4;
				}
				// tank mode - second button press
			} else {
				if (rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
					driveState = 3;
					startTime = Timer.getFPGATimestamp();
				}
				// If there's joystick input, the drive mode is set to tank
				if ((Math.abs(leftStick.getX()) > THRESHOLD) || (Math.abs(leftStick.getY()) > THRESHOLD)
						|| (Math.abs(rightStick.getX()) > THRESHOLD) || (Math.abs(rightStick.getY()) > THRESHOLD)) {
					driveState = 0;
				}
				// timeout
				if (Timer.getFPGATimestamp() - startTime > deltaTime) {
					driveState = 0;
				}
			}

			break;
		}
		// mecanum - first button press release
		case 3: {
			if (driveMode == MECANUM_MODE) {
				if (!rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
					driveState = 2;
					startTime = Timer.getFPGATimestamp();
				}
				// If there's joystick input, the drive mode is set to mecanum
				if ((Math.abs(leftStick.getX()) > THRESHOLD) || (Math.abs(leftStick.getY()) > THRESHOLD)
						|| (Math.abs(rightStick.getX()) > THRESHOLD) || (Math.abs(rightStick.getY()) > THRESHOLD)) {
					driveState = 4;
				}
				// timeout
				if (Timer.getFPGATimestamp() - startTime > deltaTime) {
					driveState = 4;
				}
				// tank mode - second button press release
			} else {
				if (!rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
					driveState = 4;
				}
				// If there's joystick input, the drive mode is set to tank
				if ((Math.abs(leftStick.getX()) > THRESHOLD) || (Math.abs(leftStick.getY()) > THRESHOLD)
						|| (Math.abs(rightStick.getX()) > THRESHOLD) || (Math.abs(rightStick.getY()) > THRESHOLD)) {
					driveState = 0;
				}
				// timeout
				if (Timer.getFPGATimestamp() - startTime > deltaTime) {
					driveState = 0;
				}
			}
			break;
		}
		// mecanum - first button press
		case 4: {
			robotSystem.ledMecanumMode(true);
			driveMode = MECANUM_MODE;
			if (rightStick.getRawButton(robotSystem.DRIVE_MODE_TRIGGER)) {
				driveState = 3;
				startTime = Timer.getFPGATimestamp();
			}
			break;
		}
		}
		//System.out.println("Drive Mode = " + driveMode + "Drive State = " + driveState);
		if (driveMode == MECANUM_MODE) {
			myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
			myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
			myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
			myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
			// team155Mecanum_fieldOriented(leftStick.getX(), leftStick.getY(),
			// rightStick.getX(), preTank);
			myrobot.mecanumDrive_Cartesian(leftStick.getX(), leftStick.getY(), -rightStick.getX(), 0);
			sol.set(DoubleSolenoid.Value.kReverse);
			preTank = false;

		} else {
			myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
			myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, false);
			myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
			myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
			myrobot.tankDrive((rightStick), (leftStick));
			sol.set(DoubleSolenoid.Value.kForward);
			preTank = true;
		}
		// System.out.println("motorscale is "+ motorScale);

		// this does a on-the-fly gyro init
		// hold both triggers for 5 seconds to make this work
		// robot must not be moving for those 5 seconds and the next 6 seconds

		prev_reset_trigger = reset_trigger;

		if ((leftStick.getRawButton(robotSystem.GYRORESET) == true)
				&& (rightStick.getRawButton(robotSystem.GYRORESET) == true))
			reset_trigger = true;
		else
			reset_trigger = false;

		if (reset_trigger && (!prev_reset_trigger))
			start_time = Timer.getFPGATimestamp();

		if (!reset_trigger)
			start_time = Timer.getFPGATimestamp();

		if ((Timer.getFPGATimestamp() - start_time) > 5.0) {
			System.out.println(
					"re-initializing the gyro..........................................................................");
			// roboGyro.initGyro();?????????????

			ahrs.reset();

			// roboGyro.calibrate();
			// roboGyro.reset();
			GyroReset();
			System.out.println(
					"done re-initializing gyro.........................................................................");
		}
		/*
		 * if (leftStick.getRawButton(robotSystem.ENCODER_RESET) == true)
		 * EncoderReset(); if (leftStick.getRawButton(robotSystem.PID_DISABLE)
		 * == true) PIDDisable();
		 */
		// DriveStraightDistance(36);
		// toteArm();

		/*
		 * lw.addActuator("Drive", "left_front", left_front);
		 * lw.addActuator("Drive", "right_front", right_front);
		 * lw.addActuator("Drive", "left_back", left_back);
		 * lw.addActuator("Drive", "right_back", right_back);
		 */
	}

	public boolean turnDrive(double setPointAngle) {
		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
		sol.set(DoubleSolenoid.Value.kReverse);

		prevCentered = centered;

		// is it centered?
		if (Math.abs(error) < .05) // may need to be upped
			centered = true;
		else {

			centered = false;
		}

		// rising edge detector
		if ((!prevCentered) && centered)
			atHeading = true;
		else
			atHeading = false; // was implied to be false

		error = -ahrs.getAngle() + setPointAngle;
		PIDoutput = error * Kp_Turn;

		// SmartDashboard.putNumber("heading setpoint is ", headingSetPoint);
		// SmartDashboard.putNumber("PIDoutput is ", PIDoutput);
		// SmartDashboard.putNumber("rightStick.getX is ", rightStick.getX());

		myrobot.mecanumDrive_Cartesian(0, 0, PIDoutput, ahrs.getAngle());
		// myrobot.mecanumDrive_Cartesian(LSgetX / motorScale, LSgetY /
		// motorScale, PIDoutput, roboGyro.getAngle());

		return atHeading;
	}

	void team155Mecanum_fieldOriented(double LSgetX, double LSgetY, double RSgetX, boolean wasTank) {

		prevCentered = centered;

		// is it centered?
		if (Math.abs(RSgetX) < .05) // may need to be upped
			centered = true;
		else {
			headingSetPoint = RSgetX * foo / motorScale + headingSetPoint; // must
																			// not
																			// be
			// centered,
			// so... command
			// to turn
			centered = false;
		}

		// rising edge detector
		if ((!prevCentered) && centered)
			holdHeading = true;
		else
			holdHeading = false; // was implied to be false

		if (wasTank)
			holdHeading = true;

		// on a rising edge, set the heading to hold
		if (holdHeading) {
			// headingSetPoint = roboGyro.getAngle();
			headingSetPoint = ahrs.getAngle();
			holdHeading = false;
		}

		// error = -roboGyro.getAngle() + headingSetPoint;
		error = -ahrs.getAngle() + headingSetPoint;
		PIDoutput = error * Kp_FieldOrientedControl;

		// SmartDashboard.putNumber("heading setpoint is ", headingSetPoint);
		// SmartDashboard.putNumber("PIDoutput is ", PIDoutput);
		// SmartDashboard.putNumber("rightStick.getX is ", rightStick.getX());

		myrobot.mecanumDrive_Cartesian(LSgetX / motorScale, LSgetY / motorScale, PIDoutput, ahrs.getAngle());
		// myrobot.mecanumDrive_Cartesian(LSgetX / motorScale, LSgetY /
		// motorScale, PIDoutput, roboGyro.getAngle());
	}

	public void EncoderReset() {
		left_front.setEncPosition(0);
		right_front.setEncPosition(0);
		left_back.setEncPosition(0);
		right_back.setEncPosition(0);
		left_front.setPosition(0);
		right_front.setPosition(0);
		left_back.setPosition(0);
		right_back.setPosition(0);

	}

	public double WallDistance() {

		double walldistance;
		
		walldistance = WALLSCALE*wallfinder.getVoltage();
		SmartDashboard.putNumber("Wallfinder voltage ", wallfinder.getVoltage());
		SmartDashboard.putNumber("wallDistance ", walldistance);
		return walldistance;
	}

	public boolean DriveStraightDistance(double distance) {
		left_front.enableBrakeMode(false);
		right_front.enableBrakeMode(false);
		left_back.enableBrakeMode(false);
		right_back.enableBrakeMode(false);

		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
		sol.set(DoubleSolenoid.Value.kForward);

		drivesetpoint = straightscale * distance;
		boolean onTarget;
		double speed;
		if (drivesetpoint > 0) {
			speed = -FORWARDSPEED;
		} else
			speed = FORWARDSPEED;
		/*
		 * left_front.set(setpoint); left_back.set(setpoint);
		 * right_front.set(setpoint); right_back.set(setpoint);
		 * 
		 */
		double distance_Back_Left;
		double distance_Front_Left;
		double distance_Back_Right;
		double distance_Front_Right;
		double averageDistance;
		distance_Front_Left = left_front.getEncPosition();
		distance_Back_Left = left_back.getEncPosition();
		// distance_Front_Right = right_front.getEncPosition();
		// distance_Back_Right = right_back.getEncPosition();

		averageDistance = (distance_Back_Left );//+ distance_Front_Left) / 2;

		System.out.println("setpoint2 = " + drivesetpoint);
		System.out.println("averageDistance = " + averageDistance);

		if (Math.abs(averageDistance) >= Math.abs(drivesetpoint)) {
			onTarget = true;
			team155Mecanum_fieldOriented(0, 0, 0, false);
			myrobot.mecanumDrive_Cartesian(0, 0, 0, ahrs.getAngle());
		} else {
			onTarget = false;
			team155Mecanum_fieldOriented(0, speed, 0, false);
		}

		return onTarget;
	}

	public boolean DriveSlideDistance(double distance) {
		left_front.enableBrakeMode(false);
		right_front.enableBrakeMode(false);
		left_back.enableBrakeMode(false);
		right_back.enableBrakeMode(false);

		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
		sol.set(DoubleSolenoid.Value.kReverse);

		double setpoint = slidescale * distance;
		boolean onTarget;
		double speed;
		if (distance > 0) {
			speed = -.75;
		} else
			speed = .75;
		/*
		 * left_front.set(setpoint); left_back.set(setpoint);
		 * right_front.set(setpoint); right_back.set(setpoint);
		 * 
		 */
		double distance_Back_Left;
		double distance_Front_Left;
		double distance_Back_Right;
		double distance_Front_Right;
		double averageDistance;
		distance_Front_Left = left_front.getEncPosition();
		distance_Back_Left = left_back.getEncPosition();
		// distance_Front_Right = right_front.getEncPosition();
		// distance_Back_Right = right_back.getEncPosition();

		averageDistance = (distance_Back_Left);// + -distance_Front_Left) / 2;

		if (Math.abs(averageDistance) >= Math.abs(setpoint)) {
			onTarget = true;
			team155Mecanum_fieldOriented(0, 0, 0, false);
			myrobot.mecanumDrive_Cartesian(0, 0, 0, ahrs.getAngle());
		} else {
			onTarget = false;
			team155Mecanum_fieldOriented(speed, 0, 0, false);
		}

		return onTarget;
	}

	public boolean DriveStraightDistanceTime(double distance) {
		left_front.enableBrakeMode(false);
		right_front.enableBrakeMode(false);
		left_back.enableBrakeMode(false);
		right_back.enableBrakeMode(false);

		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
		sol.set(DoubleSolenoid.Value.kForward);

		System.out.println("starttime = " + starttime);
		System.out.println("timer = " + timer);
		System.out.println("drivesetpointtime = " + drivesetpointtime);
		System.out.println("Timer.getFPGATimestamp( = " + Timer.getFPGATimestamp());

		if (starttime) {
			timer = Timer.getFPGATimestamp();
			starttime = false;
		}
		drivesetpointtime = straighttimescale * distance;
		boolean onTarget;
		double speed;

		if (distance > 0) {
			speed = -FORWARDSPEED;

		} else {
			speed = FORWARDSPEED;
			drivesetpointtime = -drivesetpointtime;
		}

		System.out.println("speed = " + speed);
		if (Timer.getFPGATimestamp() - timer > drivesetpointtime) {
			onTarget = true;
			//team155Mecanum_fieldOriented(0, 0, 0, false);
			//myrobot.mecanumDrive_Cartesian(0, 0, 0, ahrs.getAngle());
			starttime = true;
			speed =0;

		} else {
			onTarget = false;
			//team155Mecanum_fieldOriented(0, speed, 0, false);
			//myrobot.mecanumDrive_Cartesian(0, speed, 0, ahrs.getAngle());
			speed =speed;
			
		}

		team155Mecanum_fieldOriented(0, speed, 0, false);
		return onTarget;
	}

	public boolean DriveSlideDistanceTime(double distance) {
		left_front.enableBrakeMode(false);
		right_front.enableBrakeMode(false);
		left_back.enableBrakeMode(false);
		right_back.enableBrakeMode(false);

		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, false);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
		myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
		sol.set(DoubleSolenoid.Value.kReverse);

		if (starttime) {
			timer = Timer.getFPGATimestamp();
			starttime = false;
		}

		double setpointtime = slidescaletime * distance;
		boolean onTarget;
		double speed;
		if (distance > 0) {
			speed = -.75;
		} else {
			speed = .75;
			setpointtime = -setpointtime;
		}
		/*
		 * left_front.set(setpoint); left_back.set(setpoint);
		 * right_front.set(setpoint); right_back.set(setpoint);
		 * 
		 */
		double distance_Back_Left;
		double distance_Front_Left;
		double distance_Back_Right;
		double distance_Front_Right;
		double averageDistance;
		distance_Front_Left = left_front.getEncPosition();
		distance_Back_Left = left_back.getEncPosition();
		// distance_Front_Right = right_front.getEncPosition();
		// distance_Back_Right = right_back.getEncPosition();

		averageDistance = (distance_Back_Left + -distance_Front_Left) / 2;

		if (Timer.getFPGATimestamp() - timer > setpointtime) {
			onTarget = true;
			team155Mecanum_fieldOriented(0, 0, 0, false);
			myrobot.mecanumDrive_Cartesian(0, 0, 0, ahrs.getAngle());
			starttime = true;
		} else {
			onTarget = false;
			team155Mecanum_fieldOriented(speed, 0, 0, false);
		}

		return onTarget;
	}

	public void PIDEnable() {
		left_front.enable();
		right_front.enable();
		left_back.enable();
		right_back.enable();
	}

	public void PIDDisable() {
		left_front.disable();
		right_front.disable();
		left_back.disable();
		right_back.disable();
	}

	public void driveStop() {
		left_front.set(0);
		right_front.set(0);
		left_back.set(0);
		right_back.set(0);
	}

	public double EncoderDistance() {
		double distance_Back_Left;
		double distance_Front_Left;
		double distance_Back_Right;
		double distance_Front_Right;
		double averageDistance;
		// System.out.println("in EncoderDistance ");
		distance_Front_Left = left_front.getEncPosition();
		distance_Back_Left = left_back.getEncPosition();
		distance_Front_Right = right_front.getEncPosition();
		distance_Back_Right = right_back.getEncPosition();

		// System.out.println("distance_Front_Left = " + distance_Front_Left);
		// System.out.println("distance_Back_Left = " + distance_Back_Left);
		// System.out.println("distance_Front_Right = " + distance_Front_Right);
		// System.out.println("distance_Back_Right = " + distance_Back_Right);

		// averageDistance = (distance_Front_Left + distance_Back_Left +
		// distance_Front_Right + distance_Back_Right) / 4;
		averageDistance = (distance_Back_Left );//+ distance_Front_Left) / 2;
		// System.out.println("averageDistance = " + averageDistance);
		double DistanceConvert = averageDistance / straightscale;
		double SlideConvert = averageDistance / slidescale;
		//SmartDashboard.putNumber("Average of left side encoder : ", averageDistance);
		//SmartDashboard.putNumber("Back left Encoder Distance2 : ", distance_Back_Left);
		//SmartDashboard.putNumber("Front left Encoder Distance2 : ", distance_Front_Left);

		//SmartDashboard.putNumber("Back Right Encoder Distance2 : ", distance_Back_Right);
		//SmartDashboard.putNumber("Front Right Encoder Distance2 : ", distance_Front_Right);
		//SmartDashboard.putNumber("Straight distance: ", DistanceConvert);
		//SmartDashboard.putNumber("Slide Distance", SlideConvert);

		// SmartDashboard.putNumber("Back Right Encoder Distance2 : ",
		// distance_Back_Right);
		// SmartDashboard.putNumber("Front Right Encoder Distance2 : ",
		// distance_Front_Right);

		return averageDistance;
	}

	// Enable Disable Controller Buttons (Lights on or off according to the
	// robot)

}
