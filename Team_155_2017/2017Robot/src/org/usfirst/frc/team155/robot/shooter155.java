package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.AnalogInput;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

public class shooter155 {

	robotMap155 robotSystem;
	PixyVision155 camera;
	ShooterTarget shooterStuff;

	SmartDashboard sDash;
	LiveWindow lw;
	// MOTORS FOR COMPEITION ROBOT

	// JOYSTICKS
	Joystick leftStick;
	Joystick rightStick;
	Joystick dsStick;
	// MOTORS FOR COMPEITION ROBOT
	public CANTalon shooter;
	public CANTalon shooterB;
	public VictorSP indexer;
	public Spark shaker;
	public Servo angler;
	public AnalogInput rangefinder;
	public Relay aimlight;

	public double INDEXSPEED = -1;
	public double SHAKERSPEED = -.6;
	public double STOP = 0;

	public double ANGLE1 = .32; // 52degrees
	public double ANGLE2 = .35; // 60
	public double ANGLE3 = .38; // 68
	public double ANGLE3_5 = .4;
	public double ANGLE4 = .45; // 76
	public double ANGLE5 = .55; // 82degrees

	public double AXIS1 = -.75; // 52degrees
	public double AXIS2 = -.5; // 60
	public double AXIS3 = -.25; // 68
	public double AXIS4 = .33; // 76
	public double AXIS5 = .66; // 82degrees

	public double SLOPE1 = 10;
	public double SLOPE2 = 20;
	public double SLOPE3 = 30;
	public double SLOPE4 = 40;
	public double SLOPE5 = 50;

	public double INT1 = .1;
	public double INT2 = .2;
	public double INT3 = .3;
	public double INT4 = .4;
	public double INT5 = .5;

	public double RANGELOW = 10;
	public double RANGE1 = 20;
	public double RANGE2 = 30;
	public double RANGEMAX = 40;

	// DRIVE ENCODERS
	public CANTalon shooter_Encoder;
	public double distance;
	public boolean targetSeen;
	public boolean readyFire;
	public boolean uptoSpeed;
	public boolean atAngle;
	public double setAngle = ANGLE5;
	public double setSlope;
	public double setInt;
	public double targetSpeed;
	public boolean readytoFire = false;
	public double currentSpeed;
	public double tol = 10;
	public boolean readytoFireLED = false;

	public boolean Shooter_On = false;

	public final int STOPFIRE = 0;
	public final int LOOKFORTARGET = 1;
	public final int PREPFIRE = 2;
	public final int READYTOFIRE = 3;

	public final int HOLD = 0;
	public final int PREFIRE = 1;
	public final int FIRE = 2;

	public boolean shootermode = false;
	public double shooterpower = 0;

	public int state = 0;
	public int shooterstate = 0;

	boolean atSpeed;
	boolean fireReady;

	double SetSpeed;
	double SetAngle;

	double HIGHGOAL = 3;
	double LOWGOAL = 1.5;

	double startTimeShooter;
	double motorTime;
	final double LONG = 4;
	final double SHORT = 5;


	public shooter155(robotMap155 robot, PixyVision155 cameraSystem) {
		robotSystem = robot;
		camera = cameraSystem;
		leftStick = new Joystick(robotSystem.LEFTSTICK);
		rightStick = new Joystick(robotSystem.RIGHTSTICK);
		dsStick = new Joystick(robotSystem.DSSTICK);
		shooter = new CANTalon(robotSystem.SHOOTER_A);
		shooterB = new CANTalon(robotSystem.SHOOTER_B);
		indexer = new VictorSP(robotSystem.INDEXER);
		angler = new Servo(robotSystem.SHOOTERANGLE);
		lw = new LiveWindow();
		sDash = new SmartDashboard();
		shaker = new Spark(robotSystem.SHAKER);
		rangefinder = new AnalogInput(robotSystem.SHOOTRANGE_FINDER);
		aimlight = new Relay(robotSystem.RELAY_1);

		StringBuilder _sb = new StringBuilder();



		/* first choose the sensor */
		shooter.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		shooter.reverseSensor(false);

		shooterB.changeControlMode(CANTalon.TalonControlMode.Follower);
		shooterB.set(robotSystem.SHOOTER_A);
		// shooter.configEncoderCodesPerRev(XXX), // if using
		// FeedbackDevice.QuadEncoder
		// shooter.configPotentiometerTurns(XXX), // if using
		// FeedbackDevice.AnalogEncoder or AnalogPot

		/* set the peak and nominal outputs, 12V means full */
		shooter.configNominalOutputVoltage(+0.0f, -0.0f);
		shooter.configPeakOutputVoltage(+12.0f, -12.0f);

		/* set closed loop gains in slot0 */
		shooter.setProfile(0);
		shooter.setF(0.1097);
		shooter.setP(0.22);
		shooter.setI(0);
		shooter.setD(0);

		// shooterMotor =
		// shooter_Encoder = shooterMotor.getSpeed() ;

		// If shooter button pushed,lineup to target and then get distance
		// d = Target height ft*FOV height pixel/(2*Target height pixel*tan) -
		// FOV = 75 vert X 47 hor
		// d = 4in/12*200/(2*THp*16.12)

	}

	public void anglerInit() {
		setAngle = ANGLE5;
		angler.set(setAngle);

	}

	public double rangeFinder() {
		double scale = 1;
		double distance = scale * rangefinder.getAverageVoltage();

		return distance;
	}

	public void testSpeed() {
		shooterB.changeControlMode(CANTalon.TalonControlMode.Follower);
		shooterB.set(robotSystem.SHOOTER_A);
		/* get gamepad axis */
		double rightZstick = rightStick.getAxis(AxisType.kZ);
		double motorOutput = shooter.getOutputVoltage() / shooter.getBusVoltage();
		shooterpower = -(.5 * rightStick.getAxis(AxisType.kZ) - .5);

		if (rightStick.getRawButton(5)) {
			/* Speed mode */
			double targetSpeed = shooterpower
					* 1500.0; /* 1500 RPM in either direction */
			shooter.changeControlMode(TalonControlMode.Speed);

			shooter.set(targetSpeed); /* 1500 RPM in either direction */

		} else {
			/* Percent voltage mode */
			shooter.changeControlMode(TalonControlMode.PercentVbus);

			shooter.set(shooterpower);
		}

	}

	public void Aimlight() {
		boolean light;
		if (rightStick.getRawButton(robotSystem.AIMLIGHT_On)) {
			aimlight.set(Relay.Value.kForward);

			light = true;
		} else {
			aimlight.set(Relay.Value.kOff);
			light = false;
		}
		sDash.putBoolean("AimLight", light);
	}

	public void dumbrun() {
		Aimlight();
		shooterB.changeControlMode(CANTalon.TalonControlMode.Follower);
		shooterB.set(robotSystem.SHOOTER_A);

		double dsZstick = dsStick.getAxis(AxisType.kZ);
		double motorOutput = shooter.getOutputVoltage() / shooter.getBusVoltage();
		double dsYstick = dsStick.getAxis(AxisType.kY);
		double RightZstick = rightStick.getAxis(AxisType.kZ);
		/* prepare line to print */
		// sDash.putNumber("motorOutput is ", motorOutput);
		// sDash.putNumber("Speed is ", shooter.getSpeed());
		sDash.putNumber("DS Zaxis ", dsZstick);

		sDash.putNumber("DS Yaxis ", dsYstick);
		sDash.putNumber("Right Zaxis ", RightZstick);
		// sDash.putNumber("angler", setAngle);
		// sDash.putBoolean("Shootermode", shootermode);

		// sDash.putNumber("AutoState", state);
		// sDash.putNumber("Manual State", shooterstate);
		/*
		 * shooter.changeControlMode(TalonControlMode.PercentVbus);
		 * 
		 * shooter.set(.5*rightStick.getAxis(AxisType.kZ)-.5);
		 */
		/*
		 * if (dsStick.getRawButton(robotSystem.ANGLEUP)) { setAngle = ANGLE5; }
		 * if (dsStick.getRawButton(robotSystem.ANGLEDOWN)) { setAngle = ANGLE1;
		 * }
		 * 
		 * //setAngle = leftStick.getAxis(AxisType.kZ); angler.set(setAngle);
		 */
		// runShaker();

		dumbangle();
		if (rightStick.getRawButton(robotSystem.SHOOTER_ON)) {

			Shooter_On = true;

		}

		if (leftStick.getRawButton(robotSystem.SHOOTER_OFF)) {
			Shooter_On = false;

		}

		if (Shooter_On) {
			// runShaker();
			indexShooter();
			setShooter(dsZstick);

		} else {
			stopShaker();
			stopShooter();
			// angler.set(ANGLE1);
			shooterstate = HOLD;
			startTimeShooter = Timer.getFPGATimestamp();

		}

		/*
		 * if (!dsStick.getRawButton(robotSystem.SHOOTMODE)) {
		 * 
		 * 
		 * 
		 * automaticShooter(); shootermode = false; } else { manualShooter();
		 * shootermode = true; }
		 */
		// lw.addActuator("Shooter", "Angle", angler);
		// lw.addActuator("Shooter", "Shooter", shooter);
		// lw.addActuator("Shooter", "indexer", indexer);

	}

	public void dumbangle() {
		/*
		 * public double ANGLE1 = .32; // 52degrees public double ANGLE2 = .35;
		 * // 60 public double ANGLE3 = .38; // 68 public double ANGLE3_5 = .4;
		 * public double ANGLE4 = .45; // 76 public double ANGLE5 = .55; //
		 * 82degrees
		 * 
		 * public double AXIS1 = -.75; // 52degrees public double AXIS2 = -.5;
		 * // 60 public double AXIS3 = -.25; // 68 public double AXIS4 = .33; //
		 * 76 public double AXIS5 = .66; // 82degrees
		 */
		if (dsStick.getAxis(AxisType.kY) < AXIS1) {
			setAngle = ANGLE1;
		} else if ((dsStick.getAxis(AxisType.kY)) >= -2 && (dsStick.getAxis(AxisType.kY) < AXIS2)) {
			SetAngle = ANGLE1;
		} else if ((dsStick.getAxis(AxisType.kY)) >= AXIS1 && (dsStick.getAxis(AxisType.kY) < AXIS2)) {
			SetAngle = ANGLE2;
		} else if ((dsStick.getAxis(AxisType.kY)) >= AXIS2 && (dsStick.getAxis(AxisType.kY) < AXIS3)) {
			SetAngle = ANGLE3;
		} else if ((dsStick.getAxis(AxisType.kY)) >= AXIS3 && (dsStick.getAxis(AxisType.kY) < AXIS4)) {
			SetAngle = ANGLE3_5;
		} else if ((dsStick.getAxis(AxisType.kY)) >= AXIS4 && (dsStick.getAxis(AxisType.kY) < AXIS5)) {
			SetAngle = ANGLE4;
		} else if ((dsStick.getAxis(AxisType.kY)) >= AXIS5 && (dsStick.getAxis(AxisType.kY) < 2)) {
			SetAngle = ANGLE5;
		} else
			SetAngle = SetAngle;

		double dsAngle = dsStick.getAxis(AxisType.kY);

		/* prepare line to print */
		// sDash.putNumber("motorOutput is ", motorOutput);
		// sDash.putNumber("Speed is ", shooter.getSpeed());
		sDash.putNumber("DS Angle ", dsAngle);
		sDash.putNumber("angler", SetAngle);
		angler.set(SetAngle);
	}

	public void run() {
		Aimlight();
		shooterB.changeControlMode(CANTalon.TalonControlMode.Follower);
		shooterB.set(robotSystem.SHOOTER_A);

		double rightZstick = rightStick.getAxis(AxisType.kZ);
		double motorOutput = shooter.getOutputVoltage() / shooter.getBusVoltage();
		/* prepare line to print */
		// sDash.putNumber("motorOutput is ", motorOutput);
		// sDash.putNumber("Speed is ", shooter.getSpeed());
		// sDash.putNumber("R Zaxis ", rightZstick);
		// sDash.putNumber("angler", setAngle);
		// sDash.putBoolean("Shootermode", shootermode);

		// sDash.putNumber("AutoState", state);
		// sDash.putNumber("Manual State", shooterstate);
		/*
		 * shooter.changeControlMode(TalonControlMode.PercentVbus);
		 * 
		 * shooter.set(.5*rightStick.getAxis(AxisType.kZ)-.5);
		 */
		/*
		 * if (dsStick.getRawButton(robotSystem.ANGLEUP)) { setAngle = ANGLE5; }
		 * if (dsStick.getRawButton(robotSystem.ANGLEDOWN)) { setAngle = ANGLE1;
		 * }
		 * 
		 * //setAngle = leftStick.getAxis(AxisType.kZ); angler.set(setAngle);
		 */

		if (leftStick.getRawButton(robotSystem.SHOOTER_ON))
			Shooter_On = true;
		if (rightStick.getRawButton(robotSystem.SHOOTER_OFF))
			Shooter_On = false;

		if (Shooter_On) {
			// runShaker();
			indexShooter();
			if ((leftStick.getRawButton(robotSystem.SHOOTCAM)))// &&(camera.getGearTarget())
			{
				/*-
					atSpeed =AutoSet(cameradistance);
				aligned = align with target;
				if (atSpeed && aligned)
					readytoFire=true
				else readytoFire=false
				*/
			} else if ((rightStick.getRawButton(robotSystem.SHOOTRANGE)))// &&(camera.getGearTarget())
			{
				atSpeed = setShooter(rangeFinder());
				fireReady = atSpeed;
			} else
				shooterManual();

		} else {
			stopShaker();
			stopShooter();
			angler.set(ANGLE1);
		}

		/*
		 * if (!dsStick.getRawButton(robotSystem.SHOOTMODE)) {
		 * 
		 * 
		 * 
		 * automaticShooter(); shootermode = false; } else { manualShooter();
		 * shootermode = true; }
		 */
		// lw.addActuator("Shooter", "Angle", angler);
		// lw.addActuator("Shooter", "Shooter", shooter);
		// lw.addActuator("Shooter", "indexer", indexer);

	}

	public void shooterManual() {

		if (dsStick.getRawButton(robotSystem.SHOOTMODE)) {
			SetSpeed = rightStick.getAxis(AxisType.kZ);
			if (leftStick.getRawButton(robotSystem.VARANGLE))
				setAngle = leftStick.getAxis(AxisType.kZ);
			else if (dsStick.getRawButton(robotSystem.ANGLEUP)) {
				SetAngle = ANGLE5;
			} else if (dsStick.getRawButton(robotSystem.ANGLEDOWN)) {
				SetAngle = ANGLE1;
			} else if (leftStick.getRawButton(robotSystem.ANGLE_2)) {
				SetAngle = ANGLE2;
			}

			else if (leftStick.getRawButton(robotSystem.ANGLE_3)) {
				SetAngle = ANGLE3;
			} else if (leftStick.getRawButton(robotSystem.ANGLE_4)) {
				SetAngle = ANGLE4;
			} else
				SetAngle = SetAngle;
		} else {

			if (dsStick.getRawButton(robotSystem.ANGLEUP)) {
				SetAngle = ANGLE5;
				SetSpeed = HIGHGOAL;
			} else if (dsStick.getRawButton(robotSystem.ANGLEDOWN)) {
				SetAngle = ANGLE1;
				SetSpeed = LOWGOAL;
			}
		}
		setShooter(SetSpeed);
		angler.set(SetAngle);
	}

	public boolean setSpeedAngleOnDistance(double distance) {
		if (distance >= RANGEMAX) {
			setAngle = ANGLE1;
			setInt = INT1;
			setSlope = SLOPE1;

		} else if (distance > RANGELOW && distance <= RANGE1) {
			setAngle = ANGLE4;
			setInt = INT4;
			setSlope = SLOPE4;
		} else if (distance > RANGE1 && distance <= RANGE2) {
			setAngle = ANGLE3;
			setInt = INT3;
			setSlope = SLOPE3;
		} else if (distance > RANGE2 && distance <= RANGEMAX) {
			setAngle = ANGLE2;
			setInt = INT2;
			setSlope = SLOPE2;
		} else {
			setAngle = ANGLE5;
			setInt = INT5;
			setSlope = SLOPE5;
		}

		angler.set(setAngle);
		targetSpeed = setSlope * distance + setInt;
		uptoSpeed = setShooter(targetSpeed);

		return uptoSpeed;
	}

	public void automaticShooter() {
		camera.getShooterTarget(); // find gear target

		double distance = shooterStuff.distance;
		double boilerAngle = shooterStuff.angle;

		switch (state) {

		case STOPFIRE: {// Driving around don't care about shooting. Stop motors
			if (leftStick.getRawButton(robotSystem.FIRE)) {
				state = LOOKFORTARGET;
			}
		}

		case LOOKFORTARGET: {// Driver Press button to go into shooting mode can
								// drive until a target is seen
			if (!leftStick.getRawButton(robotSystem.FIRE)) {
				state = STOPFIRE;
			}
			if (targetSeen) {
				state = PREPFIRE;
			}

		}
		case PREPFIRE: {// Locks out Driver moves robot to location and sets
						// motorspeeds and angles. fires when ready
			// Sets Constants for Angle and Target Speeds

			setSpeedAngleOnDistance(distance);

			if (setSpeedAngleOnDistance(distance)) {
				indexer.set(INDEXSPEED);
			}

			if (!leftStick.getRawButton(robotSystem.FIRE)) {
				state = STOPFIRE;
			}
		}

		}

	}// ends function

	public void setAngle(double angle) {
		angler.set(angle);
	}

	public void runIndexer(double speed) {
		indexer.set(speed);
		// sDash.putNumber("Right Zaxis ", rightStick.getAxis(AxisType.kZ));

	}

	public void stopIndexer() {
		indexer.set(STOP);
	}

	public void runShaker() {
		shaker.set(SHAKERSPEED);
		// sDash.putNumber("Shacker speed", rightStick.getAxis(AxisType.kZ));
	}

	public void stopShaker() {
		shaker.set(STOP);
	}

	public boolean setShooter(double speed) {

		shooter.changeControlMode(TalonControlMode.PercentVbus);
		shooterpower = .5 * speed - .5;

		shooter.set(-shooterpower);

		/*
		 * shooter.changeControlMode(TalonControlMode.Speed);
		 * shooter.set(speed); currentSpeed = shooter.getEncVelocity();
		 * 
		 * if ((currentSpeed >= speed - tol) && (currentSpeed <= speed + tol))
		 * readytoFire = true; else readytoFire = false;
		 */
		readytoFire = true;
		return readytoFire;

	}

	public void stopShooter() {

		shooter.changeControlMode(TalonControlMode.PercentVbus);

		shooter.set(0);

	}

	public void fullShooter() {

		shooter.changeControlMode(TalonControlMode.PercentVbus);
		shooterpower = .5 * rightStick.getAxis(AxisType.kZ) - .5;

		shooter.set(shooterpower);

	}

	public void indexShooter() {

		switch (shooterstate) {
		case HOLD:
			indexer.set(STOP);
			stopShaker();

			if (Timer.getFPGATimestamp() - startTimeShooter > 2) {
				startTimeShooter = Timer.getFPGATimestamp();
				shooterstate = PREFIRE;
				motorTime = LONG;
			}
			readytoFireLED = false;
			break;

		case PREFIRE:
			stopShaker();
			readytoFireLED = true;
			// Check if ready to fire(Motor up to speed)
			if (leftStick.getRawButton(robotSystem.FIRE) && Timer.getFPGATimestamp() - startTimeShooter > .25) {
				shooterstate = FIRE;
				startTimeShooter = Timer.getFPGATimestamp();
			}

			if (rightStick.getRawButton(robotSystem.SHOOTER_OFF))
				shooterstate = HOLD;

			indexer.set(STOP);

			break;

		case FIRE:
			if (Timer.getFPGATimestamp() - startTimeShooter > SHORT) {
				shooterstate = PREFIRE;
				startTimeShooter = Timer.getFPGATimestamp();

			}
			indexer.set(INDEXSPEED);

			runShaker();
			break;

		}
	}

	public void manualShooter() {

		switch (shooterstate) {
		case HOLD:
			indexer.set(STOP);
			stopShooter();
			stopShaker();

			if (leftStick.getRawButton(robotSystem.SHOOTER_ON)) {
				startTimeShooter = Timer.getFPGATimestamp();
				shooterstate = PREFIRE;
				motorTime = LONG;
			}
			break;

		case PREFIRE:
			fullShooter();
			runShaker();
			// Check if ready to fire(Motor up to speed)
			if (leftStick.getRawButton(robotSystem.FIRE) && Timer.getFPGATimestamp() - startTimeShooter > motorTime) {
				shooterstate = FIRE;
				startTimeShooter = Timer.getFPGATimestamp();
			}

			if (rightStick.getRawButton(robotSystem.SHOOTER_OFF))
				shooterstate = HOLD;

			indexer.set(STOP);

			break;

		case FIRE:
			if (Timer.getFPGATimestamp() - startTimeShooter > SHORT) {
				shooterstate = PREFIRE;
				startTimeShooter = Timer.getFPGATimestamp();
				motorTime = SHORT + .5;

			}
			indexer.set(INDEXSPEED);
			fullShooter();
			runShaker();
			break;

		}

		// fullShooter();

		if (dsStick.getRawButton(robotSystem.ANGLEUP)) {
			setAngle = ANGLE5;
		}
		if (dsStick.getRawButton(robotSystem.ANGLEDOWN)) {
			setAngle = ANGLE1;
		}

		angler.set(setAngle);

		if (!rightStick.getRawButton(robotSystem.SHOOTER_OFF)) {
			state = HOLD;
		}

	}



}
