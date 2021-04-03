package org.usfirst.frc.team155.robot;

import org.usfirst.frc.team155.robot.PixyPacket;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String defaultAuto = "Straight";
	final String leftAuto = "Left Side";
	final String rightAuto = "Right Side";

	String autoSelected;

	SendableChooser AutoMode;
	SendableChooser Poschooser;
	SendableChooser Gearchooser;
	SendableChooser Linechooser;
	SendableChooser AutoShootchooser;
	SendableChooser AutoGearchooser;
	SendableChooser waitchooser;
	SendableChooser Colorchooser;

	double startTimeShooter = 0;

	private int mode = 1;
	private int modename = 0;

	private int Posmode = 1;
	private int Posmodename = 0;
	private int Gearmode = 1;
	private int Gearmodename = 0;
	boolean donedrive =true;

	private int AutoShootmode = 1;
	private int AutoShootmodename = 0;
	private int colormode = 1;
	private int colormodename = 0;

	private int AutoGearmode = 1;
	private int AutoGearmodename = 0;

	private int waitmode = 1;
	private int waitmodename = 0;
	public double waittime = 0;

	public Drive155 drive;
	// public DrivePrac155 driveP;
	public gear155 gear;
	public shooter155 shooter;
	public climber155 climber;
	//public ballgather155 gatherer;
	public robotMap155 robotSystem;
	public Auto155 robotAuto;
	// public PixyCamBasic pixyCam;
	// public pixyCamVision pixyCam;
	// public PixyShooter pixyShoot;
	// public pixyGear pixyGear;
	public PixyVision155 pixyVision;
	public GearTarget pixyGear;
	//public LED155 LEDs;

	public Servo angler;
	/*
	 * public CANTalon shootermotor = new CANTalon(robotSystem.SHOOTER); public
	 * VictorSP indexer = new VictorSP(robotSystem.INDEXER);
	 * 
	 * public DoubleSolenoid gearSol= new DoubleSolenoid(robotSystem.GEAR_SOL_A,
	 * robotSystem.GEAR_SOL_B); public DigitalInput gearSwitch1= new
	 * DigitalInput(robotSystem.GEAR_SWITCH1); public DigitalInput gearSwitch2=
	 * new DigitalInput(robotSystem.GEAR_SWITCH2); public CANTalon gatherMotor =
	 * new CANTalon(robotSystem.GATHERER); public VictorSP climbMotor1= new
	 * VictorSP(robotSystem.CLIMBER1); public VictorSP climbMotor2= new
	 * VictorSP(robotSystem.CLIMBER2);
	 * 
	 * public CANTalon left_front = new CANTalon(robotSystem.DRIVE_LEFT_FRONT);
	 * public CANTalon right_front = new
	 * CANTalon(robotSystem.DRIVE_RIGHT_FRONT); public CANTalon left_back= new
	 * CANTalon(robotSystem.DRIVE_LEFT_BACK); public CANTalon right_back = new
	 * CANTalon(robotSystem.DRIVE_RIGHT_BACK);
	 */
	public SmartDashboard sDash = new SmartDashboard();
	public LiveWindow lw = new LiveWindow();

	Joystick translateStick = new Joystick(0);
	Joystick rotateStick = new Joystick(1);
	Joystick shooterStick = new Joystick(2);

	// public Victor shooter;
	public double speed;
	public int hold;
	public int counter;
	double distance;
	double angle;
	boolean didWeFindTarget;

	int i, j;
	int returnVal;
	PixyPacket[] pixyBlock = new PixyPacket[16];

	// public boolean holdPrevious;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// pixyCam = new PixyCamBasic();
		// pixyCam = new pixyCamVision();

		CameraServer.getInstance().startAutomaticCapture();
		AutoMode = new SendableChooser();
		Poschooser = new SendableChooser();
		Gearchooser = new SendableChooser();
		//Linechooser = new SendableChooser();
		AutoShootchooser = new SendableChooser();
		Gearchooser = new SendableChooser();
		Colorchooser = new SendableChooser();

		//waitchooser = new SendableChooser();

		AutoMode.addDefault("Line Only", 0);
		AutoMode.addObject("Line Backwards Only", 1);
		AutoMode.addObject("Short Shoot Blue", 2);
		AutoMode.addObject("Left Gear Mode", 3);
		AutoMode.addObject("Short Shoot Red", 4);
		AutoMode.addObject("Right Gear Mode", 5);
		AutoMode.addObject("Center Gear Mode", 6);

		Colorchooser.addDefault("Blue", 0);
		Colorchooser.addObject("Red", 1); // move

		Poschooser.addDefault("Left", 0);
		Poschooser.addObject("Center", 1); // move
		Poschooser.addObject("Right", 2);

		Gearchooser.addDefault(" Gear Yes", 0);
		Gearchooser.addObject("Gear No", 1); // move

		AutoShootchooser.addDefault("Shoot Yes", 0);
		AutoShootchooser.addObject("Shoot No", 1); // move

		// AutoGearchooser.addDefault("Auto Gear Yes", 0);
		// AutoGearchooser.addObject("No", 1); // move

		/*
		 * chooser.addDefault("default, do nothing ", 0);
		 * chooser.addObject("Shoot Left", 1); // move
		 * chooser.addObject("Shoot, Move to Gear Left", 2);
		 * chooser.addObject("Shoot, Move to Gear Center", 3);
		 * chooser.addObject("Shoot, Move to Gear Right", 4);
		 * chooser.addObject("Shoot, Move to Gear Left, Overline", 5);
		 * chooser.addObject("Shoot, Move to Gear Right, Overline", 6);
		 * chooser.addObject("Shoot, Move to Gear Center, Overline", 7);
		 * chooser.addObject("Shoot from Left, Move Overline", 8);
		 * chooser.addObject("Shoot from Right, Move Overline", 9);
		 * chooser.addObject("Shoot Center", 10); // move
		 * chooser.addObject("Shoot Right", 11); // move
		 */
		/*
		waitchooser = new SendableChooser();

		waitchooser.addDefault("No Wait", 0);
		waitchooser.addObject("0.1 sec", 1);
		waitchooser.addObject("0.25 sec", 2);
		waitchooser.addObject("0.5 sec", 3);
		waitchooser.addObject("0.75 sec", 4);
		waitchooser.addObject("1 sec", 5);
		waitchooser.addObject("1.5 sec", 6);
*/
		// SmartDashboard.putData("AutoChooser", chooser);
		sDash.putData("AutoChooser", AutoMode);
		sDash.putData("Color", Colorchooser);
		//sDash.putData("Wait choices", waitchooser);
		sDash.putData("Start Position", Poschooser);
		sDash.putData("AutoAim", AutoShootchooser);
		//sDash.putData("AutoGear", Gearchooser);
		sDash.putData("Try Gear", Gearchooser);
		//sDash.putData("Go OverLine", Linechooser);

		// shooter = new Victor(4);
		speed = 0;
		hold = 0;

		robotSystem = new robotMap155();
		pixyVision = new PixyVision155();

		// driveP = new DrivePrac155(robotSystem, pixyVision);

		drive = new Drive155(robotSystem, pixyVision);

		shooter = new shooter155(robotSystem, pixyVision);
		gear = new gear155(robotSystem);
		climber = new climber155(robotSystem);
		//gatherer = new ballgather155(robotSystem);
		//LEDs = new LED155(robotSystem, drive, shooter, gear, climber, pixyVision);
		shooter.anglerInit();
		robotAuto = new Auto155(robotSystem, drive, shooter, gear, pixyVision);
		drive.GyroReset();

		//LEDs.setColorMode(3);
		// angler = new Servo(robotSystem.SHOOTERANGLE);
		/*
		 * lw = new LiveWindow(); sDash = new SmartDashboard();
		 * 
		 * 
		 * left_front = new CANTalon(robotSystem.DRIVE_LEFT_FRONT); right_front=
		 * new CANTalon(robotSystem.DRIVE_RIGHT_FRONT); left_back = new
		 * CANTalon(robotSystem.DRIVE_LEFT_BACK); right_back = new
		 * CANTalon(robotSystem.DRIVE_RIGHT_BACK);
		 * 
		 * shootermotor = new CANTalon(robotSystem.SHOOTER); indexer = new
		 * VictorSP(robotSystem.INDEXER);
		 * 
		 * 
		 * gatherMotor = new CANTalon(robotSystem.GATHERER);
		 * 
		 * gearSwitch1 = new DigitalInput(robotSystem.GEAR_SWITCH1); gearSwitch2
		 * = new DigitalInput(robotSystem.GEAR_SWITCH2); gearSol = new
		 * DoubleSolenoid(robotSystem.GEAR_SOL_A, robotSystem.GEAR_SOL_B);
		 * 
		 * climbMotor1 = new VictorSP(robotSystem.CLIMBER1); climbMotor2 = new
		 * VictorSP(robotSystem.CLIMBER2);
		 */

	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		shooter.anglerInit();
		// drive.GyroReset();
		// drive.mecanumstop();
		mode = (int) AutoMode.getSelected();
		Posmode = (int) Poschooser.getSelected();
		Gearmode = (int) Gearchooser.getSelected();

		AutoShootmode = (int) AutoShootchooser.getSelected();
		AutoGearmode = (int) Gearchooser.getSelected();
		colormode = (int) Colorchooser.getSelected();
		// waitmode = (int) waitchooser.getSelected();

		startTimeShooter = Timer.getFPGATimestamp();
		// drive.roboGyro.reset();
		counter = 0;

		didWeFindTarget = false;
		for (int i = 0; i < 10; i++) {
			GearTarget t = pixyVision.getGearTarget();
			if (t != null) {
				distance = t.distance;
				angle = t.angle;
				// drive with t.angle goes here
				sDash.putString("Gear Target t", t.toString());
				break;
			}
		}
		// pixyVision.getGearTargetFiltered(); // find gear target
		//LEDs.setColorMode(3);
	}

	public void disabledPeriodic() {
		robotSystem.ledOverRide(7);
		robotSystem.run();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		drive.EncoderDistance();
		//LEDs.setColorMode(3);
		
		GearTarget t = pixyVision.getGearTarget(); // find gear target
		// pixyVision.testGearPixy();
		if (t == null) {
			sDash.putString("Gear Target status", "null " + counter++);
		} else {
			sDash.putString("Gear Target exec", t.toString());
			sDash.putString("Gear Target status", "okay");
		}

		/*
		 * switch (waitmode) { case 1: // Mode 1 is to go forward thru defense,
		 * waitmodename = 1; waittime = 0.1; break; case 2: waitmodename = 2; //
		 * Moving over defense to courtyard waittime = 0.25; break; case 3:
		 * waitmodename = 3; // Moving ovedefense to courtyard waittime = 0.5;
		 * break; case 4: waitmodename = 4; // Moving ovedefense to courtyard
		 * waittime = 0.75; break; case 5: waitmodename = 5; // Moving
		 * ovedefense to courtyard waittime = 1; break;
		 * 
		 * case 6: waitmodename = 6; // Moving ovedefense to courtyard waittime
		 * = 1.5; break;
		 * 
		 * default: // Default is to do nothing waittime = 0; break; }
		 * 
		 * sDash.putNumber("Wait Mode name = ", waitmodename);
		 * sDash.putNumber("Wait time = ", waittime);
		 */
		//sDash.putNumber("AutoMode name = ", modename);
		//sDash.putNumber("mode = ", mode);
		// if (Timer.getFPGATimestamp() - startTimeShooter > waittime) {
		if (mode == 0) {

			robotAuto.drivetoLine();
		} else if (mode == 1) {
			// System.out.println("Shoot got Here in auto");
			robotAuto.drivetoLineBackwards();

		} else if (mode == 2) {
			// System.out.println("Shoot got Here in auto");
			//robotAuto.shootOnlyLeft();

		} else if (mode == 3) {
			// System.out.println("Shoot got Here in auto");
			//robotAuto.shootOnlyRight();
			robotAuto.DriveGearSide(1);
			
		} else if (mode == 4) {
			// System.out.println("Shoot got Here in auto");
			robotAuto.shootOnlyLeftRED();
		} else if (mode == 5) {
			// System.out.println("Shoot got Here in auto");
			//robotAuto.shootOnlyRightRED();
			robotAuto.DriveGearSide(0);
		}

		else {
			drive.EncoderDistance();
			robotAuto.DriveGearCenterTest();
			// robotAuto.DriveGearLeft();
		
/*
			if(donedrive)
				donedrive = !drive.DriveStraightDistanceTime(-3);
			*/
				
			// drive.DriveSlideDistance(5);
			// drive.GyroReset();
			// if(drive.turnDrive(45))
			System.out.println("donedrive" + donedrive);
			// drive.turnDrive(45);
			// robotAuto.shootOnlyRight(); //code error
			// robotAuto.MegaAuto(Posmode, Gearmode, Linemode, AutoShootmode,
			// AutoGearmode);
		}
		/*
		 * 
		 * switch (mode) { case 1: // "Shoot Left" modename = 1;
		 * robotAuto.shootOnlyLeft(); break; case 2: modename = 2; //
		 * "Shoot, Move  to Gear Left" // robotAuto.move2Court(); break; case 3:
		 * modename = 3; // "Shoot, Move  to Gear Center"
		 * robotAuto.shootDriveToCenterGear(); break; case 4: modename = 4; //
		 * "Shoot, Move  to Gear Right" // robotAuto.move2Def(); break; case 5:
		 * modename = 5; // "Shoot, Move  to Gear Left,  Overline" //
		 * robotAuto.move2CourtNoGyroTime(); break;
		 * 
		 * case 6: modename = 6; // "Shoot, Move  to Gear Right,  Overline" //
		 * robotAuto.move2CourtNoGyroTime(); break;
		 * 
		 * case 7: modename = 7; // "Shoot, Move  to Gear Center,  Overline" //
		 * robotAuto.move2CourtNoGyroTime(); break; case 8: modename = 8; //
		 * "Shoot from Left, Move Overline" robotAuto.shootLeftDriveOverline();
		 * break;
		 * 
		 * case 9: modename = 9; // "Shoot from Right, Move Overline" //
		 * robotAuto.move2CourtNoGyroTime(); break;
		 * 
		 * default: // Default is to do nothing //
		 * robotAuto.move2CourtNoGyroTime();
		 * 
		 * // use camera to find target // line up on target
		 * 
		 * // move the robot to the sweet spot // shoot the ball
		 * 
		 * break; }
		 * 
		 */
		// }
		// SmartDashboard.putNumber("Mode name = ", modename);

	}

	/*
	 * This function is called periodically during operator control
	 */

	/**
	 * This function is called periodically during operator control
	 */
	@Override

	public void teleopInit() {
		shooter.anglerInit();
		robotSystem.ledOverRide(0);			//to disable the override
		//LEDs.setColorMode(3);

	}

	public void teleopPeriodic() {
		drive.EncoderDistance();
		drive.run();
		
		robotSystem.run();
		// driveP.run();
		// shooter.run();
		climber.run();
		//gatherer.run();
		gear.run();
		// shooter.testSpeed();
		// shooter.runIndexer();
		shooter.dumbrun();
		//LEDs.modeSelect();
		// shooter.runShaker();
		drive.WallDistance();

		/*
		 * switch (hold) { case 0: { // user operated //
		 * System.out.println("state 0"); shooter.setSpeed(shooterStick.getY());
		 * if (shooterStick.getRawButton(4)) { speed = shooterStick.getY(); hold
		 * = 1; } break; } case 1: { // hold speed at current or set speed //
		 * System.out.println("state 1"); shooter.setSpeed(speed); // return to
		 * case 0 and forget current set speed if (shooterStick.getRawButton(5))
		 * { hold = 0; } if (shooterStick.getRawButton(7)) { hold = 2; } break;
		 * } case 2: { // stop shooter to resume set speed (on return to case 1)
		 * // System.out.println("state 2"); shooter.setSpeed(0); if
		 * (shooterStick.getRawButton(5)) { hold = 0; } if
		 * (shooterStick.getRawButton(6)) { hold = 1; } break; } }
		 */
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {

		// lw.addActuator("Shooter", "Angle", angler);
		/*
		 * lw.addActuator("Shooter", "Shooter", shootermotor);
		 * lw.addActuator("Shooter", "indexer", indexer);
		 * 
		 * lw.addActuator("Drive", "left_front", left_front);
		 * lw.addActuator("Drive", "right_front", right_front);
		 * lw.addActuator("Drive", "left_back", left_back);
		 * lw.addActuator("Drive", "right_back", right_back);
		 * 
		 * lw.addActuator("Gatherer", "gatherMotor", gatherMotor);
		 * 
		 * lw.addActuator("Gear", "gearSol", gearSol); lw.addSensor("Gear",
		 * "Gear Switch1", gearSwitch1); lw.addSensor("Gear", "Gear Switch2",
		 * gearSwitch2);
		 * 
		 * lw.addActuator("CLIMBER", "climbMotor1", climbMotor1);
		 * lw.addActuator("CLIMBER", "climbMotor1", climbMotor2);
		 */

	}

}
