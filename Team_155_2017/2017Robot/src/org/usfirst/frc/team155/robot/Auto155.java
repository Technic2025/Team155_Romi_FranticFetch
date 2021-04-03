package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Auto155 {
	private static final double pegdistance = -20;
	private static final double LINEDISTANCE = -40;
	private static final double BACKUPDISTANCE = 4;//4 for encoder
	private static final double CENTERFORWARDDIST = -5;
	private static final double CENTERFORWARDAGAIN = -5;
	private static final double STRAIGHTPEG = -6;
	private static final double SIDEDISTANCE = -6;
	private static final double LEFTTURNANGLE = 30;
	
	boolean donedrivingagain=false;
	boolean donedrive=false;
	boolean doneturn=false;

	robotMap155 robot;
	Drive155 drive;
	shooter155 shooter;
	gear155 gear;
	SmartDashboard sDash;
	PixyVision155 camera;
	int drivestate = 0;
	int shootstate = 0;
	int gearstate = 0;
	int autostate=0;
	boolean done;
	boolean geardone;
	boolean gearpress = false;
	double startTimeShooter = 0;
	double shootDelay = 6;
	double shooterspeed = .11;
	double CENTERANGLE;
	double LEFTANGLE;
	double RIGHTANGLE;
	double FORWARDSPEED = .75;
	double FORWARDTIME = 2;

	double drivedistance;
	double slidedistance;
	
	boolean shootdone = false;
	boolean autodone = false;
	boolean donedrivetime =true;

	private final double CENTERSPEED = .5;
	private final double LEFTSPEED = .5;
	private final double RIGHTSPEED = .5;

	private final int STARTSHOOT = 0;
	private final int FIRE = 1;
	private final int DRIVEFWD = 2;
	private final int TURN = 3;
	private final int DRIVETOPEG = 4;
	private final int BACKUP = 5;
	private final int TURNSLIDE = 6;
	private final int OVERLINE = 7;
	private final int STOP = 8;
	double Angle;

	public Auto155(robotMap155 robotSystem, Drive155 driveSystem, shooter155 shooterSystem, gear155 gearSystem,
			PixyVision155 cameraSystem) {
		robot = robotSystem;
		drive = driveSystem;
		shooter = shooterSystem;
		gear = gearSystem;
		camera = cameraSystem;

		sDash = new SmartDashboard();

		CENTERANGLE = shooter.ANGLE3;
		LEFTANGLE = shooter.ANGLE1;
		RIGHTANGLE = shooter.ANGLE5;

	}

	/*
	 * public void MegaAuto(int Position, int Gearmode, int Overlinemode, int
	 * camShoot, int camGear) {
	 * 
	 * switch (drivestate) { case STARTSHOOT:// Start Shooter
	 * 
	 * // If (camShoot ==1 && shooter.SeeTarget) distance = findDistance //
	 * motorSpeed =setSpeed(Distance) setAngle = setAngler(Distance)
	 * 
	 * else if (Position == 0) { shooterspeed = LEFTSPEED; Angle = LEFTANGLE; }
	 * else if (Position == 2) { shooterspeed = RIGHTSPEED; Angle = RIGHTANGLE;
	 * } else { shooterspeed = CENTERSPEED; Angle = CENTERANGLE; }
	 * 
	 * shooter.stopIndexer(); drive.PIDDisable(); drive.driveStop(); if
	 * (shooter.setShooter(shooterspeed)) { startTimeShooter =
	 * Timer.getFPGATimestamp(); drivestate = FIRE; }
	 * 
	 * break;
	 * 
	 * case FIRE:// Fire Balls shooter.runIndexer(shooter.INDEXSPEED);
	 * shooter.runShaker(); if (Timer.getFPGATimestamp() - startTimeShooter >
	 * shootDelay) {
	 * 
	 * if (Gearmode == 0) { drivestate = DRIVEFWD; }
	 * 
	 * else if (Overlinemode == 0) { drivestate = STOP; } else { drivestate =
	 * OVERLINE; }
	 * 
	 * shooter.stopShaker(); shooterspeed = 0; startTimeShooter =
	 * Timer.getFPGATimestamp(); drive.PIDEnable();
	 * 
	 * }
	 * 
	 * break;
	 * 
	 * case DRIVEFWD:// Drive Forward if (Position == 1) drivedistance = -20;
	 * else drivedistance = -160; drive.DriveStraightDistance(drivedistance);
	 * shooter.stopIndexer(); shooter.stopShaker(); shooterspeed = 0;
	 * 
	 * if (drive.DriveStraightDistance(drivedistance)) { startTimeShooter =
	 * Timer.getFPGATimestamp(); if (Position == 1) drivestate = DRIVETOPEG;
	 * else drivedistance = -160; { drivestate = TURN; } drive.EncoderReset(); }
	 * 
	 * break; case TURN:// Turn double turnangle; if (Position == 0) turnangle =
	 * 45; else turnangle = -45;
	 * 
	 * drive.turnDrive(turnangle);
	 * 
	 * if (drive.turnDrive(turnangle)) { drive.EncoderReset(); drivestate =
	 * DRIVETOPEG; } break;
	 * 
	 * case DRIVETOPEG: drive.DriveStraightDistance(pegdistance);
	 * 
	 * if (gear.gearPusherAuto() || drive.DriveStraightDistance(-pegdistance)) {
	 * startTimeShooter = Timer.getFPGATimestamp(); drivestate = BACKUP;
	 * drive.EncoderReset(); } break;
	 * 
	 * case BACKUP: // Drive Back drive.DriveStraightDistance(20);
	 * 
	 * if (drive.DriveStraightDistance(20)) {
	 * 
	 * drivestate = TURNSLIDE; } break;
	 * 
	 * case TURNSLIDE:
	 * 
	 * if (Position == 0) { turnangle = 0; drive.turnDrive(turnangle); if
	 * (drive.turnDrive(turnangle)) { drive.EncoderReset(); drivestate =
	 * OVERLINE; } } else if (Position == 2) { turnangle = 0;
	 * drive.turnDrive(turnangle); if (drive.turnDrive(turnangle)) {
	 * drive.EncoderReset(); drivestate = OVERLINE; } } else { if (Overlinemode
	 * == 2) { slidedistance = 120;
	 * 
	 * } else { slidedistance = -120; } drive.DriveSideDistance(slidedistance);
	 * 
	 * if (drive.DriveSideDistance(slidedistance)) { drive.EncoderReset();
	 * drivestate = OVERLINE; } }
	 * 
	 * break;
	 * 
	 * case OVERLINE: drive.DriveStraightDistance(LINEDISTANCE);
	 * 
	 * if (drive.DriveStraightDistance(LINEDISTANCE)) {
	 * 
	 * drivestate = STOP; }
	 * 
	 * break;
	 * 
	 * 
	 * case STOP: // Stop drive.PIDDisable(); drive.driveStop();
	 * 
	 * break; }
	 * 
	 * shooter.setAngle(LEFTANGLE); shooter.setShooter(shooterspeed); }
	 */
	public void drivetoLine() {
		System.out.println("Got HERE");
		switch (drivestate) {
		case 0:// Start Shooter

			System.out.println("Here 1");
			startTimeShooter = Timer.getFPGATimestamp();
			drivestate = 1;

			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
			drive.myrobot.tankDrive(FORWARDSPEED, FORWARDSPEED);
			drive.sol.set(DoubleSolenoid.Value.kForward);

			break;

		case 1:// Fire Balls
			System.out.println("Here 2");
			drive.myrobot.tankDrive(FORWARDSPEED, FORWARDSPEED);
			if (Timer.getFPGATimestamp() - startTimeShooter > FORWARDTIME) {
				drive.myrobot.tankDrive(0, 0);
				drivestate = 3;
			}

			break;
		case 2:// StopFire
			System.out.println("Here 3");
			drive.myrobot.tankDrive(0, 0);

			shooterspeed = 0;

			break;
		}

	}

	public boolean shootOnlyshort() {
		System.out.println("Shoot got Here" + shootstate);
		switch (shootstate) {
		case 0:// Start Shooter
			shooterspeed = .11;
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here int");

			startTimeShooter = Timer.getFPGATimestamp();
			shootstate = 1;
			shootdone = false;
			break;

		case 1:// Start Shooter
			shooterspeed = .11;
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here 1");
			if (Timer.getFPGATimestamp() - startTimeShooter > 3) {
				startTimeShooter = Timer.getFPGATimestamp();
				shootstate = 2;
				System.out.println("Shoot here 2");
			}

			break;

		case 2:// Fire Balls
			System.out.println("Shoot here 3");
			shooter.runIndexer(shooter.INDEXSPEED);
			//shooter.runShaker();
			/*
								 * if (Timer.getFPGATimestamp() -
								 * startTimeShooter > 4) { shootstate = 3;
								 * shooterspeed = 0; startTimeShooter =
								 * Timer.getFPGATimestamp();
								 * System.out.println("Shoot here 4"); }
								 */
			if (Timer.getFPGATimestamp() - startTimeShooter > 4) {
				shootstate = 3;
				shooterspeed = 0;
				startTimeShooter = Timer.getFPGATimestamp();
				System.out.println("Shoot here 4");
			}

			break;
		case 3:// StopFire
			shooter.stopIndexer();
			//shooter.stopShaker();
			System.out.println("Shoot here 5");
			shooterspeed = 0;
			shootdone = true;
			

			break;

		}
		// drive.PIDDisable();
		// drive.driveStop();
		shooter.setAngle(.55);
		shooter.setShooter(shooterspeed);
		
		
		return shootdone;
	}

	public void shootOnlyRight() {
		System.out.println("Shoot got Here" + shootstate);
		switch (shootstate) {
		case 0:// Start Shooter
			shooterspeed = .66;
			// shooter.indexer.set(0);
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here int");

			startTimeShooter = Timer.getFPGATimestamp();
			shootstate = 1;

			break;

		case 1:// Start Shooter
			shooterspeed = .11;
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here 1");
			if (Timer.getFPGATimestamp() - startTimeShooter > 3) {
				startTimeShooter = Timer.getFPGATimestamp();
				shootstate = 2;
				System.out.println("Shoot here 2");
			}

			break;

		case 2:// Fire Balls
			System.out.println("Shoot here 3");
			shooter.runIndexer(shooter.INDEXSPEED);
			//shooter.runShaker();
			/*
								 * if (Timer.getFPGATimestamp() -
								 * startTimeShooter > 4) { shootstate = 3;
								 * shooterspeed = 0; startTimeShooter =
								 * Timer.getFPGATimestamp();
								 * System.out.println("Shoot here 4"); }
								 */
			if (Timer.getFPGATimestamp() - startTimeShooter > 4) {
				shootstate = 3;
				shooterspeed = 0;
				startTimeShooter = Timer.getFPGATimestamp();
				System.out.println("Shoot here 4");
			}

			break;
		case 3:// StopFire
			shooter.stopIndexer();
			//shooter.stopShaker();
			System.out.println("Shoot here 5");
			shooter.shooter.stopMotor();
			drivetoLine();

			break;

		}
		// drive.PIDDisable();
		// drive.driveStop();
		shooter.setAngle(.35);
		shooter.setShooter(shooterspeed);
	}

	public void shootOnlyRightRED() {
		System.out.println("Shoot got Here" + shootstate);
		switch (shootstate) {
		case 0:// Start Shooter
			shooterspeed = .66;
			// shooter.indexer.set(0);
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here int");

			startTimeShooter = Timer.getFPGATimestamp();
			shootstate = 1;

			break;

		case 1:// Start Shooter
			shooterspeed = .11;
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here 1");
			if (Timer.getFPGATimestamp() - startTimeShooter > 3) {
				startTimeShooter = Timer.getFPGATimestamp();
				shootstate = 2;
				System.out.println("Shoot here 2");
			}

			break;

		case 2:// Fire Balls
			System.out.println("Shoot here 3");
			shooter.runIndexer(shooter.INDEXSPEED);
			//shooter.runShaker();
			/*
								 * if (Timer.getFPGATimestamp() -
								 * startTimeShooter > 4) { shootstate = 3;
								 * shooterspeed = 0; startTimeShooter =
								 * Timer.getFPGATimestamp();
								 * System.out.println("Shoot here 4"); }
								 */
			if (Timer.getFPGATimestamp() - startTimeShooter > 4) {
				shootstate = 3;
				shooterspeed = 0;
				startTimeShooter = Timer.getFPGATimestamp();
				System.out.println("Shoot here 4");
			}

			break;
		case 3:// StopFire
			shooter.stopIndexer();
			//shooter.stopShaker();
			System.out.println("Shoot here 5");
			shooter.shooter.stopMotor();
			drivetoLineBackwards();

			break;

		}
		// drive.PIDDisable();
		// drive.driveStop();
		shooter.setAngle(.35);
		shooter.setShooter(shooterspeed);
	}

	public void drivetoLineBackwards() {
		System.out.println("Got HERE");
		switch (drivestate) {
		case 0:// Start Shooter

			System.out.println("Here 1");
			startTimeShooter = Timer.getFPGATimestamp();
			drivestate = 1;

			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kRearLeft, false);
			drive.myrobot.setInvertedMotor(RobotDrive.MotorType.kRearRight, false);
			drive.myrobot.tankDrive(-FORWARDSPEED, -FORWARDSPEED);
			drive.sol.set(DoubleSolenoid.Value.kForward);

			break;

		case 1:// Fire Balls
			System.out.println("Here 2");
			drive.myrobot.tankDrive(-FORWARDSPEED, -FORWARDSPEED);
			if (Timer.getFPGATimestamp() - startTimeShooter > FORWARDTIME) {
				drive.myrobot.tankDrive(0, 0);
				drivestate = 3;
			}

			break;
		case 2:// StopFire
			System.out.println("Here 3");
			drive.myrobot.tankDrive(0, 0);

			shooterspeed = 0;

			break;
		}

	}

	public void shootOnlyLeftRED() {
		System.out.println("Shoot got Here" + shootstate);
		switch (shootstate) {
		case 0:// Start Shooter
			shooterspeed = .11;
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here int");

			startTimeShooter = Timer.getFPGATimestamp();
			shootstate = 1;

			break;

		case 1:// Start Shooter
			shooterspeed = .11;
			// shooter.indexer.set(0);
			// shooter.runShaker();
			System.out.println("Shoot here 1");
			if (Timer.getFPGATimestamp() - startTimeShooter > 3) {
				startTimeShooter = Timer.getFPGATimestamp();
				shootstate = 2;
				System.out.println("Shoot here 2");
			}

			break;

		case 2:// Fire Balls
			System.out.println("Shoot here 3");
			shooter.runIndexer(shooter.INDEXSPEED);
			//shooter.runShaker();
			/*
								 * if (Timer.getFPGATimestamp() -
								 * startTimeShooter > 4) { shootstate = 3;
								 * shooterspeed = 0; startTimeShooter =
								 * Timer.getFPGATimestamp();
								 * System.out.println("Shoot here 4"); }
								 */
			if (Timer.getFPGATimestamp() - startTimeShooter > 4) {
				shootstate = 3;
				shooterspeed = 0;
				startTimeShooter = Timer.getFPGATimestamp();
				System.out.println("Shoot here 4");
			}

			break;
		case 3:// StopFire
			shooter.stopIndexer();
			//hooter.stopShaker();
			System.out.println("Shoot here 5");
			shooter.shooter.stopMotor();
			drivetoLineBackwards();

			break;

		}
		// drive.PIDDisable();
		// drive.driveStop();
		shooter.setAngle(.55);
		shooter.setShooter(shooterspeed);
	}

	public boolean DriveGearCenterTest() {
		double wait;
		// centerstraight = 6ft

		System.out.println("gearpress = " + gearpress);
		System.out.println("gearstate = " + gearstate);

		System.out.println("starttime = " + startTimeShooter);
		System.out.println("time = " + Timer.getFPGATimestamp());

		switch (gearstate) {
		case 0:
			startTimeShooter = Timer.getFPGATimestamp();
			gearstate = 1;
			break;
		case 1:// Drive Striaght
			//drive.DriveStraightDistance(CENTERFORWARDDIST);
			gearpress = gear.readSwitch();

			
			if (gearpress) {
				startTimeShooter = Timer.getFPGATimestamp();
				gearstate = 2;
				drive.EncoderReset();
				//drive.DriveStraightDistance(BACKUPDISTANCE);
				
			}
			else {
				drive.DriveStraightDistance(CENTERFORWARDDIST);
				/*
				if(donedrivetime)
					donedrivetime = !drive.DriveStraightDistanceTime(CENTERFORWARDDIST);
				else donedrivetime = true;
			*/
			
				}

			if (Timer.getFPGATimestamp() - startTimeShooter > 7) {
				gearstate = 3;
				//drive.EncoderReset();
				startTimeShooter = Timer.getFPGATimestamp();
			}

			break;
		case 2:// wait
			//gear.gearSol.set(DoubleSolenoid.Value.kForward);
			if (Timer.getFPGATimestamp() - startTimeShooter > 1) {
				//drive.EncoderReset();
				startTimeShooter = Timer.getFPGATimestamp();
				gearstate = 3;

			}

			break;

		case 3:// Drive Back
			gearstate = 3;
			gear.gearSol.set(DoubleSolenoid.Value.kForward);
			donedrive = drive.DriveStraightDistance(BACKUPDISTANCE);
			/*
			if(donedrivetime)
				donedrivetime = !drive.DriveStraightDistanceTime(BACKUPDISTANCE);
			else {donedrivetime =true;
			donedrive =true;
			}
			*/
			//gear.gearSol.set(DoubleSolenoid.Value.kReverse);
			if (donedrive ||(Timer.getFPGATimestamp() - startTimeShooter > 1.5)){
				gearstate = 5;
				drive.EncoderReset();
			}
			break;

		case 4:// Drive Striaght
			//donedrivingagain = drive.DriveStraightDistance(CENTERFORWARDAGAIN);
			
			if(donedrivetime)
				donedrivetime = !drive.DriveStraightDistanceTime(CENTERFORWARDAGAIN);
			else{
				donedrivetime=true;
			donedrivingagain = true;
			}
			
			if (donedrivingagain){
				gearstate = 5;
				drive.EncoderReset();
			}
			break;

		case 5:// Stop
			gear.gearSol.set(DoubleSolenoid.Value.kReverse);
			drive.driveStop();
			done = true;
			break;

		}

		return done;
	}

	public boolean DriveGearSide(int color) {
		double wait;
		double colorangle;
		double colorconstant = 45;
		if (color == 1){//blue
			colorangle = colorconstant;
		}
		else colorangle = -colorconstant; //red
		// centerstraight = 6ft

		System.out.println("gearpress = " + gearpress);
		System.out.println("autostate = " + autostate);

		System.out.println("starttime = " + startTimeShooter);
		System.out.println("time = " + Timer.getFPGATimestamp());

		switch (autostate) {
		case 0:
			startTimeShooter = Timer.getFPGATimestamp();
			autostate = 1;
			break;
			
		case 1:// Drive Straight
			//autostate = 3;
			donedrive = drive.DriveStraightDistance(SIDEDISTANCE);
			/*
			if(donedrivetime)
				donedrivetime = !drive.DriveStraightDistanceTime(SIDEDISTANCE);
			else {donedrive = true;
			donedrivetime=true;
			}
			*/
			
			gear.gearSol.set(DoubleSolenoid.Value.kReverse);
			if (donedrive||
				(Timer.getFPGATimestamp() - startTimeShooter > 1.5)
				||(drive.WallDistance()>Math.abs(SIDEDISTANCE))){
				autostate = 2;
				drive.EncoderReset();
			}
			break;	
			
		case 2:// Turn
			boolean doneturn = drive.turnDrive(colorangle);
			if (doneturn) {

				autostate = 3;
				drive.EncoderReset();
				drive.GyroReset();
			}

			break;	
			
			
		case 3:// Drive Striaght
			
			geardone=DriveGearCenterTest();
			
			if(geardone)done=true;
		}

		return done;
		
	
	}

	
	public boolean shootTurn(int color) {
		double wait;
		boolean moveon;	
		double turnangle;
		double turnconstant = 35;
		if (color == 1){//blue
			turnangle = turnconstant;
		}
		else turnangle = 180+turnconstant; //red
		

		switch (autostate) {
		case 0: //shoot
			moveon = shootOnlyshort();
			if(moveon){
			startTimeShooter = Timer.getFPGATimestamp();
			autostate = 1;
			moveon =false;
			}
			
			break;
			
		case 1: //away from wall
			donedrive = drive.DriveStraightDistance(-2);
			//donedrive = drive.DriveStraightDistanceTime(-2);
			gear.gearSol.set(DoubleSolenoid.Value.kReverse);
			if (donedrive||(Timer.getFPGATimestamp() - startTimeShooter > .5)){
				autostate = 2;
				drive.EncoderReset();
			}
			break;		
			
			
			
		case 2: //turn
			moveon = drive.turnDrive(turnangle);
			if (moveon){
				autostate = 3;
				drive.EncoderReset();
				drive.GyroReset();
			}
			break;	
			
		case 3://go
			DriveGearSide(1);
			autodone =true;
			break;
			
			
		}	
	
		
	return autodone;
	}
	/*
	 * public void shootDriveToCenterGear() {
	 * 
	 * switch (drivestate) { case 1:// Start Shooter shooterspeed = CENTERSPEED;
	 * shooter.stopIndexer(); drive.PIDDisable(); drive.driveStop(); if
	 * (shooter.setShooter(shooterspeed)) { startTimeShooter =
	 * Timer.getFPGATimestamp(); drivestate = 2; }
	 * 
	 * break;
	 * 
	 * case 2:// Fire Balls shooter.runIndexer(shooter.INDEXSPEED); if
	 * (Timer.getFPGATimestamp() - startTimeShooter > shootDelay) { drivestate =
	 * 3; shooterspeed = 0; startTimeShooter = Timer.getFPGATimestamp();
	 * drive.PIDEnable(); } drivestate = 3; break;
	 * 
	 * case 3:// Drive Forward drive.DriveStraightDistance(-160);
	 * shooter.stopIndexer(); shooterspeed = 0;
	 * 
	 * if (gear.gearPusherAuto() || drive.DriveStraightDistance(-160)) {
	 * startTimeShooter = Timer.getFPGATimestamp(); drivestate = 4;
	 * drive.EncoderReset(); }
	 * 
	 * break; case 4: // Drive Back drive.DriveStraightDistance(20);
	 * 
	 * if (drive.DriveStraightDistance(20)) { drive.PIDDisable();
	 * drive.driveStop(); drivestate = 5; }
	 * 
	 * break; case 5: // Stop drive.PIDDisable(); drive.driveStop();
	 * 
	 * break; }
	 * 
	 * shooter.setAngle(CENTERANGLE); shooter.setShooter(shooterspeed); }
	 */

	/*
	 * public void shootLeftDriveOverline() {
	 * 
	 * switch (drivestate) { case 1:// Start Shooter shooterspeed = LEFTSPEED;
	 * shooter.stopIndexer(); drive.PIDDisable(); drive.driveStop(); if
	 * (shooter.setShooter(shooterspeed)) { startTimeShooter =
	 * Timer.getFPGATimestamp(); drivestate = 2;
	 * 
	 * }
	 * 
	 * break;
	 * 
	 * case 2:// Fire Balls shooter.runIndexer(shooter.INDEXSPEED);
	 * drive.driveStop(); if (Timer.getFPGATimestamp() - startTimeShooter >
	 * shootDelay) { drivestate = 3; shooterspeed = 0; startTimeShooter =
	 * Timer.getFPGATimestamp(); drive.PIDEnable(); } drivestate = 3; break;
	 * 
	 * case 3:// Drive Forward drive.DriveStraightDistance(-160);
	 * shooter.stopIndexer(); shooterspeed = 0;
	 * 
	 * if (gear.gearPusherAuto() || drive.DriveStraightDistance(-160)) {
	 * startTimeShooter = Timer.getFPGATimestamp(); drivestate = 4;
	 * drive.EncoderReset(); }
	 * 
	 * break; case 4: // Stop drive.PIDDisable(); drive.driveStop();
	 * 
	 * break; }
	 * 
	 * shooter.setAngle(LEFTANGLE); shooter.setShooter(shooterspeed); }
	 * 
	 * public void shootDriveToLeftGear() {
	 * 
	 * switch (drivestate) { case 1:// Start Shooter shooterspeed = LEFTSPEED;
	 * shooter.stopIndexer(); drive.PIDDisable(); drive.driveStop(); if
	 * (shooter.setShooter(shooterspeed)) { startTimeShooter =
	 * Timer.getFPGATimestamp(); drivestate = 2; }
	 * 
	 * break;
	 * 
	 * case 2:// Fire Balls shooter.runIndexer(shooter.INDEXSPEED); if
	 * (Timer.getFPGATimestamp() - startTimeShooter > shootDelay) { drivestate =
	 * 3; shooterspeed = 0; startTimeShooter = Timer.getFPGATimestamp();
	 * drive.PIDEnable(); } drivestate = 3; break;
	 * 
	 * case 3:// Drive Forward drive.DriveStraightDistance(-160);
	 * shooter.stopIndexer(); shooterspeed = 0;
	 * 
	 * if (gear.gearPusherAuto() || drive.DriveStraightDistance(-160)) {
	 * startTimeShooter = Timer.getFPGATimestamp(); drivestate = 4;
	 * drive.EncoderReset(); }
	 * 
	 * break; case 4: // Drive Back drive.DriveStraightDistance(20);
	 * 
	 * if (drive.DriveStraightDistance(20)) { drive.PIDDisable();
	 * drive.driveStop(); drivestate = 5; }
	 * 
	 * break; case 5: // Stop drive.PIDDisable(); drive.driveStop();
	 * 
	 * break; }
	 * 
	 * shooter.setAngle(LEFTANGLE); shooter.setShooter(shooterspeed); }
	 */
}
