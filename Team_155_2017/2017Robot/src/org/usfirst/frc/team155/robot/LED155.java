package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LED155 {
	robotMap155 robot;
	Drive155 drive;         
	shooter155 shooter;
	gear155 gear;
	SmartDashboard sDash;
	PixyVision155 camera;
	climber155 climber;
	// arduino control pins
	public DigitalOutput ard_0;
	public DigitalOutput ard_1;
	public DigitalOutput ard_2;
	boolean DriveLED;
	boolean ShooterLED;
	boolean fireLED;
	boolean climbLED;
	boolean gearLED;

	public LED155(robotMap155 robotSystem, Drive155 driveSystem, shooter155 shooterSystem, gear155 gearSystem,
			climber155 climbSystem, PixyVision155 cameraSystem) {

		ard_0 = new DigitalOutput(robotSystem.ARDUINO_0);
		ard_1 = new DigitalOutput(robotSystem.ARDUINO_1);
		ard_2 = new DigitalOutput(robotSystem.ARDUINO_2);
		robot = robotSystem;
		drive = driveSystem;
		shooter = shooterSystem;
		gear = gearSystem;
		camera = cameraSystem;
		climber = climbSystem;

	}

	public void modeSelect() {
		int total = 0;

		DriveLED = drive.inMech;
		ShooterLED = shooter.Shooter_On;
		fireLED = shooter.readytoFireLED;
		gearLED = gear.readSwitch();
		climbLED = climber.climbing;

		if (DriveLED)
			total = total + 4;
		if (ShooterLED)
			total = total + 2;
		if (fireLED)
			total = total + 1;
		if (gearLED)
			total = 7;
		if (climbLED)
			total = 6;

		switch (total) { // ball motor fire
		case 0: // false false false
			setColorMode(0);
			break; // ball motor fire
		case 1: // false false true

			break; // ball motor fire
		case 2: // false true false
			setColorMode(5);
			break; // ball motor fire
		case 3: // false true true
			setColorMode(4);
			break; // ball motor fire
		case 4: // true false false
			setColorMode(1);
			break; // ball motor fire
		case 5: // true false true

			break; // ball motor fire
		case 6: // true true false
			setColorMode(2);
			break; // ball motor fire
		case 7: // true true true
			setColorMode(3);
			break;
		// ball motor fire

		}

	}

	public void setColorMode(int colormode) { // System.out.println("colormode =" + (colormode));
		switch (colormode) {
		case 0: // no ball Orange?
			ard_0.set(false);
			ard_1.set(false);
			ard_2.set(false); //
			// System.out.println("trying 0");
			break;
		case 1: // have ball Blue?
			ard_0.set(true);
			ard_1.set(false);
			ard_2.set(false); //
			// System.out.println("trying 1");
			break;
		case 2: // shooter not up to speed ---Flashing Blue
			ard_0.set(false);
			ard_1.set(true);
			ard_2.set(false); //
			// System.out.println("trying 2");
			break;
		case 3: // shooter up to speed and have ball ---Green
			ard_0.set(true);
			ard_1.set(true);
			ard_2.set(false); //
			// System.out.println("trying 3");
			break;
		case 4: // shooter up to speed and no ball ---Orange Flashing----Red
			ard_0.set(false);
			ard_1.set(false);
			ard_2.set(true); //
			System.out.println("trying 4");
			break;
		case 5: // shooter on and no ball Orange Flashing----Red
			ard_0.set(true);
			ard_1.set(false);
			ard_2.set(true); //
			// System.out.println("trying 5");
			break;
		case 6: // climb mode- Party Mode
			ard_0.set(false);
			ard_1.set(true);
			ard_2.set(true); //
			// System.out.println("trying 6");
			break;
		case 7: // disabled Knightrider mode
			ard_0.set(true);
			ard_1.set(true);
			ard_2.set(true); //
			// System.out.println("trying 7");
			break;

		}

	}

}
                                      
