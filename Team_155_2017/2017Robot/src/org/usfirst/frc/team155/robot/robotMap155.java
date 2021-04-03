package org.usfirst.frc.team155.robot;

/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

/**
 *
 * @author kevin
 */
public class robotMap155 {

	// all public static final int were added from Drive155
	public static final int FULL_SPEED = 1;
	public static final int HALF_SPEED = 2;
	public static final int THREE_QUARTER_SPEED = 3;
	// public static final int GYRO_RESET = 3; //top middle
	public static final int ENCODER_RESET = 5;
	public static final int PID_DISABLE = 6;
	// END added from Drive155
	// The constants to define robot connections
	// PWMs
	/*
	 * public final int DRIVE_LEFT_FRONT = 0; public final int DRIVE_LEFT_BACK =
	 * 1; public final int DRIVE_RIGHT_FRONT = 2; public final int
	 * DRIVE_RIGHT_BACK = 3; public final int SHAKER =9;
	 */

	public final int INDEXER = 4;
	public final int CLIMBER1 = 5;
	public final int CLIMBER2 = 6;
	public final int SHOOTERANGLE = 7;
	public final int GEARROTATE = 8;
	public final int GEARSUCKER = 9;
	public final int SHAKER = 0;
	public final int GATHERER=2;			//to make things happy
	// CANTalon

	public final int DRIVE_LEFT_FRONT = 1;
	public final int DRIVE_LEFT_BACK = 2;
	public final int DRIVE_RIGHT_FRONT = 3;
	public final int DRIVE_RIGHT_BACK = 4;

	public final int SHOOTER_A = 5;
	public final int SHOOTER_B = 6;

	// solenoids
	public final int DRIVE_SOL_A = 7;
	public final int DRIVE_SOL_B = 6;
	public final int GEAR_SOL_A = 4;
	public final int GEAR_SOL_B = 5;
	public final int CLIMBER_SOL_A = 0;
	public final int CLIMBER_SOL_B = 1;
	public final int SOL_6 = 2;
	public final int SOL_7 = 3;

	// digital I/O
	public final int SHOOTER_BUTTON = 10;
	public final int CLIMBER_ENCODER_B = 11;
	public final int FRONT_RIGHT_ENCODER_A = 2;
	public final int FRONT_RIGHT_ENCODER_B = 3;
	public final int FRONT_LEFT_ENCODER_A = 4;
	public final int FRONT_LEFT_ENCODER_B = 5;
	public final int BACK_RIGHT_ENCODER_A = 7;// this was 7, 12 for practicebot
	public final int BACK_RIGHT_ENCODER_B = 6;
	public final int BACK_LEFT_ENCODER_A = 8;// 8, 10 for practice bot
	public final int BACK_LEFT_ENCODER_B = 9;// 9, 11 for practice bot
	public final int GEAR_SWITCH1 = 0;// 10, 7 for practice
	public final int GEAR_SWITCH2 = 1;// 11, 8 for practice
	public final int GEAR_SWITCHFWD = 5; // THIS CONFLICTS WHEN USED WITH THE
											// PRACTICE BOT
	public final int GEAR_ROTATEUP = 6;
	public final int GEAR_ROTATEDOWN = 7;
	public final int shooter_Encoder = 2;// 12, 9 for practice

	public final int ARDUINO_0 = 6;
	public final int ARDUINO_1 = 7;
	public final int ARDUINO_2 = 8;

	// buttons
	public final int GEARFWDUP = 7;
	public final int GEARFWDDOWN = 6; // could be 7... they both go to the same
										// physical switch
	public final int LEFTSTICK = 0;
	public final int RIGHTSTICK = 1;
	public final int DSSTICK = 2;
	
	public final int FIRE = 1;
	public final int SHOOTER_ON = 2;
	public final int GYRORESET = 3;
	public final int SHOOTCAM = 4;
	public final int LEFTSTICK_5 = 5;
	public final int VARANGLE = 6;
	public final int ANGLE_2 = 7;
	public final int ANGLE_3 = 8;
	public final int ANGLE_4 = 9;
	public final int LEFTSTICK_10 = 10;
	public final int LEFTSTICK_11 = 11;
	public final int DRIVE_MODE_TRIGGER = 1;
	public final int SHOOTER_OFF = 2;
	public final int AUTOGEAR = 5;
	public final int SHOOTRANGE = 4;
	public final int AIMLIGHT_On = 3;
	public final int RIGHTSTICK_6 = 6;
	public final int RIGHTSTICK_7 = 7;
	public final int RIGHTSTICK_8 = 8;
	public final int RIGHTSTICK_9 = 9;
	public final int RIGHTSTICK_10 = 10;
	public final int RIGHTSTICK_11 = 11;
	public final int SLOWCLIMB = 5;
	public final int FASTCLIMB = 4;
	public final int SUCK = 3;
	public final int SPIT = 2;
	public final int ARMSTICK_1 = 1;
	public final int ANGLEUP = 6;
	public final int ANGLEDOWN = 7;
	public final int SHOOTMODE = 8;
	public final int MANGEAR = 9;
	public final int ARMSTICK_10 = 10;
	public final int ARMSTICK_11 = 11;

	// relays
	public final int AIMLIGHT = 0;
	public final int RELAY_1 = 1;
	public final int RELAY_2 = 2;
	public final int RELAY_3 = 3;

	// ANALOGS
	public final int GYRO = 1;
	public final int SHOOTRANGE_FINDER = 4;
	public final int GEARRANGE_FINDER = 2;
	public final int WALL_FINDER = 0;
	public final int FWD_GEAR_POSITION = 3;

	// declare inputs to be used in multiple objects here
	// call as robotSystem.dig1.get();
	DigitalInput dig1;
	DigitalInput toteSwitch;

	// arduino control pins
	public DigitalOutput ard_0;
	public DigitalOutput ard_1;
	public DigitalOutput ard_2;
	
	private boolean m_mecanum; 
	private boolean m_haveGear;
	private int m_overRideState;

	public robotMap155() {
		ard_0 = new DigitalOutput(ARDUINO_0);
		ard_1 = new DigitalOutput(ARDUINO_1);
		ard_2 = new DigitalOutput(ARDUINO_2);
		
		m_mecanum=false;
		m_haveGear=false;
		m_overRideState=0;
	}
	
	public void ledMecanumMode(boolean state){
		m_mecanum=state;
	}
	
	public void ledHaveGear(boolean state){
		m_haveGear=state;
	}
	
	public void ledOverRide(int state){
		m_overRideState=state;		
	}

	public void run() {
		int total = 0;
		
		//in order to over ride, pass a number >0 using ledOverRide
		//to restore operation, pass ledOverRide "0"
		if (m_overRideState>0) {	
			setColorMode(m_overRideState);
			return;
		}

		if (m_mecanum)
			total = total + 4;
		if (m_haveGear)
			total = total + 2;


		// 0 - no color
		// 1 - flash red
		// 2 - solid red
		// 3 - flash green
		// 4 - solid green
		// 5 - NOT USED
		// 6 - fancy LED
		// 7 - bouncy LED

		switch (total) { // ball motor fire
		case 0: // false false false
			setColorMode(1);
			break;
		case 1: // false false true
			
			break;
		case 2: // false true false
			setColorMode(2);
			break;
		case 3: // false true true
			
			break;
		case 4: // true false false
			setColorMode(3);
			break;
		case 5: // true false true

			break;
		case 6: // true true false
			setColorMode(4);
			break;
		case 7: // true true true
			
			break;
			

		}

	}

	public void setColorMode(int colormode) { //
		System.out.println("colormode =" + (colormode));

		switch (colormode) {
		case 0: // no ball
			ard_0.set(false);
			ard_1.set(false);
			ard_2.set(false);
			// System.out.println("trying 0");
			break;
		case 1: // have ball
			ard_0.set(true);
			ard_1.set(false);
			ard_2.set(false);
			// System.out.println("trying 1");
			break;
		case 2: // shooter not up to speed
			ard_0.set(false);
			ard_1.set(true);
			ard_2.set(false);
			// System.out.println("trying 2");
			break;
		case 3: // shooter up to speed and have ball
			ard_0.set(true);
			ard_1.set(true);
			ard_2.set(false); //
			// System.out.println("trying 3");
			break;
		case 4: // shooter up to speed and no ball
			ard_0.set(false);
			ard_1.set(false);
			ard_2.set(true);
			// System.out.println("trying 4");
			break;
		case 5: // shooter on and no ball
			ard_0.set(true);
			ard_1.set(false);
			ard_2.set(true);
			// System.out.println("trying 5");
			break;
		case 6: // climb mode
			ard_0.set(false);
			ard_1.set(true);
			ard_2.set(true);
			// System.out.println("trying 6");
			break;
		case 7: // disabled
			ard_0.set(true);
			ard_1.set(true);
			ard_2.set(true);
			// System.out.println("trying 7");
			break;

		}

	}
}
