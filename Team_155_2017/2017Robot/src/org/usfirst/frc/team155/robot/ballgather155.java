package org.usfirst.frc.team155.robot;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Spark;



public class ballgather155 {
	robotMap155 robotSystem;
	SmartDashboard sDash;
	public RobotDrive myrobot;
	public Spark gatherMotor;
	Joystick dsStick;
	public double SUCKSPEED = 1;
	public double SPITSPEED = -1;
	public double STOP = 0;
	public double motorspeed;

	LiveWindow lw;

	// public Joystick

	public ballgather155(robotMap155 robot) {
		robotSystem = robot;
		lw = new LiveWindow();
		sDash = new SmartDashboard();
		
		gatherMotor = new Spark(robotSystem.GATHERER);
		dsStick = new Joystick(robotSystem.DSSTICK);

		

	}

	public void run() {

		// TalonControlMode.Voltage

		if (dsStick.getRawButton(robotSystem.SUCK)) {
			motorspeed = SUCKSPEED;
		} else if (dsStick.getRawButton(robotSystem.SPIT)) {
			motorspeed = SPITSPEED;
		} else {
			motorspeed = STOP;
		}

		gatherMotor.set(motorspeed);
		/*
		lw.addActuator("Gatherer", "gatherMotor", gatherMotor);
		*/
	}
}
