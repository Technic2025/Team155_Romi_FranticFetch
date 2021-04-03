
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class gear155 {
	robotMap155 robotSystem;

	public Spark gearRotate;
	public Spark gearSucker;

	public DoubleSolenoid gearSol;
	public DigitalInput gearSwitch1;
	public DigitalInput gearSwitch2;
	public DigitalInput gearSwitchFWD;
	public AnalogInput gearLeverPosition;
	public Joystick driverJoy;
	public Joystick dsStick;
	SmartDashboard sDash;
	LiveWindow lw;
	int state;
	boolean justactuate = false;

	enum FWD_GEAR_STATES {
		STOWED, DOWN, GOING_UP, UP, GOING_DOWN
	}

	private FWD_GEAR_STATES frontGearState;
	double timerStart;

	public gear155(robotMap155 robot) {
		robotSystem = robot;
		lw = new LiveWindow();
		sDash = new SmartDashboard();

		// REAR GEAR
		gearSwitch1 = new DigitalInput(robotSystem.GEAR_SWITCH1);
		gearSwitch2 = new DigitalInput(robotSystem.GEAR_SWITCH2);
		gearSol = new DoubleSolenoid(robotSystem.GEAR_SOL_A, robotSystem.GEAR_SOL_B);
		// FRONT GEAR
		gearSwitchFWD = new DigitalInput(robotSystem.GEAR_SWITCHFWD);
		gearLeverPosition = new AnalogInput(robotSystem.FWD_GEAR_POSITION);

		gearRotate = new Spark(robotSystem.GEARROTATE);
		gearSucker = new Spark(robotSystem.GEARSUCKER);

		driverJoy = new Joystick(robotSystem.RIGHTSTICK);
		dsStick = new Joystick(robotSystem.DSSTICK);
		

		frontGearState = FWD_GEAR_STATES.STOWED;
	}

	public void run() {

		switchstates();

		if (dsStick.getRawButton(robotSystem.MANGEAR)) {
			justactuate = true;
			gearSol.set(DoubleSolenoid.Value.kForward);
		} else if ((driverJoy.getRawButton(robotSystem.AUTOGEAR)) && (gearSwitch1.get() || gearSwitch2.get())) {
			gearSol.set(DoubleSolenoid.Value.kForward);
			justactuate = true;
		} else if (!driverJoy.getRawButton(robotSystem.AUTOGEAR)) {
			justactuate = false;
			gearSol.set(DoubleSolenoid.Value.kReverse);
		} else if (justactuate && driverJoy.getRawButton(robotSystem.AUTOGEAR))
			gearSol.set(DoubleSolenoid.Value.kForward);
		else
			gearSol.set(DoubleSolenoid.Value.kReverse);
		
		//System.out.println("state is " + frontGearState);
		// This section is for the Front gear
		switch (frontGearState) {
		case STOWED:
			robotSystem.ledHaveGear(false);
			// don't rotate
			gearRotate.set(0);
			// don't suck
			gearSucker.set(0);

			if (dsStick.getRawButton(robotSystem.GEARFWDDOWN)) {
				frontGearState = FWD_GEAR_STATES.GOING_DOWN;
			}
			break;
		case DOWN:
			robotSystem.ledHaveGear(false);
			// always try to bring a gear in
			gearSucker.set(.5);
			// don't rotate
			gearRotate.set(0);
			
			if (dsStick.getRawButton(robotSystem.GEARFWDUP)) {
				frontGearState = FWD_GEAR_STATES.GOING_UP;
				timerStart = Timer.getFPGATimestamp();
			}
			
			if (gearSwitchFWD.get()) {
				frontGearState = FWD_GEAR_STATES.GOING_UP;
				timerStart = Timer.getFPGATimestamp();
			}
			break;
		case GOING_UP:
			robotSystem.ledHaveGear(true);
			// always try to bring a gear in
			gearSucker.set(.0);
			// rotate UP
			gearRotate.set(.4);

			if (gearLeverPosition.getVoltage()>4.2) {
				frontGearState = FWD_GEAR_STATES.UP;
			}
			break;
		case UP:
			robotSystem.ledHaveGear(true);
			// stop sucker motor
			gearSucker.set(0);
			// don't rotate
			gearRotate.set(0);
			
			if (gearLeverPosition.getVoltage()<4.1) {
				frontGearState = FWD_GEAR_STATES.GOING_UP;
			}

			if (dsStick.getRawButton(robotSystem.GEARFWDDOWN)) {
				frontGearState = FWD_GEAR_STATES.GOING_DOWN;
			}
			break;
		case GOING_DOWN:
			robotSystem.ledHaveGear(false);
			// stop sucker motor
			gearSucker.set(0);
			// rotate down
			gearRotate.set(-.2);

			if (gearLeverPosition.getVoltage()<2.45) {
				frontGearState = FWD_GEAR_STATES.DOWN;
			}
			break;
		}

		/*
		 * lw.addActuator("Gear", "gearSol", gearSol); lw.addSensor("Gear",
		 * "Gear Switch1", gearSwitch1); lw.addSensor("Gear", "Gear Switch2",
		 * gearSwitch2);
		 */
	}

	public boolean readSwitch() {
		if (gearSwitch2.get() && gearSwitch1.get()) {

			return true;

		} else {
			return false;
		}
	}

	public void switchstates() {
		sDash.putBoolean("gearswitch 1", gearSwitch1.get());
		sDash.putBoolean("gearswitch 2", gearSwitch2.get());
	}

	public boolean gearPusherAuto() {
		switchstates();
		// If either switch has been triggered, we want the gear pusher to
		// activate.
		if (gearSwitch2.get() && gearSwitch1.get()) {
			gearSol.set(DoubleSolenoid.Value.kForward);
			return true;

		} else
			gearSol.set(DoubleSolenoid.Value.kReverse);

		return false;

	}

}
