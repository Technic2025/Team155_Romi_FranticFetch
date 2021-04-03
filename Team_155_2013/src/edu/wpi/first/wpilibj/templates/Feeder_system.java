package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Feeder_system {

    static Victor motor;
    static String direction;

    public Feeder_system(int slot) {
        motor = new Victor(slot);
    }

    public static void run(boolean fSwitch, boolean rSwitch) {
        if (!fSwitch) {
            motor.set(1);
            direction = "Forward";
        }
        if (!rSwitch) {
            direction = "Reverse";
            motor.set(-1);
        }
        if (fSwitch && rSwitch) {
            motor.set(0);
            direction = "No";
        }
        SmartDashboard.putString("Feeder Direction", direction);
        SmartDashboard.putNumber("Feeder Motor", motor.get());
    }
}
