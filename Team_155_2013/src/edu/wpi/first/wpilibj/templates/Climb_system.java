
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.*;

public class Climb_system {

    static Solenoid sol1;
    static Solenoid sol2;

    Climb_system(int solenoid_1, int solenoid_2) {
        sol1 = new Solenoid(solenoid_1);
        sol2 = new Solenoid(solenoid_2);
    }

    public static void hang() {
        //May have to switch true/false
        sol1.set(false);
        sol2.set(true);
    }

    public static void release() {
        //May have to switch true/false
        sol1.set(true);
        sol2.set(false);
    }
}
