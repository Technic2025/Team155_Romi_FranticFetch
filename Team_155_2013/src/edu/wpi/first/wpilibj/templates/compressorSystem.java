/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class compressorSystem {

    static Relay compRelay;
    static DigitalInput compSwitch;

    public compressorSystem(int relaySlot, int switchSlot) {
        compRelay = new Relay(relaySlot);
        compSwitch = new DigitalInput(switchSlot);
    }

    public void run() {
        if (!compSwitch.get()) {
            compRelay.set(Relay.Value.kForward);
        } else {
            compRelay.set(Relay.Value.kOff);
        }
    }
}
