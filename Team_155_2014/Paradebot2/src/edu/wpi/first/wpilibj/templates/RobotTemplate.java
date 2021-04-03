/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;


import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class RobotTemplate extends IterativeRobot {
    Joystick leftStick = new Joystick(1);
    Joystick rightStick = new Joystick(2);
    Joystick gamePad = new Joystick(3);
    
    Solenoid piston1extract = new Solenoid(1);
    Solenoid piston1retract = new Solenoid(2);
    Jaguar leftfrontDrive = new Jaguar(1);
    Jaguar rightfrontDrive = new Jaguar(2);
    Jaguar leftrearDrive = new Jaguar(3);
    Jaguar rightrearDrive = new Jaguar(4);
    RobotDrive mainDrive = new RobotDrive(leftfrontDrive, rightfrontDrive, leftrearDrive, rightrearDrive);
    double left;
    double right;
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
   
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {

    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
        left = gamePad.getY();
        right =gamePad.getThrottle();
        
        mainDrive.tankDrive(leftStick, rightStick);
        //mainDrive.tankDrive(left, right);
       
               

        //this code shifts gears when certain buttons are pressed
        if (leftStick.getRawButton(1)) { //button 1 on right is equal to shift to low gear.
            piston1extract.set(true);
            piston1retract.set(false);
        }
        if (rightStick.getRawButton(1)) { //button 1 on left is equal to high gear
            piston1extract.set(false);
            piston1retract.set(true);
        }
        
        
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
