/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveSystem {

    //private Victor leftDriveMotor;
    //private Victor rightDriveMotor;
    private Talon leftDriveMotor;
    private Talon rightDriveMotor;
    private Joystick leftStick;
    private Joystick rightStick;
    private int drive_selector;
    private int m_drive_mode;
    private int scale_selector;
    private DriverStationLCD dsLCD = DriverStationLCD.getInstance();
    String dir;
    

    DriveSystem(int driveleft, int driveright, Joystick LStick, Joystick RStick, int switchSlot, int scale_sel) {
        leftDriveMotor = new Talon(driveleft);
        rightDriveMotor = new Talon(driveright);
        leftStick = LStick;
        rightStick = RStick;
        scale_selector = scale_sel; 
        m_drive_mode = 0;
        System.out.print("drive initialized");

    }

    public void run(boolean direction) {
        
        
        
        if (direction)
        {
          dir = "Tank Drive";
           tankDrive(leftStick, rightStick);
             ///System.out.println("Tank");  
        }
        else
        {
            dir = "Inverted Tank Drive";
           invertedTankDrive(leftStick, rightStick);
               //System.out.println("Invert"); 
        }
       
        

        //dsLCD.println(DriverStationLCD.Line.kUser1, 1, "Left Drive = " + leftDriveMotor.get());
        //dsLCD.println(DriverStationLCD.Line.kUser2, 1, "Right Drive = " + rightDriveMotor.get());
        //System.out.println("Left Drive = " + leftDriveMotor.get());
        //System.out.println("Right Drive = " + rightDriveMotor.get());
        /*if (m_drive_mode == 0) {
         dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Forward");
         } else if (m_drive_mode == 1) {
         dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Reverse");
         }*/

        SmartDashboard.putString("Direction", dir);
        SmartDashboard.putNumber("Left Drive Motor", leftDriveMotor.get());
        SmartDashboard.putNumber("Right Drive Motor", rightDriveMotor.get());
    }

    /* Tank Drive that pulls input for the left side from the left joystick
     * And input for the right side from the right joystick
     * 
     * inputs:  1 - joystick
     *          2 - joystick
     * 
     * output:  none
     * 
     */
    private void tankDrive(Joystick leftStick, Joystick rightStick) {
        leftDriveMotor.set(-leftStick.getAxis(Joystick.AxisType.kY));
        rightDriveMotor.set(rightStick.getAxis(Joystick.AxisType.kY));
    }

    /* Arcade drive that pulls input for the throttle from the left joystick
     * And input for steering from the right joystick
     * 
     * inputs:  1 - joystick
     *          2 - joystick
     * output:  none
     */
    private void arcadeDrive(Joystick leftStick, Joystick rightStick) {
        //the OLD OLD OLD method of mixing  --adding and subtracting in right places?
        //left_motor = joystick_left + joystick_right
        //right_motor = joystick_left - joystick_right
        //test ensure motors are between [-1,1]
    }

    /* Tank Drive that pulls input for the right side from the left joystick and inverts it
     * And pulls input for the left side from the right joystick and inverts it
     * inputs:  1 - joystick
     *          2 - joystick
     * output:  none
     */
    private void invertedTankDrive(Joystick leftStick, Joystick rightStick) {
        leftDriveMotor.set(rightStick.getAxis(Joystick.AxisType.kY) * 1);
        rightDriveMotor.set(-leftStick.getAxis(Joystick.AxisType.kY) * 1);
    }

    /* Tank drive that pulls input for the left side from the left joystick at 3/4 the power
     * And pulls input for the right side from the right joystick at 3/4 the power
     * 
     * inputs:  1 - joystick
     *          2 - joystick
     * output:  none
     */
    private void scaledDownTankDrive(Joystick leftStick, Joystick rightStick) {
        leftDriveMotor.set(leftStick.getAxis(Joystick.AxisType.kY) * .75);
        rightDriveMotor.set(-rightStick.getAxis(Joystick.AxisType.kY) * .75);
    }

    /* Tank Drive that pulls input for the left side from the right stick at 3/4 inverted power
     * And pulls input for the right side from the left joystick at 3/4 inverted power
     * 
     * inputs:  1 - joystick
     *          2 - joystick
     * output:  none
     */
    private void scaledDownInvertedTankDrive(Joystick leftStick, Joystick rightStick) {
        leftDriveMotor.set(rightStick.getAxis(Joystick.AxisType.kY) * -.75); //Winter is Comment
        rightDriveMotor.set(-leftStick.getAxis(Joystick.AxisType.kY) * -.75);
    }
}
