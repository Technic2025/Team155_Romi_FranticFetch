/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Jon Hrenko
 */
public class mainSensors155 {
    robotMap155 robotSystem; 
    
    public mainSensors155(robotMap155 robot){
    robotSystem = robot;  
    }
    
    

    public double getDistance() {
        double input;
        double output;
        input = robotSystem.rangeFinder.getAverageVoltage();
        output = (input - 0.0254)/ 0.118;
        SmartDashboard.putNumber("Rangefinder Distance = ", output);
        return output;
    
    }
    public double getForce() {
        double input;
        double output;
        input = robotSystem.loadSensor.getAverageVoltage();
        output = (input - 0.0254)/ 0.118;
        SmartDashboard.putNumber("Load Sensor Force = ", output);
        return output;
    
    }
    
}
