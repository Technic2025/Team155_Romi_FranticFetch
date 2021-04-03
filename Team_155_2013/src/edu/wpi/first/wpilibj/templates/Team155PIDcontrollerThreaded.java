/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.AnalogChannel;
import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Programming
 */
public class Team155PIDcontrollerThreaded implements Runnable {

    DriverStationLCD dsLCD = DriverStationLCD.getInstance();
    double Kp;
    double Ki;
    double Kd;
    double update_freq;
    double pre_error;
    double integral_sum;
    double setPoint;
    //publics that we may wnat access to
    public Victor motor_out1;
    public Victor motor_out2;
    public DigitalInput lowLimit;
    public DigitalInput highLimit;
    //private Talon motor_out1;
    //private Talon motor_out2;
    public Encoder m_encoder;
    public AnalogChannel pot;
    public double rpm;
    //internal control 'signals'
    private double curr_count;
    private short m_status;
    private short m_mode;
    private boolean m_type;
    private double m_tolerance;
    private double prev_count;
    private double start_time;
    private double motor_manual;
    short motors;
    private double old_start_time;
    private double offset;
    //constants
    private final int MANUAL = 2;
    private final int ENABLED = 1;
    private final int DISABLED = 0;
    private final int ENCODER = 0;
    private final int ANALOG = 1;
    public double motor_input;
    private int highcount;
    private  boolean m_settled;
    
    
    private double PID_input;

    Team155PIDcontrollerThreaded(double Kp_in, double Ki_in, double Kd_in,
            double freq, int motor, int encoderA, int encoderB, boolean position,
            int lowLimit_in, int highLimit_in, int highCount_in) {
        common_init(Kp_in, Ki_in, Kd_in, freq);

        m_encoder = new Encoder(encoderA, encoderB, false, CounterBase.EncodingType.k1X);

        m_encoder.start();
        prev_count = m_encoder.get();

        m_type = position;    //true if position, false if rate
        m_mode = ENCODER;
        motor_out1 = new Victor(motor);
        motors = 1;

        lowLimit = new DigitalInput(lowLimit_in);
        highLimit = new DigitalInput(highLimit_in);
        highcount = highCount_in;
    }

    //encoder input, 2 motor
    Team155PIDcontrollerThreaded(double Kp_in, double Ki_in, double Kd_in, double freq, int motor1, int motor2, int encoderA, int encoderB, boolean position) {
        common_init(Kp_in, Ki_in, Kd_in, freq);

        m_encoder = new Encoder(encoderA, encoderB, false, CounterBase.EncodingType.k2X);
        m_encoder.start();
        prev_count = m_encoder.get();

        m_type = position;    //true if position, false if rate
        m_mode = ENCODER;

        motor_out1 = new Victor(motor1);
        motor_out2 = new Victor(motor2);
        motors = 2;
    }

    //analog input, 1 motor
    Team155PIDcontrollerThreaded(double Kp_in, double Ki_in, double Kd_in, double freq, int motor, int analog_chan) {
        common_init(Kp_in, Ki_in, Kd_in, freq);

        pot = new AnalogChannel(analog_chan);

        m_type = true;    //true if position, false if rate
        m_mode = ANALOG;

        motor_out1 = new Victor(motor);
        motors = 1;
    }

    private void common_init(double Kp_in, double Ki_in, double Kd_in, double freq) {
        Kp = Kp_in;
        Ki = Ki_in;
        Kd = Kd_in;
        update_freq = freq;
        m_status = DISABLED;
        pre_error = 0;
        integral_sum = 0;
        m_tolerance = 0;
        motor_manual = 0;
        offset = 0;
        m_settled=false;
    }

    public void run() {

        

        double delta_time;

        while (0 != 1) {//loop for all of time
            old_start_time = start_time;
            start_time = Timer.getFPGATimestamp();  //get current time
            delta_time = start_time - old_start_time;
            if (delta_time < (1 / update_freq)) { //let's make sure we aren't early  --should this be deleted????
                Thread.yield();
            }
            motor_input = 0;        //initialize to zer0
            PID_input = 0;
            curr_count = m_encoder.get();
            rpm = Math.abs(curr_count - prev_count) / update_freq * 1;
            prev_count = curr_count;
            switch (m_status) {
                case ENABLED:
                    if (m_mode == ENCODER) {
                        if (m_type) {       //position
                            PID_input = curr_count - offset;
                            /*
                             dsLCD.println(DriverStationLCD.Line.kUser4, 1, "encoder = " + m_encoder.get());
                             dsLCD.println(DriverStationLCD.Line.kUser5, 1, "offset = " + offset);
                             dsLCD.println(DriverStationLCD.Line.kUser6, 1, "pid_input = " + PID_input);
                             */
                            //System.out.println("current encoder value  " + curr_count);
                        } else {            //speed
                            PID_input = rpm;  //make it a rate since we know time with high certainty
                        }
                    } else if (m_mode == ANALOG) {
                        PID_input = pot.getVoltage();
                    } else {
                        System.out.println("Congratulations on making a sentient program or messsing up");
                        return;
                    }

                    //run the update and set the motor
                    //System.out.println("updating PID");
                    motor_input = update(PID_input);
                    break;
                case DISABLED:
                    motor_input = 0;
                    break;
                case MANUAL:
                    motor_input = motor_manual;
                    break;
            }


            //this stops the motors when it hits the limit switches and then reset our offset
            if (m_mode == ENCODER) {
                if (m_type) {               //is it in position mode?
                    if (lowLimit.get()) {        //we're at the lowest limit
                        offset = curr_count;
                        if (motor_input > 0) {  //this allows us to move higher
                            motor_input = 0;
                        }
                    }

                    if (highLimit.get()) {       //we're at the highest limit
                        offset = curr_count - highcount;
                        if (motor_input < 0) {  //this allows us to move lower
                            motor_input = 0;
                        }
                    }
                    //System.out.println("pid input" + PID_input + "offset" + offset + "raw" + curr_count);
                }
            }



            //System.out.println("PID_input" + PID_input + "   set point" + setPoint + "   motor= "+motor_input);
            if (motors == 1) {
                motor_out1.set(motor_input);
            }
            if (motors == 2) {
                //System.out.println("setting motor");
                motor_out1.set(motor_input);
                motor_out2.set(motor_input);
            }


            //we're done, so let's give others a chance to run
            try {
                //Thread.
                Thread.sleep((long) ((1 / update_freq - (Timer.getFPGATimestamp() - start_time)) * 1000));// - (Timer.getFPGATimestamp() - start_time))); //sleep just long enough.  =period - execution time
            } catch (Exception ex) {
                ex.printStackTrace();
            }
        }
    }

    public double update(double input) {
        double error;
        double integral, diff, prop;
        double return_val;

        error = setPoint - input;
        integral_sum = integral_sum + error / update_freq;

        prop = Kp * error;
        diff = Kd * (error - pre_error) * update_freq;
        integral = Ki * integral_sum;
        pre_error = error;

        //System.out.println("Prop = " + prop + "  Diff = " + diff + 
        //                    "integral_sum);

        // dsLCD.println(DriverStationLCD.Line.kUser3, 1, "Prop = " + prop);
        // dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Diff = " + diff);
        //dsLCD.println(DriverStationLCD.Line.kUser5, 1, "Integral = " + integral +integral_sum);
        // dsLCD.println(DriverStationLCD.Line.kUser6, 1, "Integral Sum = " + integral_sum);

        return_val = LimitOutput(prop + diff + integral);


        if (m_tolerance == 0) { //is it enabled?
            return return_val;
        } else {
            if ((error * error) < (m_tolerance)) {
                m_settled =true;
                //System.out.println("deadbanded");
                return 0;
            } else {
                return return_val;
            }

            /* compare integral_sum to m_tolerance
             * when the system is _AT_ steady state, error=0 and integral_sum=0
             */

        }

    }

    /* enable - PID controls motor(s)
     * disable - motor=0, PID does not update values
     * manual - allows for manual control of motors (skips over setting motor outputs)
     */
    public void enable() {
        m_status = ENABLED;
    }

    public void disable() {
        m_status = DISABLED;
    }

    public void manual() {
        m_status = MANUAL;
    }

    public double status() {
        return m_status;
    }

    public void set(double motor_input) {
        motor_manual = motor_input;
        //m_status = MANUAL;        //if this is called, do we want to 'stop' the PID control and auto switch to manual control?
    }

    public boolean isSettled(){
            return m_settled;
    }
    //reset internal counts etc., sort of like a re-constructor
    public void reset() {
        m_status = DISABLED;
        pre_error = 0;
        integral_sum = 0;
        m_tolerance = 0;
        motor_manual = 0;

    }

    /* Sets the tolerance
     * input: a double [0,1)
     * returns 1 on success
     */
    public int setTolerance(double tol) {
        if (tol < 0) {
            m_tolerance = 0;
            return 0;
        } else {
            m_tolerance = tol*tol;
            return 1;
        }
    }

    public void setSetpoint(double x) {
        setPoint = x;
    }

    private double LimitOutput(double input) {
        double return_val;

        return_val = input;

        if (input > 1) {
            return_val = 1;
        }

        if (input < -1) {
            return_val = -1;
        }

        return return_val;

    }

    public double getPosition() {
        return (curr_count - offset);
    }
}
 