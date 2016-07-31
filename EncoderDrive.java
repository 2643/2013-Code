/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Victor; //If your team uses victors, import them instead
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar; //If your team uses victors, import them instead
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;

/**
 * 2013 robot converted from C++ to Java
 * Written @team2643 2013
 * Converted by @team2643 Adley WOng
 */
public class EncoderDrive extends SimpleRobot
{
    boolean useOfJaguars = true;
    
    //Drive Motors
    static Victor leftDriveMotorFront = new Victor(5);
    static Victor leftDriveMotorBack = new Victor(6);
    static Victor rightDriveMotorFront = new Victor(1); 
    static Victor rightDriveMotorBack =new Victor(2);
    
    //shooting motors Talon
    static Talon frontShooterMotor = new Talon(4);
    static Talon backShooterMotor = new Talon(3); 
   
    //shooter motors Jaguars
    //static Jaguar frontShooterMotor = new Jaguar(4);
    //static Jaguar backShooterMotor = new Jaguar(3);
    
    //climbing motors
    static Victor climbingMotor = new Victor(7);
    
    //joysticks
    static Joystick operaterStick = new Joystick(0);
    //static Joystick rightStick = new Joystick(2);
    static Joystick leftStick = new Joystick(1);
    
    //Drive stick 1 buttons
    static int arcadeDriveButton = 7;
    static int tankDriveButton = 8;
    
    //Drive Stick 2 buttons
    static int climberOn = 1;
    static int climberDown = 2;
    static int climberUp = 3;
    static int sendDown = 10;
    static int sendUp = 11;
    
    //operator stick
    int shooterPiston = 1;
    int tiltUp = 5;
    int tiltDown = 4;
    int frontPos = 3;
    int backPos1 = 2;
    int backPos2 = 6;
    int backPos3 = 7;
    int backPos4 = 11;
    int backPos5 = 10;
    int dumperA = 8;
    int dumperB = 9;
    
    //speeds
    double frontPos_rps = 55.0;
    double backPos1_rps = 46.0;
    double backPos2_rps = 36.5;
    double backPos3_rps = 38.0;
    double backPos4_rps = 36.8;
    double backPos5_rps = 36.9;
    double dumper_rps = 20.0;
    
    //limit switches
    DigitalInput topLimitSwitch = new DigitalInput(6);
    DigitalInput bottomLimitSwitch = new DigitalInput(7);
    
    //encoders vars *CHECK
    int shooterMotorFrontEncoderA = 1;
    int shooterMotorFrontEncoderB = 2;
    int shooterMotorBackEncoderA = 8;
    int shooterMotorBackEncoderB = 9;
    
    //Encoders
    Encoder frontShooterEncoder = new Encoder(shooterMotorFrontEncoderA, shooterMotorFrontEncoderB, false);
    Encoder backShooterEncoder = new Encoder(shooterMotorBackEncoderA, shooterMotorBackEncoderB, false);
 
    //states
    class states
    {
        public final static int stabilizing = 0;
        public final static int unstable = 1;
        public final static int retracting = 2;
        public final static int fire = 3;
    }
    
    //climber states
    class climber
    {
        public final static int sendup = 0;
        public final static int go_up = 1;
        public final static int go_down = 2;
        public final static int stop = 3;
    }
    
    int climberStates = climber.sendup;
    int autoStstes = states.unstable;
    
    //Solenoids
    Solenoid shooterAngle1 = new Solenoid(3);
    Solenoid shooterAngle2 = new Solenoid(4);
    Solenoid shooterPistonA = new Solenoid(1);
    Solenoid shooterPistonB = new Solenoid(2);
    
    //vars
    double shooterPistonDelay = 1.0;
    double ptermFirst = 0.4;
    double iterm = 0.0005;
    double dterm = 0.0;
    
    //Variables
    int test_encoder_value;
    int total_test_encoder_value;
    int second_count;
    int old_encoder_value;
    int additive_error;
    int counter;
    int number;
    int average_counter;
    int cycle_counter;
    
    double prev_desired_RPS;
    double RPS_back;
    double RPS_front;
    double turret_speed;
    double set_speed;
    double speed2;
    double old_RPS;
    double new_RPS;
    double average;
    double test_speed;
    double RPS_speed_front;
    double RPS_speed_back;
    double divided;
    double ROC;
    double error_front;
    double error_back;
    double integral_front;
    double integral_back;
    double derivitive_front;
    double derivitive_back;
    double prev_error_front;
    double prev_error_back;
    double shooter_motor_back_RPS;
    double shooter_motor_front_RPS;
    double accumerror;
    double VC_error;
    double preverror;
    double current;
    double dt;
    double P;
    double I;
    double D;
    double F;
    double out;
    double desired_RPS_control;
    
    double old_timer;
       
    boolean first_press;
    boolean arcadedrive;
    boolean claw_;
    boolean claw_go_down;
    boolean button;
    boolean pickup_on;
    boolean switch_on;
    boolean kicker_piston_on;
    boolean kicker_button_on;
    boolean constant_;
    boolean constant_desired_RPS;
    boolean test_RPS;
    boolean slow_control;
   
    //fires
    int firstFire = 5;
    int secondFire = 9;
    int thirdFire = 13;
    
    //retracts
    int firstRetract = 6;
    int secondRetraact = 11;
    int thirdRetract = 14;
    
    double pidTime = 0.2;
    int arraySize = 32;
    
    int pressureSwitch = 14;
    int compressorEnable = 2;
    
    //misc vars
    double shooterMotorSpeed = 1.0;
    double pistonWait = 1.0;
    double fireWait = 5.0;
    double upToSpeedWait = 5.0;
    double adjustSpeed  = 0.5;
    
    //array *check
    double[] array ={ 0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0,
		14.0, 15.0, 16.0, 17.0, 18.0, 19.0, 20.0, 21.0, 22.0, 23.0, 24.0, 25.0,
		26.0, 27.0, 28.0, 29.0, 30.0, 31.0 };
    
    //timer
    Timer shooterPistonTimer = new Timer();
    Timer loopTimeMeasureTimer = new Timer();
    Timer VCTimer = new Timer();
    Timer shooterResetTimer = new Timer();
    Timer pidCodeTimer = new Timer();
    Timer autoTimer = new Timer();
    Timer errorTimer = new Timer();
    Timer retractionTimer = new Timer();
    Timer stabilizingTimer = new Timer();
    Timer overrideTimer = new Timer();
    Timer shooterStopTimer = new Timer();
    
    //compressor
    Compressor compressor1 = new Compressor(pressureSwitch, compressorEnable);
    
    DriverStationLCD dsLCD;
    
    RobotDrive drive = new RobotDrive(leftDriveMotorBack, leftDriveMotorFront, rightDriveMotorBack, rightDriveMotorFront);
    
    //extras
    int lowerDistance;
    int upperDistance;
    double lowerValue;
    double upperValue;
    double difference;
    double interpolation;
    
    public void RobotDemo()
    {
        getWatchdog().setEnabled(true);
        
        //creating drive + others
        drive.setExpiration(15);
        drive.setSafetyEnabled(false);
        
        //inverting motors
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);    
        
        //implement timer
        compressor1.start();
        loopTimeMeasureTimer.start();
        VCTimer.start();
        pidCodeTimer.start();
        autoTimer.start();
        errorTimer.start();
        retractionTimer.start();
        stabilizingTimer.start();
        overrideTimer.start();
        
        //var init
        arcadedrive = true;
        test_encoder_value = 0;
        total_test_encoder_value = 0;
        old_RPS = 0;
        new_RPS = 0;
        second_count = 0;
        set_speed = 0.5;
        cycle_counter = 0;
        speed2 = 0.0;
        test_speed = 0.3;
        button = false;
        pickup_on = false;
        switch_on = false;
        kicker_piston_on = false;
        kicker_button_on = false;
        average_counter = 0;
        counter = 1;
        number = 1;
        additive_error = 0;
        prev_error_front = 0;
        prev_desired_RPS = 0;
        ROC = 0;
        claw_ = false;
        claw_go_down = false;
        constant_ = false;
        constant_desired_RPS = false;
        divided = 0;
        shooter_motor_back_RPS = backShooterMotor.get();
        first_press = true;
        test_RPS = false;
        desired_RPS_control = 0.0;
        slow_control = false;
        
        //encoder init
        frontShooterEncoder.start();
        backShooterEncoder.start();
    }
    
    public void Autonomous()
    {
        while (isAutonomous()&& isEnabled())
        {
            getWatchdog().setEnabled(true);
        }
    }
    
    public void DriverLCD()
    {
        //prints out the values of the 
        if(cycle_counter >= 50)
        {
            dsLCD.println(DriverStationLCD.Line.kUser1, 1, "RPS Back: " + RPS_back);
            dsLCD.println(DriverStationLCD.Line.kUser2, 1, "RPS Front: " + RPS_front);
            dsLCD.println(DriverStationLCD.Line.kUser3, 1, "PRS Desired RPS: " + desired_RPS_control);
        }
        
        //if any limit swicth provides a value, driver station will be notified
        if(topLimitSwitch.get())
        {
            dsLCD.println(DriverStationLCD.Line.kUser4, 1, "TOP REACHED!");
        }
        else if(bottomLimitSwitch.get())//**CHECK**
        {
            dsLCD.println(DriverStationLCD.Line.kUser4, 1, "BOTTOM REACHED!");
        }
        else
        {
            dsLCD.println(DriverStationLCD.Line.kUser4, 1, "Neither Reached");
        }
        
        //if angler is moving, driver station will be notified
        if(shooterAngle1.get())
        {
            dsLCD.println(DriverStationLCD.Line.kUser5, 1, "UP    ");
        }
        else
        {
            dsLCD.println(DriverStationLCD.Line.kUser5, 1, "DOWN    ");
        }
        
        //tells which drive mode is activated on driver station
        if(arcadedrive)
        {
            dsLCD.println(DriverStationLCD.Line.kUser6, 1, "arcade");
        }
        else
        {
            dsLCD.println(DriverStationLCD.Line.kUser6, 1, "tank");
        }
        dsLCD.updateLCD();
    }
    
    public void pneumatic_feeder_code()
    {
        //if operator joystick presses button 1
        if(operaterStick.getRawButton(shooterPiston))
        {
            //if the kicker button is false
            if(kicker_button_on == false)
            {
                //the shoot timer resets, turns the button on, starts the shooter time, and moves the piston to shoot
                shooterResetTimer.reset();
                kicker_button_on = true;
                shooterPistonTimer.start();
                
                shooterPistonA.set(false);
                shooterPistonB.set(true);
                
                System.out.println("OUT");
            }
            else //else if the boolean is true, set to false
            {
                kicker_button_on = false;
            }
            
            //if the shooter timer is now greater than the delay of 1 second, it retracts the piston along with reseting and stopping the timer
            if(shooterPistonTimer.get() >= shooterPistonDelay)
            {
                shooterPistonA.set(true);
                shooterPistonB.set(false);
                
                shooterPistonTimer.reset();
                shooterPistonTimer.stop();
                
                System.out.println("BACK");
            }
        }
    }
    
    public void pneumatic_shooter_angler_code()
    {
        //if the operator presses button 5, the climber tilts up 
        if(operaterStick.getRawButton(tiltUp))
        {
            shooterAngle1.set(true);
            shooterAngle2.set(false);
        }
        //else if the operator presses button 4, the climber tilts down
        else if(operaterStick.getRawButton(tiltDown))
        {
            shooterAngle1.set(false);
            shooterAngle2.set(true);
        }    
    }
    
   public void RPS_control_code(double desired_RPS)
    {
        if(desired_RPS != prev_desired_RPS)
        {
            integral_back = 0;
            integral_front = 0;
        }
        
        System.out.println("Desired RPS: " + desired_RPS + "  Previous RPS: " + prev_desired_RPS);

        if(pidCodeTimer.get() >= pidTime)
        {
            double actualTime = pidCodeTimer.get();
            prev_error_back = error_back;
            prev_error_front = error_front;
            RPS_back = (backShooterEncoder.get() / 360.0) / actualTime;
            RPS_front = (frontShooterEncoder.get() / 360.0) / actualTime;
            error_back = desired_RPS - RPS_back;
            error_front = desired_RPS - RPS_front;
            integral_back = integral_back + (error_back * actualTime);
            integral_front = integral_front + (error_front * actualTime);
            derivitive_back = (error_back - prev_error_back) / actualTime;
            derivitive_front = (error_front - prev_error_front) / actualTime;
            
            
            RPS_speed_back = (error_back * ptermFirst) + (integral_back * iterm) + (derivitive_back * dterm);
            RPS_speed_front = (error_front * ptermFirst) + (integral_front * iterm) + (derivitive_front * dterm);
            
            if (RPS_speed_back > 1)
            {
            	RPS_speed_back = 1;
            }
            
            if (RPS_speed_front > 1)
            {
            	RPS_speed_front = 1;
            }
            
            if (RPS_speed_back < 0)
            {
            	RPS_speed_back = 0;
            }
            
            if (RPS_speed_front < 0)
            {
            	RPS_speed_front = 0;
            }
            
            pidCodeTimer.reset();
            backShooterEncoder.reset();
            frontShooterEncoder.reset();
        }
    }
    
    public void arcade_tank_code()
    {
        //two button toggle
        if(leftStick.getRawButton(arcadeDriveButton))//button is 7
        {
            arcadedrive = true;
        }
        else if(leftStick.getRawButton(tankDriveButton))//button is 8
        {
            arcadedrive = false;
        }
        
        //if any controller presses button 2, the speed is slowed down by an adjusted speed
        if(leftStick.getRawButton(5) && leftStick.getRawButton(6))
        {
            slow_control = true;
        }
        else
        {
            slow_control = false;
        }
        
        if(arcadedrive)
        {
            //arcade drive, also the default button
            if(slow_control)
            {
                drive.arcadeDrive(adjustSpeed * (leftStick.getY()), adjustSpeed * ((leftStick.getX())), true);
            }
            else
            {
                drive.arcadeDrive(leftStick.getY(), leftStick.getX(), true);
            }
        }
        else
        {
            if(slow_control)
            {
                drive.tankDrive(adjustSpeed * (leftStick.getRawAxis(1)), adjustSpeed * (leftStick.getRawAxis(5)), true);
            }
            else
            {
                drive.tankDrive(leftStick.getRawAxis(1), leftStick.getRawAxis(5), true);
            }         
        }
    }
    
    public void z_axis_control()
    {
        if(!constant_desired_RPS)
        {
            desired_RPS_control = ((-operaterStick.getZ() + 1) / 2) * 75;
            RPS_control_code(desired_RPS_control);
        }
    }

    public void printfs()
    {
        //debuging code to print out the PRS along with the values of the limit switches
        if(cycle_counter >= 70)
        {
            System.out.println("RPS Back: " + RPS_back + "    RPS Front: " + RPS_front + "    RPS Desired: " + desired_RPS_control);
            System.out.println("LS Top: " + topLimitSwitch.get() + " LS Bottom: " + bottomLimitSwitch.get());
            
            //prints where the shooter is moving
            if(shooterAngle1.get())
            {
                System.out.println("Up   ");
            }
            else
            {
                System.out.println("Down   ");
            }
            
            //prints drive
            if(arcadedrive)
            {
                System.out.println("arcade   ");
            }
            else
            {
                System.out.println("tank    ");
            }
            System.out.println("\n");
            
            //continues to print 
            cycle_counter = 0;
        }
        cycle_counter++;
    }

    public void integral_reset()
    {
        //resets the intergals for PID
        if (operaterStick.getRawButton(10))
	{
            integral_back = 0;
            integral_front = 0;
	}
    }

    public void constant_RPS_code()
    {
        //if operator presses button 3
        if(operaterStick.getRawButton(frontPos))
        {
            //the desired rps will equal to the front position rps constant at 55.0 rps
            desired_RPS_control = frontPos_rps;
        }
        else
        {
            //if any other button is pressed
            if(operaterStick.getRawButton(backPos1))//button 2
            {
                desired_RPS_control = backPos1_rps;//46.0 rps
            }
            else if(operaterStick.getRawButton(backPos2))//button 6 
            {
                desired_RPS_control = backPos2_rps;//36.5 rps
            }
            else if(operaterStick.getRawButton(backPos3))//button 7
            {
                desired_RPS_control = backPos3_rps;//38.0 rps
            }
            else if(operaterStick.getRawButton(backPos4))//button 11
            {
                desired_RPS_control = backPos4_rps;//36.8 rps
            }
            else if(operaterStick.getRawButton(backPos5))//button 10
            {
                desired_RPS_control = backPos5_rps;//36.9 rps
            }
            else
            {
                //if operator presses either button 8 or 9
                if(operaterStick.getRawButton(dumperA) || operaterStick.getRawButton(dumperB))
                {
                    //desired rps equals to 20.0 rps
                    desired_RPS_control = dumper_rps;
                }
                else
                {
                    //else it becomes that equation below
                    desired_RPS_control = ((-operaterStick.getZ() + 1) / 2) * 75;
                }
            }
        }
        
        RPS_control_code(desired_RPS_control);
    }

    /*public void climber_state()
    {
        //climber states, very easy just read it 
        switch(climberStates)
        {
            case climber.sendup:
                if(!topLimitSwitch.get())
                {
                    climbingMotor.set(1.0);
                }
                else
                {
                    climberStates = climber.stop;
                }
            
                if(leftStick.getRawButton(3))
                {
                    climberStates = climber.go_up;
                }
            
                if(leftStick.getRawButton(2))
                {
                    climberStates = climber.go_down;
                }
                break;
                
            case climber.go_up:
                if(leftStick.getRawButton(3) && !bottomLimitSwitch.get())
                {
                    climbingMotor.set(1.0);
                }
                else
                {
                    climberStates = climber.stop;
                }
            
                if(leftStick.getRawButton(2) && !topLimitSwitch.get())
                {
                    climberStates = climber.go_down;
                }
                break;
                
            case climber.go_down:
                if(leftStick.getRawButton(2) && !bottomLimitSwitch.get())
                {
                    climbingMotor.set(-1.0);
                }
                else
                {
                    climberStates = climber.stop;
                }
                break;
                
            case climber.stop:
                climbingMotor.set(0.0);
                
                if(leftStick.getRawButton(2) && !bottomLimitSwitch.get())
                {
                    climberStates = climber.go_down;
                }
                
                if(leftStick.getRawButton(3) && !topLimitSwitch.get())
                {
                    climberStates = climber.go_up;
                }
            
                if(leftStick.getRawButton(10) && !topLimitSwitch.get())
                {
                    climberStates = climber.sendup;
                }
        }
    }*/

    public void OperatorControl()
    {
        System.out.println("********************* JAVA VERSION 0.001 Alpha ********************");
        getWatchdog().setEnabled(true);
        shooterResetTimer.start();
        
        frontShooterEncoder.reset();
        backShooterEncoder.reset();
        
        integral_back = 0.0;
        integral_front = 0.0;
        desired_RPS_control = 0.0;
        
        //retracting piston when teleop begins
        overrideTimer.reset();
        stabilizingTimer.reset();
        retractionTimer.reset();
        
        shooterPistonA.set(true);
        shooterPistonB.set(false);
        shooterStopTimer.start();
        
        while(isOperatorControl() && isEnabled())
        {
            //getWatchdog().feed();
            //---------------------Display Output---------------------------
            //dsLCD = DriverStationLCD.getInstance();
            //DriverLCD();
            printfs();
            
            //-------------------------Climber-----------------------------
            //climber_state();
            
            //---------------------------Drive-----------------------------
            arcade_tank_code();
            //**constant_RPS_code();
            
            //--------------------------PID-------------------------------
            //**integral_reset();
            
            //-------------------------Shooter----------------------------
            //**pneumatic_shooter_angler_code();
            //**pneumatic_feeder_code();
            
            //------------cRIO Housekeeping Timing-MUST HAVE--------------
            //don't really understand why b/c crio if needed try the code below
            /*try
            {
                Thread.sleep(5);
            }
            catch(InterruptedException e)
            {
                Thread.currentThread().interrupt();
            }*/
        }
    }
}