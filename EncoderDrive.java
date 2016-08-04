/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Talon; //If your team uses victors, import them instead
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;

/**
 * This program sets up a basic robot, with two motors and encoders.
 * It then drives for five feet during teleop mode.
 * 
 * @author: Fredric Silberberg
 */
public class EncoderDrive extends SimpleRobot 
{
    //Drive Motors
    Victor leftDriveMotorFront = new Victor(5);
    Victor leftDriveMotorBack = new Victor(6);
    Victor rightDriveMotorFront = new Victor(1); 
    Victor rightDriveMotorBack = new Victor(2);
    
    //shooting motors Talon
    Talon frontShooterMotor = new Talon(4);
    Talon backShooterMotor = new Talon(3); 

    //climbing motor
    Victor climbingMotor = new Victor(7);
    
     //joysticks**incremented by 1
    
    Joystick operatorStick = new Joystick(1);
    Joystick leftStick = new Joystick(2);
    
    //Solenoids
    
     Solenoid shooterAngle1 = new Solenoid(3);
    Solenoid shooterAngle2 = new Solenoid(4);
    Solenoid shooterPistonA = new Solenoid(1);
    Solenoid shooterPistonB = new Solenoid(2);
    
    //Timers
    
    Timer shooterResetTimer = new Timer();
    Timer shooterPistonTimer = new Timer();
    
 
    //Variables
    // time in seconds
    double shooterPistonOutDelay = 1.0;
    double shooterPistonInDelay = 1.0;
    
    boolean kickerButtonOn = false; 
    
    boolean cDrive = true;
    
    int state = 0;
    double shooterPistonDelay = 1.0;
    
    //Compressor compressor = new Compressor( 14 ,1 ); 
    
    //Relay spike1 = new Relay(1);
    
    RobotDrive drive = new RobotDrive(leftDriveMotorFront, leftDriveMotorBack, rightDriveMotorFront, rightDriveMotorBack);
    
    public EncoderDrive()
    {
        //Pistons
        
        //invert drive motors
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearLeft, true);
        drive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true); 
           
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    
    //Button 1
    public void shooter()
    {
        switch(state)
        {
            case 0:
            //Not shooting, wait for button
                
                if(operatorStick.getRawButton(1)) 
                {
                    //just for insurance
                    //System.out.println();
                    shooterPistonTimer.reset();
                    state = 1;
                }
                break;
                  
                case 1:
                //move piston out, start shooting, start timer
                    
                    shooterPistonTimer.start();
                    
                    
                    shooterPistonA.set(false);
                    shooterPistonB.set(true);
                    
                    //print 10 times so we can see it
                    for(int x = 0; x < 10; x++)
                    {
                        System.out.println("OUT");
                    }
                    
                    state = 2;
                break;
                    
                case 2:
                    //wait for timer
                    if(shooterPistonTimer.get() > shooterPistonOutDelay)
                    {
                        state = 3;
                    }
                break;
                    
                
                case 3:
                    //move piston in, resets timers and shooter 
                    //we stay in this state until we press the trigger again 
                    shooterPistonA.set(true);
                    shooterPistonB.set(false);
                    
                    
                    shooterPistonTimer.reset();
                    // shooterPistonTimer.stop(); for insurance
                    shooterPistonTimer.stop();
                    shooterPistonTimer.start();
                    
                    System.out.println("BACK");
                    state = 4;
                break;    
               
                case 4:
                    //wait for pistons to retract
                if(shooterPistonTimer.get() > shooterPistonInDelay)
                {
                    state = 0;
                }
                    
            break; 
        }
    }
    
    //Button 2
    public void shooterWheels()
    {
        double rps = 0.55;
        if(operatorStick.getRawAxis(5) >= 0.1)
        {
            System.out.println("adding");
            rps = rps + 0.5;
        }
        else if(operatorStick.getRawAxis(5) <= -0.1)
        {
            System.out.println("subtracting");
            rps = rps - 0.5;
        }
        else if(rps <= 0)
        {
            rps = 0.55;
        }
        
        if(operatorStick.getRawButton(3))
        {
            frontShooterMotor.set(rps);
            backShooterMotor.set(rps);            
        }
        else
        {
            frontShooterMotor.set(0);
            backShooterMotor.set(0);
        }
    }
    //Buttons 5 and 6
    public void angler()
    {
        //if the operator presses button 4, the climber tilts up 
        if(operatorStick.getRawButton(5))
        {
            System.out.println("Up");
            shooterAngle1.set(true);
            shooterAngle2.set(false);
        }
        //else if the operator presses button 5, the climber tilts down
        else if(operatorStick.getRawButton(6))
        {
            System.out.println("Down");
            shooterAngle1.set(false);
            shooterAngle2.set(true);
        }   
        
    }

    
    public void operatorControl() 
    {
        //sets safety off
        drive.setSafetyEnabled(false);
        
        //sets the pistons to in at the start
        shooterPistonA.set(true);
        shooterPistonB.set(false);
        
        
        while(isOperatorControl() && isEnabled())
        {
            //System.out.println(operatorStick.getRawAxis(1)+  "    " + operatorStick.getRawAxis(0) + "    " + operatorStick.getRawAxis(3));

            drive.arcadeDrive((0.75 * operatorStick.getY()), (0.75 * operatorStick.getX()), true);

            /*//Compressor
            if(operatorStick.getRawButton(6))
            {
                compressor.start();
            }
            else if(operatorStick.getRawButton(7))
            {
                compressor.stop();
            }*/
            
            shooter();
            shooterWheels();
            angler();
            
            Timer.delay(0.005);
        } 
        
            
    }        
}

