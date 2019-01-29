/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

  private Spark myLeft = new Spark(1);
  private Spark myRight = new Spark(2);
  private DifferentialDrive myDrive = new DifferentialDrive(myLeft, myRight);
  private Joystick leftJoystick = new Joystick(0);
  private Joystick rightJoystick = new Joystick(1);
  //private AnalogGyro gyro = new AnalogGyro(1);

  @Override
  public void robotInit() {
    System.out.println("Robot Started");
  }

  @Override
  public void teleopPeriodic(){
    //driving stuff
    myDrive.tankDrive(leftJoystick.getY(),  rightJoystick.getY());



    //gyroscope stuff
    /*double angle= gyro.getAngle();
    angle *=100;
    angle= Math.round(angle);
    angle /= 100;
    double oldangle=0;
    if(angle!= oldangle){
      System.out.println(angle);
      oldangle=angle;*/
    }
    /*gyro.reset();
    try{
      Thread.sleep(500);
    }catch(Exception e){
      e.printStackTrace();
    }*/
  }

