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
  private Spark spark1= new Spark(0);
  private Spark spark2= new Spark(1);
  private Spark spark3= new Spark(2);
  private Spark spark4= new Spark(3);
  private Joystick joystick1= new Joystick(0);
  private Joystick joystick2= new Joystick(1);
  private AnalogGyro gyro = new AnalogGyro(1);

  @Override
  public void robotInit() {
    System.out.println("Robot Started");
  }

  @Override
  public void teleopPeriodic(){
    spark1.set(joystick1.getY());
    spark2.set(joystick1.getY());
    spark3.set(joystick2.getY());
    spark4.set(joystick2.getY());

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

