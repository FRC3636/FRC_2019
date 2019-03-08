package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.Spark;
//import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot{
	private Encoder redlineEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
	

	Joystick armJoystick = new Joystick(0);
	
	private final double offsetValue = 18.0;
	
  private double previous_error;

  private double[] setpointList = {offsetValue, 60.0, /*69.0,*/ 144.0, 133.0, 61.0, 100.0, 87.0};   //manually initialize array later    offset + degree

  private Spark redlineMotor1 = new Spark(2);
  private Spark redlineMotor2 = new Spark(4);
  
  //private DifferentialDrive tempDrive = new DifferentialDrive(redlineMotor1, redlineMotor2);
  
  //PIDController mainPID = new PIDController(Kp, 0, Kd, Kf, redlineEncoder, redlineMotor);

  private final double P = 1.0;
	private final double D = 1.0;
	
	

	//320 : 1

	//2.6667 pulses per degree

	//degree to radian     degrees * pi/180

	private int lastButtonPressed = 100;


	@Override
	public void teleopPeriodic(){
		if(armJoystick.getRawButton(2)){
			if(lastButtonPressed != 2){
				PDF(0, false);
			}
			else{
				PDF(0, true);
			}
			lastButtonPressed = 2;
		}

	}

  private void PDF(int buttonPressed, boolean lastPressed){
    System.out.println("Direction: " + redlineEncoder.getDirection());
    System.out.println("Value From Encoder: " + redlineEncoder.get());

    //double setpoint = current preset height     List will be made up of angles from Philip's 3D CAD
    //double theta = calculation from encoder     need to know position from ground using encoder pulses/rotations total
    double error = setpointList[buttonPressed] - (offsetValue + redlineEncoder.get()/2.6667); // Error = Target - Actual
    
    if(!lastPressed){
      previous_error = error;
    }

    double derivative = (error - previous_error) / 0.02;

    //Kf = kpf * cos(theta)       theta = current arm angle in radians (need to find this value)    
      //kpf = voltage needed to hold arm still (changes over the course of the movement of arm because of springs)
		double Kf = returnSetMotorVoltage(0, 0) * Math.cos(offsetValue + redlineEncoder.get()/2.6667);

    double rcw = P*error + D*derivative + Kf;
    //tempDrive.tankDrive(rcw/12.0, rcw/12.0);

    redlineMotor1.set(-rcw/12.0);
    redlineMotor2.set(rcw/12.0);

    previous_error = error;
  }


	public double returnSetMotorVoltage(double v, final double w) {
    //w = encoder rpm
    // R (resistance) and Ke are motor constants for the Redline/775 pro
		//WHAT IS Ke?: (According to Vex 775 Data Sheet ) At 18730 RPM 
		//and 12v, a 775 draws 0.7a
		//so w = 18730, v = 12, I = 0.7, and R = .0896 
		//then just plugged that all into that equation and solved for x
		//12-Ke*18730 = 0.7*0.0896
		//then just solve for Ke
		//that's how you get Ke
		v *= 12;
	    final double R = 0.0896;
	    final double Ke = 0.000637335;
	    final int currentLimitAmps = 37;
	    final double estimateDrawCurrent = Math.abs((v-Ke*Math.abs(w))/R);

		if (estimateDrawCurrent > currentLimitAmps) {
			if(!(Math.abs(w) > 0 && Math.abs(w) < 13000)){
				System.out.println("V: " + v + " estimateDraw:" + estimateDrawCurrent);
		    	if(v > 0){
					v = (currentLimitAmps*R)+(Ke*w);
		    	}
		    	else if(v < 0){
		    		v = -1*(currentLimitAmps*R)-(Ke*w);
		    	}
		    	System.out.println("NewV: " + v + " W: " + w + " NewEstimateDraw: " + (Math.abs((v-Ke*Math.abs(w))/R)));
			}
		}
		return v;
	}

}








/*import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot{
	//AnalogGyro gyro = new AnalogGyro(1);
	// private SerialPort port;
	private Spark liftMotor = new Spark(2);
	private Spark liftMotor2 = new Spark(4);
	private Spark overMotor = new Spark(3);
	private double starttime;
	private double currenttime;
	private double dif;
	private DifferentialDrive m_myRobot = new DifferentialDrive(new Spark(0), new Spark(1));
	private Joystick m_leftStick = new Joystick(0);
	private Joystick m_rightStick = new Joystick(1);
	
	private Encoder armEncoder = new Encoder(0,1,false,Encoder.EncodingType.k4X);
	
	double rotation = 0;
	double rotations = 20;
	private boolean leftTrigger = false;
	private boolean rightTrigger = false;
	private Encoder leftDriveEncoder = new Encoder(2,3,false,Encoder.EncodingType.k4X);
	private Encoder rightDriveEncoder = new Encoder(4,5,false,Encoder.EncodingType.k4X);
	@Override
	public void robotInit() {
		System.out.println("This is a test");
		m_myRobot.setSafetyEnabled(false);
	}
	@Override
	public void teleopInit() {
		armEncoder.reset();
		armEncoder.setSamplesToAverage(5);
		armEncoder.setMaxPeriod(10);
		armEncoder.setDistancePerPulse(1);
		starttime = System.nanoTime();
	
	}
	boolean lifting;
	boolean lowering;
	@Override
	public void teleopPeriodic() {
		//scale(m_leftStick.getY(), m_rightStick.getY());
		System.out.println(armEncoder.get());
		m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());
		//liftMotor.set(1);//pulses per sec / pulses per revolution / 60
		//System.out.println(armEncoder.getDistance());
		if(m_leftStick.getRawButton(2)){
			setMotorVoltage(.43, ((armEncoder.getRate() / 3) * 60));
			//liftMotor2.set((0.2987));
			//liftMotor.set(-(0.2987));
			
		}
		else if(m_rightStick.getRawButton(4)){
			setMotorVoltage(0.17, ((armEncoder.getRate() / 3) * 60));
		}
		//else if(m_rightStick.getRawButton(2)){
		//	lifting = false;
		//	lowering = true;
		//	setMotorVoltage(-1, ((armEncoder.getRate() / 3) * 60));
		//}
		else if(m_rightStick.getRawButton(2)){
			//setMotorVoltage(-.15, ((armEncoder.getRate() / 3) * 60));
			//armEncoder.reset();
			liftMotor.set(0.25);
			liftMotor2.set(-0.25);
		}
		else{
			setMotorVoltage(0,0);
		}
		//if(m_rightStick.getRawButton(2)){
		//	setMotorVoltage(-1, ((armEncoder.getRate() / 3) * 60));
		//	//armEncoder.reset();
		//}
		//else{
		//	setMotorVoltage(0,0);
		//}
		if(m_leftStick.getTrigger()){
			overMotor.set(0.3333333333333333);
			System.out.println("button 2 left pressed");
		}
		else if(m_rightStick.getTrigger()){
			overMotor.set(-1);
			System.out.println("button 2 pressed right");
		}
		else{
			overMotor.set(0);
		}
		//liftMotor.set(.25);
		//System.out.println((armEncoder.getRate()*60));
		//System.out.println(armEncoder.get());
		//m_myRobot.tankDrive(m_leftStick.getY(), m_rightStick.getY());
	}
	
	public void setMotorVoltage(double v, final double w) {
    //w = encoder rpm
    // R (resistance) and Ke are motor constants for the Redline/775 pro
		//WHAT IS Ke?: (According to Vex 775 Data Sheet ) At 18730 RPM 
		//and 12v, a 775 draws 0.7a
		//so w = 18730, v = 12, I = 0.7, and R = .0896 
		//then just plugged that all into that equation and solved for x
		//12-Ke*18730 = 0.7*0.0896
		//then just solve for Ke
		//that's how you get Ke
		v *= 12;
	    final double R = 0.0896;
	    final double Ke = 0.000637335;
	    final int currentLimitAmps = 37;
	    final double estimateDrawCurrent = Math.abs((v-Ke*Math.abs(w))/R);

		if (estimateDrawCurrent > currentLimitAmps) {
			if(!(Math.abs(w) > 0 && Math.abs(w) < 13000)){
				System.out.println("V: " + v + " estimateDraw:" + estimateDrawCurrent);
		    	if(v > 0){
					v = (currentLimitAmps*R)+(Ke*w);
		    	}
		    	else if(v < 0){
		    		v = -1*(currentLimitAmps*R)-(Ke*w);
		    	}
		    	System.out.println("NewV: " + v + " W: " + w + " NewEstimateDraw: " + (Math.abs((v-Ke*Math.abs(w))/R)));
			}
		}
		liftMotor2.set((v/12));
		liftMotor.set(-(v/12));
	}
}*/