package frc.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends IterativeRobot{
    private XboxController controller = new XboxController(1);

	private DigitalInput limitSwitch = new DigitalInput(6);
	private final double[] setpointList = {64.0, 75.0, 98.5, 130.0, 140.0};
	private final double offset = 19.0;
	private SynchronousPID pidObj = new SynchronousPID(0.075, 0.001, 0);
	private final double toAngle = 8.0/3.0;

	private Spark liftMotor = new Spark(2);
	private Spark liftMotor2 = new Spark(4);
	private Spark overMotor = new Spark(3);
	private DifferentialDrive m_myRobot = new DifferentialDrive(new Spark(0), new Spark(1));
	
	//private Joystick m_leftStick = new Joystick(0);
	//private Joystick m_rightStick = new Joystick(1);
	
	private Encoder armEncoder = new Encoder(0,1,false,Encoder.EncodingType.k4X);
	
	@Override
	public void robotInit() {
		System.out.println("This is a test");
		m_myRobot.setSafetyEnabled(false);
		liftMotor.setSafetyEnabled(false);
		liftMotor2.setSafetyEnabled(false);
		overMotor.setSafetyEnabled(false);

		armEncoder.reset();
		armEncoder.setSamplesToAverage(5);
		armEncoder.setMaxPeriod(10);
		armEncoder.setDistancePerPulse(1);
	}

	@Override
	public void autonomousPeriodic() {
		teleopPeriodic();
	}

	@Override
	public void teleopPeriodic() {
		System.out.println((-armEncoder.get()/toAngle + offset) + " " + (pidObj.getSetpoint()));
		//m_myRobot.tankDrive(-m_leftStick.getY(), -m_rightStick.getY());

        m_myRobot.tankDrive(-controller.getY(GenericHID.Hand.kLeft), -controller.getY(GenericHID.Hand.kRight));

        //liftMotor.set(1);//pulses per sec / pulses per revolution / 60
		//System.out.println(armEncoder.getDistance());

		if(!limitSwitch.get()){
			System.out.println("Limit switch triggered");
			armEncoder.reset();
			armEncoder.setSamplesToAverage(5);
			armEncoder.setMaxPeriod(10);
			armEncoder.setDistancePerPulse(1);
		}


        //    4(270=left)(X=right)     3(0=left)(Y=right)    5(90=left)(B=right)
        //          2(180=left)(A=right)

		//if(m_leftStick.getRawButton(2)){
		if(controller.getPOV() == 180){
			setMotorVoltage(0.43, ((armEncoder.getRate() / 3) * 60));
			//liftMotor2.set((0.2987));
			//liftMotor.set(-(0.2987));
			
		}
		//else if(m_rightStick.getRawButton(4)){
		else if(controller.getXButton()){
			setMotorVoltage(0.17, ((armEncoder.getRate() / 3) * 60));
		}

		//else if(m_rightStick.getRawButton(2)){
		//	lifting = false;
		//	lowering = true;
		//	setMotorVoltage(-1, ((armEncoder.getRate() / 3) * 60));
		//}

		//else if(m_rightStick.getRawButton(2)){
		else if(controller.getAButton()){
			//setMotorVoltage(-.15, ((armEncoder.getRate() / 3) * 60));
			//armEncoder.reset();
			liftMotor.set(0.25);
			liftMotor2.set(-0.25);
		}

		//else if(m_leftStick.getRawButton(4)){
		else if(controller.getPOV() == 270){
			setSetpoint(setpointList[0]);
			pidVolts();
		}
		//else if(m_leftStick.getRawButton(3)){
		else if(controller.getPOV() == 0){
			setSetpoint(setpointList[1]);
			pidVolts();
		}
		//else if(m_rightStick.getRawButton(3)){
		else if(controller.getYButton()){
			setSetpoint(setpointList[2]);
			pidVolts();
		}

		//else if(m_rightStick.getRawButton(5)){
		else if(controller.getBButton()){
			setSetpoint(setpointList[3]);
			pidVolts();
		}
		//else if(m_leftStick.getRawButton(5)){
		else if(controller.getPOV() == 90){
			setSetpoint(setpointList[4]);
			pidVolts();
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

        //if(m_leftStick.getTrigger()){
		if(controller.getBumper(GenericHID.Hand.kLeft)){
			overMotor.set(0.3333333333333333);
			System.out.println("button 2 left pressed");
		}
		//else if(m_rightStick.getTrigger()){
		else if(controller.getBumper(GenericHID.Hand.kRight)){
			overMotor.set(-1);
			System.out.println("button 2 pressed right");
		}
		else{
			overMotor.set(0);
		}

		System.out.println(liftMotor.get() + " " + liftMotor2.get());
	}
	
	private void setMotorVoltage(double v, final double w) {
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
		if(v > 6){
			v = 6;
		}
		else if(v < -6){
			v = -6;
		}
		liftMotor2.set((v/12));
		liftMotor.set(-(v/12));
	}

	private void setSetpoint(double setpoint){
		pidObj.setSetpoint(setpoint);
	}

	private void pidVolts(){
		double v = pidObj.calculate(-armEncoder.get()/toAngle + offset);
		System.out.println("v: " + v);
		liftMotor2.set(v);
		liftMotor.set(-v);
	}

}
