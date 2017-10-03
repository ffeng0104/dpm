package lab5obj;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ObjectPush{
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	public static int ROTATION_SPEED = 60;
	public double theta;
	private static double leftR = 2.1, rightR = 2.1, width = 15, thetaTurn;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odo;
	private Navigation navig;
	private SampleProvider usSensor;
    private SampleProvider colorIDSensor=colorSensor.getColorIDMode();
    int sampleSize=colorIDSensor.sampleSize();
    float[]sample=new float [sampleSize];
	private float[] usData;
	private double lastDistance;
	private boolean isDetect;
//	Object targetDistance;
	private ArrayList<Double> Data = new ArrayList<Double>();
	
	
	
	public ObjectPush (Odometer odo, SampleProvider usSensor, float[] usData,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) 
	{
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	//	this.colorValue = colorValue;
	}
	
	public void doPush()
	{
		LCD.clear();
	//	odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {   //update the odometer to 0,0,0 (to make our second part easier )
	//			true, true, true });
		double angleA, angleB;

		while (getFilteredData() <= 45) {                        //if the robot starts facing the wall, rotate clock-wise
			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.forward();
			rightMotor.backward();
		}

		while (getFilteredData() > 30) {                         //if the robot start facing against the wall
			leftMotor.setSpeed(ROTATION_SPEED);                  //rotates clock-wise
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.forward();
			rightMotor.backward();
		}
		angleA = odo.getAng();                                   //usSensor detects the first wall. Record angle
		Sound.buzz();

		while (getFilteredData() < 30) {                         //let the robot rotates counter-clock wise 

			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.backward();
			rightMotor.forward();
		}

		while (getFilteredData() > 30) {

			leftMotor.setSpeed(ROTATION_SPEED);
			rightMotor.setSpeed(ROTATION_SPEED);
			leftMotor.backward();
			rightMotor.forward();                                //until it detects the second wall. Record the angle.
		}
		angleB = odo.getAng();
		Sound.beep();
		leftMotor.stop();
		rightMotor.stop();

		if (angleA < angleB) {                                   //calculate the angle the robot needs to turn to be straight.
			thetaTurn = -(angleB - angleA - 70) / 2;              
		}
		if (angleA > angleB) {
			thetaTurn = -(angleB + 145 - (angleA + angleB) / 2);
		}

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
		leftMotor.rotate(-convertAngle(leftR, width, thetaTurn), true);
		rightMotor.rotate(convertAngle(rightR, width, thetaTurn), false);
		
		

		odo.setPosition(new double[] { 0.0, 0.0, 90.0 }, new boolean[] {   //update the odometer to 0,0,0 (to make our second part easier )
				true, true, true });
	
		
		travelTo(15,15);
		turnTo(120);
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		leftMotor.forward();
		rightMotor.backward();
		
		while(Math.abs(odo.getAng()-330)>3)
		{
			if(getFilteredData()<50)
			{
				Data.add(odo.getAng());
			}
		}
		
		turnTo(Data.get(0)-15);
		Sound.beep();
		
		while(getFilteredData()>=4)
		{
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			leftMotor.forward();
			rightMotor.forward();
		}
		leftMotor.stop();
		rightMotor.stop();
		
			if(isBlock()==true)
			{
				Sound.beep();
				travelTo(60,60);
			   
			   leftMotor.stop();
			   rightMotor.stop();
			   System.exit(0);
			   
			}
			if(isBlock()==false)
			{
				Sound.beep();
				Sound.beep();
				leftMotor.backward();
				rightMotor.backward();
			}
		
		while(Math.abs(odo.getX()-15)>3||Math.abs(odo.getY()-15)>2)
		{
			leftMotor.backward();
			rightMotor.backward();
		}
		
		leftMotor.stop();
		rightMotor.stop();
		
		
		
		
		
		
		
		turnTo(Data.get(Data.size()-1)+15);
		Sound.buzz();
		
		while(getFilteredData()>=4)
		{
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			leftMotor.forward();
			rightMotor.forward();
		}
		leftMotor.stop();
		rightMotor.stop();
	
		
			
			if(isBlock()==true)
			{
				Sound.beep();
				
				travelTo(60,60);
				

			   leftMotor.stop();
			   rightMotor.stop();
			   System.exit(0);
			}
			
			else if(isBlock()==false)
			{
				Sound.beep();
				Sound.beep();
				leftMotor.setSpeed(150);
				rightMotor.setSpeed(150);
				leftMotor.backward();
				rightMotor.backward();
				
			}
	
		while(Math.abs(odo.getX()-15)>2 || Math.abs(odo.getY()-15)>2)
		{
			leftMotor.backward();
			rightMotor.backward();
		}
		leftMotor.stop();
		rightMotor.stop();
		
		
		
		
		
		
/*		if(getFilteredData()<50)
		{
			Delay.msDelay(1500);
			leftMotor.setSpeed(150);
			rightMotor.setSpeed(150);
			leftMotor.forward();
			rightMotor.forward();
		}
		while(getFilteredData()>=5)
		{
			leftMotor.forward();
			rightMotor.forward();
		}
		leftMotor.stop();
		rightMotor.stop();
		
			if(isBlock()==true)
			{
				Sound.beep();
				travelTo(60,60);
				
			   while(odo.getX()<70)
			   {
				   leftMotor.forward();
				   rightMotor.forward();
			   }
			   leftMotor.stop();
			   rightMotor.stop();
			   
			}
			if(isBlock()==false)
			{
				Sound.beep();
				Sound.beep();
				leftMotor.backward();
				rightMotor.backward();
			}
		
		while(Math.abs(odo.getX()-15)>3||Math.abs(odo.getY()-15)>2)
		{
			leftMotor.backward();
			rightMotor.backward();
		}
		
		leftMotor.stop();
		rightMotor.stop();
		
		leftMotor.setSpeed(50);
		rightMotor.setSpeed(50);
		leftMotor.forward();
		rightMotor.backward();
		
		Delay.msDelay(4000);
		
		if(getFilteredData()<50)
		{
			Delay.msDelay(2600);
			rightMotor.forward();
		}
		while(getFilteredData()>=5)
		{
			leftMotor.forward();
			rightMotor.forward();
		}
		leftMotor.stop();
		rightMotor.stop();
	
		
			
			if(isBlock()==true)
			{
				Sound.beep();
				
				travelTo(60,60);
				
			   while(odo.getX()<70)
			   {
				   leftMotor.forward();
				   rightMotor.forward();
			   }
			   leftMotor.stop();
			   rightMotor.stop();
			   
			}
			
			else if(isBlock()==false)
			{
				Sound.beep();
				Sound.beep();
				leftMotor.setSpeed(150);
				rightMotor.setSpeed(150);
				leftMotor.backward();
				rightMotor.backward();
				
			}
	
		while(Math.abs(odo.getX()-15)>2 || Math.abs(odo.getY()-15)>2)
		{
			leftMotor.backward();
			rightMotor.backward();
		}
		leftMotor.stop();
		rightMotor.stop();
	*/	

	}
	
	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
       // LCD.drawString("distance "+distance, 0, 1);
        if(distance>100)
        {
        	distance=100;
        }
		return distance;
	}
	
/*	public void scan(double angle1,double angle2)
	{
		turnTo(angle1);
		turnTo(angle2);
		while(Math.abs(angle2-odo.getAng())>3)
		{
			double lastDistance=getFilteredData();
		    Data.add(lastDistance);
		}
	}
*/	
	public void turnTo(double angle)
	{
	double error = angle - odo.getAng();

	while (Math.abs(error) > 3) {

		error = angle - odo.getAng();

		if (error < -180.0) {
			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);
			leftMotor.backward();
			rightMotor.forward();
		} else if (error < 0.0) {
			
			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);
			leftMotor.forward();
			rightMotor.backward();
		} else if (error > 180.0) {
			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);
			leftMotor.forward();
			rightMotor.backward();
			
		} else {
			leftMotor.setSpeed(50);
			rightMotor.setSpeed(50);
			leftMotor.backward();
			rightMotor.forward();
		}
	}
	leftMotor.stop();
	rightMotor.stop();
	}
	
	public void travelTo(double x, double y) {
		double minAng;
		leftMotor.setSpeed(200);
		rightMotor.setSpeed(200);
		
		minAng = (Math.atan2(y - odo.getY(), x - odo.getX())) * (180.0 / Math.PI);
		if (minAng < 0)
			{minAng += 360.0;}
		turnTo(minAng);
		while (Math.abs(x - odo.getX()) > 2 || Math.abs(y - odo.getY()) > 2) 
		{
			leftMotor.forward();
			rightMotor.forward();
			
		}
		leftMotor.stop();
		rightMotor.stop();
		
	}

	
	private boolean isBlock()
	{
		colorSensor.getColorIDMode().fetchSample(sample, 0);
		if((sample[0]==6)||(sample[0]==7))
		{
			return true;
		}
		return false;
	}
	
	
	
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}
}