package lab5obj;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ObjectDetection {
	EV3ColorSensor colorSensor = new EV3ColorSensor(SensorPort.S2);
	public static int ROTATION_SPEED = 40;
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

//	private float[] colorData;
 
	
	public ObjectDetection(Odometer odo, SampleProvider usSensor, float[] usData,EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) 
	{
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
	//	this.colorValue = colorValue;
	}

	public void doDetection() {
		LCD.clear();
		while (true)
		{
	  //  colorSensor.getColorIDMode().fetchSample(sample, 0);
	    
		while(getFilteredData()>=6)
		{
			if (isObject()==false)
			{
				LCD.drawString("No Object       ",0,5);
			
			}
			else if(isObject()==true)
			{
			LCD.drawString("Object Detected", 0,5);
	
			}
		}
		
		while(getFilteredData()<6)
		{
		  if(isBlock()==true)
		{
			LCD.drawString("Object Detected", 0,5);
			LCD.drawString("Block    ",0,6);
			Delay.msDelay(200);
			
		}
		  else if(isBlock()==false)
		  {
			LCD.drawString("Object Detected", 0,5);
			LCD.drawString("Not Block   ",0,6);
			Delay.msDelay(200);
		  }
		 }
		}
	}

	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;
        LCD.drawString("distance "+distance, 0, 1);
		return distance;
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

	private boolean isObject()
	{
		if(getFilteredData()>50)
		{
			return false;
		}
		return true;
	}
//	private static int convertDistance(double radius, double distance) {
//		return (int) ((180.0 * distance) / (Math.PI * radius));
//	}

//	private static int convertAngle(double radius, double width, double angle) {
//		return convertDistance(radius, Math.PI * width * angle / 360.0);
//	}

}
