package lab5obj;

import lab5obj.LCDInfo;
import lab5obj.Odometer;
import lab5obj.USLocalizer;
import lejos.hardware.*;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class Lab5 {

	// Static Resources:
	// Left motor connected to output A
	// Right motor connected to output D
	// Ultrasonic sensor port connected to input S1
	// Color sensor port connected to input S2
	private static final EV3LargeRegulatedMotor leftMotor = new EV3LargeRegulatedMotor(
			LocalEV3.get().getPort("A"));
	private static final EV3LargeRegulatedMotor rightMotor = new EV3LargeRegulatedMotor(
			LocalEV3.get().getPort("D"));
	private static final Port usPort = LocalEV3.get().getPort("S1");
	private static final Port colorPort = LocalEV3.get().getPort("S2");

	public static void main(String[] args) {
		
		int buttonChoice;
		SensorModes usSensor = new EV3UltrasonicSensor(usPort);
		SampleProvider usValue = usSensor.getMode("Distance"); // colorValue
																// provides
																// samples from
																// this instance
		float[] usData = new float[usValue.sampleSize()]; // colorData is the
															// buffer in which
															// data are returned

		do {
			// clear the display
			LCD.clear();

			// ask the user whether the motors should drive in a square or float
			LCD.drawString("< Left | Right >", 0, 0);
			LCD.drawString("       |        ", 0, 1);
			LCD.drawString(" PART1 | PART2  ", 0, 2);
		

			buttonChoice = Button.waitForAnyPress();
		} while (buttonChoice != Button.ID_LEFT
				&& buttonChoice != Button.ID_RIGHT);

		if (buttonChoice == Button.ID_LEFT) {
			

			Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);

			ObjectDetection usd = new ObjectDetection(odo, usValue, usData,leftMotor,rightMotor);
			
			usd.doDetection();
			
		} else {
			Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
		//	USLocalizer usl = new USLocalizer(odo, usValue, usData, leftMotor, rightMotor);
		//	usl.doLocalization();
			
	//		ObjectDetection usd = new ObjectDetection(odo, usValue, usData,leftMotor,rightMotor);
	//		usd.doDetection();
			LCDInfo lcd = new LCDInfo(odo);
			ObjectPush obp= new ObjectPush(odo, usValue, usData,leftMotor,rightMotor);
			obp.doPush();
			
		}
		
		
		
		//Odometer odo = new Odometer(leftMotor, rightMotor, 30, true);
	//	LCDInfo lcd = new LCDInfo(odo);

		// perform the ultrasonic localization
		//ObjectDetection usl = new ObjectDetection(odo, usValue, usData,leftMotor,rightMotor);
		
		//usl.doDetection();

		// perform the light sensor localization
	//	ColorDetection lsl = new ColorDetection(odo, colorValue, colorData,	leftMotor, rightMotor);
	//	lsl.doLocalization();

		while (Button.waitForAnyPress() != Button.ID_ESCAPE)
			;
		System.exit(0);

	}

}