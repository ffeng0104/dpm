package lab5obj;

import java.util.ArrayList;

import lejos.hardware.Sound;
import lejos.hardware.lcd.LCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class ColorDetection {
	private Odometer odo;
	private Navigation navigate;
	private int counter = 0;
	private SampleProvider colorSensor;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private static double leftR = 2.1, rightR = 2.1, width = 15;
	private double angleNY, anglePY, angleNX, anglePX;
	private double x, y, DCS = 13;

	private float[] colorData;
	private ArrayList<Double> Data = new ArrayList<Double>();
	public static int ROTATION_SPEED = 50;

	public ColorDetection(Odometer odo, SampleProvider colorSensor,
			float[] colorData, EV3LargeRegulatedMotor left,
			EV3LargeRegulatedMotor right) {
		this.odo = odo;
		this.colorSensor = colorSensor;
		this.colorData = colorData;
		this.navigate = navigate;
		leftMotor = left;
		rightMotor = right;
		// Navigation navigate=new Navigation(odo);

	}

	public void doLocalization() {

		// drive to location listed in tutorial
		// start rotating and clock all 4 gridlines
		// do trig to compute (0,0) and 0 degrees
		// when done travel to (0,0) and turn to 0 degrees
		Navigation navigate = new Navigation(odo);
		navigate.travelTo(8, -8);                               //let the robot travel first to make sure the usSensor
		leftMotor.setSpeed(ROTATION_SPEED);                     //is able to detect 4 lines;
		rightMotor.setSpeed(ROTATION_SPEED);                    //start rotating counter-clock wise.
		leftMotor.backward();
		rightMotor.forward();

		while (counter < 4) {                                   //every time the sensor detects a line. record angle into
			colorSensor.fetchSample(colorData, 0);              //array list.
			LCD.drawString("lightSensor " + colorData[0] * 100, 0, 5);   
			if (colorData[0] * 100 < 55) {
				Data.add(odo.getAng());
				Sound.beep();
				counter++;
				Delay.msDelay(500);

			}

		}
		leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.endSynchronization();

		x = 0 - (DCS * Math.cos((double) (((Data.get(2) - Data.get(0)) / 2         //calculate real x,y coordinate
				* Math.PI / 180))));
		y = (DCS * Math.cos((double) (((Data.get(3) - Data.get(1)) / 2
				* Math.PI / 180))));

		odo.setPosition(new double[] { x, y, odo.getAng() }, new boolean[] {       //update odometer.
				true, true, true });

		Delay.msDelay(2000);
		navigate.travelTo(0, 0);

		navigate.turnTo(352, true);                                                //travel to 0,0 and turn straight
	

		leftMotor.startSynchronization();
		leftMotor.stop();
		rightMotor.stop();
		leftMotor.endSynchronization();

	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}