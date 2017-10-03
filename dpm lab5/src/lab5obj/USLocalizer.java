package lab5obj;

import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;

public class USLocalizer {
	

	public static int ROTATION_SPEED = 40;
	public double theta;
	private static double leftR = 2.1, rightR = 2.1, width = 15, thetaTurn;
	private static EV3LargeRegulatedMotor leftMotor, rightMotor;
	private Odometer odo;
	private SampleProvider usSensor;
	private float[] usData;


	public USLocalizer(Odometer odo, SampleProvider usSensor, float[] usData,
			 EV3LargeRegulatedMotor leftMotor,
			EV3LargeRegulatedMotor rightMotor) {
		this.odo = odo;
		this.usSensor = usSensor;
		this.usData = usData;
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

	}

	public void doLocalization() {
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

			odo.setPosition(new double[] { 0.0, 0.0, 0.0 }, new boolean[] {   //update the odometer to 0,0,0 (to make our second part easier )
					true, true, true });
		

		}


	private float getFilteredData() {
		usSensor.fetchSample(usData, 0);
		float distance = usData[0] * 100;

		return distance;
	}

	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}