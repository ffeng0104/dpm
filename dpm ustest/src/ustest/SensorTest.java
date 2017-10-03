package ustest;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;

import ustest.UltrasonicPoller;

import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.EV3UltrasonicSensor;

public class SensorTest {
	public static void main(String[] args) throws InterruptedException, FileNotFoundException, UnsupportedEncodingException {
		PrintWriter writer = null;
		writer = new PrintWriter("data.csv", "UTF-8");
		EV3UltrasonicSensor sensor = new EV3UltrasonicSensor (SensorPort.S1);
		UltrasonicPoller usPoller = new UltrasonicPoller(sensor.getDistanceMode());
		usPoller.start();
		try{
			for(int i=0; i<500; i++){
				int distance = usPoller.getDistance();
				System.out.print(String.format("%d:%d%n", System.currentTimeMillis(),distance));
				writer.write(distance + "\n");
				Thread.sleep(100);
			}
		}finally{
			writer.close();
			sensor.close();
		}
	}

}
