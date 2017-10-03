package ustest;

import lejos.robotics.SampleProvider;

public class UltrasonicPoller extends Thread{
	
	
		private SampleProvider us;
		private float[] usData;
		int distance;

		public UltrasonicPoller(SampleProvider us) {
			this.us = us;
			usData = new float[us.sampleSize()];
		}

		// Sensors now return floats using a uniform protocol.
		// Need to convert US result to an integer [0,255]

		public void run() {
			while (true) {
				us.fetchSample(usData, 0); // acquire data
				distance = (int) (usData[0] * 100.0); // extract from buffer, cast
														// to int

//				Log.log(Log.Sender.usSensor, Integer.toString(distance));

				try {
					Thread.sleep(50);
				} catch (Exception e) {
					e.printStackTrace();
				} 
			}
		}

		public int getDistance() {
			return distance;
		}

	}


