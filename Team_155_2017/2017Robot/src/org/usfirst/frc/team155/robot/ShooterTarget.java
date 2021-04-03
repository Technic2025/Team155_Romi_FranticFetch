package org.usfirst.frc.team155.robot;

public class ShooterTarget {

	PixyPacket block1, block2;
	double avgWidth;
	double avgHeight;
	double avgArea;
	double avgX;
	double angle;
	double distance;
	
	// our shooter camera is mounted sideways
	final double TARGET_X = 100; //mid point of horizontal frame
	final double DEGREES_PER_PIXEL = 47.5 / 200; //used to convert pixel target location to degrees for steering
	final double FRAME_IN_DEGREES = 47.5; //frame width is 75 degrees
	final double FRAME_IN_PIXELS = 2000; //frame width is 320 pixels
	final double SHOOTER_TARGET_WIDTH = 8/12; //reflective strip around the boiler stack target is 8 inches wide (in feet)

	
	public ShooterTarget(PixyPacket p1, PixyPacket p2) {
		block1 = p1;
		block2 = p2;
		if (block1 == null && block2 != null)
			block1 = block2;
		else if (block1 != null && block2 == null)
			block2 = block1;
		doMath();
	}
	
	// TODO: add some private functions to do math and stuff to convert data in blocks to distance and angle.
	private void doMath(){
		avgWidth = (block1.Height + block2.Height) / 2;
		avgHeight = (block1.Width + block2.Width) / 2;
		avgArea = avgHeight * avgWidth;
		avgX = (block1.Y + block2.Y) / 2;
		angle = (avgX - TARGET_X) * DEGREES_PER_PIXEL;
		distance = SHOOTER_TARGET_WIDTH * FRAME_IN_PIXELS/(2 * avgWidth * Math.tan(FRAME_IN_DEGREES));
	}
	

	public double distance() {
		// return distance needed to drive to hang gear.
		return distance;
	}
	
	public double angle() {
		// return angle needed to turn robot to line up with gear peg
		return angle; //X distance in pixels converted to degrees
	}
	public String toString() {
		return "" +
	"avgWidth: " + avgWidth + 
	" avgHeight: " + avgHeight +
	" avgArea: " + avgArea +
	" avgX: " + avgX +
	" dpp: " + DEGREES_PER_PIXEL +
	" angle: " + angle +
	" distance: " + distance;
	}


}
