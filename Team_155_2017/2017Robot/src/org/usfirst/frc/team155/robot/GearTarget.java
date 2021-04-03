package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GearTarget {

		PixyPacket block1, block2;
		double avgWidth;
		double avgHeight;
		double avgArea;
		double avgX;
		double angle;
		double distance;
		
		final double TARGET_X = 160; //mid point of horizontal frame
		final double DEGREES_PER_PIXEL = 75.0 / 320.0; //used to convert pixel target location to degrees for steering
		//final double FRAME_IN_DEGREES = 75; //frame width is 75 degrees
		//final double FRAME_IN_PIXELS = 320; //frame width is 320 pixels
		//final double GEAR_TARGET_WIDTH = 2/12; //reflective strips around gear target is 2 inches wide (in feet)

		
		public GearTarget(PixyPacket p1, PixyPacket p2) {
			//System.out.println("in GearTarget...");
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
			//System.out.println("in doMath... ");
			
			avgWidth = (block1.Width + block2.Width) / 2;
			avgHeight = (block1.Height + block2.Height) / 2;
			avgArea = avgHeight * avgWidth;
			avgX = (block1.X + block2.X) / 2;
			angle = (avgX - TARGET_X) * DEGREES_PER_PIXEL;
			//distance = GEAR_TARGET_WIDTH *FRAME_IN_PIXELS/(2 * avgWidth * Math.tan(FRAME_IN_DEGREES));
			distance = 106.82987509669 * Math.pow(.96060468112129, avgHeight);
			
			//System.out.println("doMath avgWidth "+ avgWidth);
			//System.out.println("doMath avgHeight: "+ avgHeight);
			//System.out.println("doMath avgArea: "+ avgArea);
			//System.out.println("doMath avgX: "+ avgX);
			//System.out.println("doMath angle: "+ angle);
			//System.out.println("doMath distance: "+ distance);
			SmartDashboard.putNumber("doMath avgWidth: ", avgWidth);
			SmartDashboard.putNumber("doMath avgHeight: ", avgHeight);
			SmartDashboard.putNumber("doMath avgArea: ", avgArea);
			SmartDashboard.putNumber("doMath avgX: ", avgX);
			SmartDashboard.putNumber("doMath angle: ", angle);
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
