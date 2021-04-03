package org.usfirst.frc.team155.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.DrawMode;
import com.ni.vision.NIVision.Image;
import com.ni.vision.NIVision.ShapeMode;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.tables.TableKeyNotDefinedException;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.networktables2.type.NumberArray;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This demo shows the use of the AxisCamera class. Uses AxisCamera class to
 * manually acquire a new image each frame, and annotate the image by drawing a
 * circle on it, and show it on the FRC Dashboard.
 */

public class Robot extends SampleRobot {
	int session;
	Image frame;
	AxisCamera camera;
	int inited;

	// NetworkTable server;

	public void robotInit() {

		frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);

		// open the camera at the IP address assigned. This is the IP address
		// that the camera
		// can be accessed through the web interface.
		camera = new AxisCamera("10.1.55.11");
		inited =0;
	}

	public void operatorControl() {

		/**
		 * grab an image from the camera, draw the circle, and provide it for
		 * the camera server which will in turn send it to the dashboard.
		 */
		NIVision.Rect rect = new NIVision.Rect(10, 10, 100, 100);

		while (isOperatorControl() && isEnabled()) {
			camera.getImage(frame);

			CameraServer.getInstance().setImage(frame);

			/** robot code here! **/
			Timer.delay(0.005); // wait for a motor update time

			// get X and Y values from RR.
			// NetworkTable server = NetworkTable.getTable("SmartDashboard");
			if (inited == 1) {
				inited = 1;
				NetworkTable.setClientMode();
				NetworkTable.setIPAddress("10.1.55.22");
			}
			NetworkTable server = NetworkTable.getTable("SmartDashboard");

			System.out.println("attempting to get data from rr");

			try {
				System.out.println(server.getString("COG_X"));
			} catch (TableKeyNotDefinedException ex) {
				ex.printStackTrace();
			}

			/*
			 * NetworkTable server = NetworkTable.getTable(""); try { final
			 * NumberArray targetNum = new NumberArray();
			 * server.retrieveValue("COG_X", targetNum); if (targetNum.size()>0)
			 * { System.out.print(targetNum.get(0)); System.out.print(' ');
			 * System.out.println(targetNum.get(1)); } } catch
			 * (TableKeyNotDefinedException exp) { }
			 */
			/*
			 * while(true) { final NumberArray targetNum = new NumberArray();
			 * table.retrieveValue("COG", targetNum);
			 * System.out.println("recieved data"); for (int
			 * i=0;i<targetNum.size();i++) {
			 * System.out.println("index"+i+"COG_X=" + targetNum.get(i));
			 * SmartDashboard.putNumber("COG_X", targetNum.get(0));
			 * SmartDashboard.putNumber("COG_Y", targetNum.get(1)); } }
			 */

		}
	}

	public void test() {
	}
}
