package org.usfirst.frc.team155.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
*
*/
public class PixyVision155 {
	public PixyI2C gearPixy;
	public PixyI2C boilerPixy;
	Port port = Port.kOnboard;
	String print;
	public PixyPacket[] packet1 = new PixyPacket[7];
	public PixyPacket[] packet2 = new PixyPacket[7];

	public PixyVision155() {
		gearPixy = new PixyI2C("gear", new I2C(port, 0x55), packet1, new PixyException(print), new PixyPacket());
		boilerPixy = new PixyI2C("Shooter", new I2C(port, 0x54), packet2, new PixyException(print), new PixyPacket());
	}

	public void testGearPixy() {
		SmartDashboard.putString("testGearPixy", "working...");

		for (int i = 0; i < packet1.length; i++)
			packet1[i] = null;

		for (int i = 1; i < 8; i++) {
			// System.out.println("i "+i);
			try {
				packet1[i - 1] = gearPixy.readPacket(i);
			} catch (PixyException e) {
				SmartDashboard.putString("testGearPixy Error: " + i, "exception");
			}

			if (packet1[i - 1] == null) {
				SmartDashboard.putString("testGearPixy packet null?: " + i, "True");
				// System.out.println("packet[i - 1] is null");
				continue;
			}
			// System.out.println("testGearPixy X Value: " + i +
			// packet1[i-1].X);
			// System.out.println("testGearPixy Y Value: " + i +
			// packet1[i-1].Y);
			// System.out.println("testGearPixy Width Value: " + i +
			// packet1[i-1].Width);
			// System.out.println("testGearPixy Height Value: " + i +
			// packet1[i-1].Height);

			SmartDashboard.putNumber("testGearPixy X Value: " + i, packet1[i - 1].X);
			SmartDashboard.putNumber("testGearPixy Y Value: " + i, packet1[i - 1].Y);
			SmartDashboard.putNumber("testGearPixy Width Value: " + i, packet1[i - 1].Width);
			SmartDashboard.putNumber("testGearPixy Height Value: " + i, packet1[i - 1].Height);
			SmartDashboard.putString("testGearPixy Error: " + i, "False");

		}
	}

	public void testBoilerPixy() {
		for (int i = 0; i < packet2.length; i++)
			packet2[i] = null;
		SmartDashboard.putString("shooterPixy hello", "working");
		for (int i = 1; i < 8; i++) {
			try {
				packet2[i - 1] = boilerPixy.readPacket(i);
			} catch (PixyException e) {
				SmartDashboard.putString("shooterPixy Error: " + i, "exception");
			}
			if (packet2[i - 1] == null) {
				SmartDashboard.putString("shooterPixy Error: " + i, "True");
				continue;
			}
			SmartDashboard.putNumber("shooterPixy X Value: " + i, packet2[i - 1].Y);
			SmartDashboard.putNumber("shooterPixy Y Value: " + i, packet2[i - 1].X);
			SmartDashboard.putNumber("shooterPixy Width Value: " + i, packet2[i - 1].Height);
			SmartDashboard.putNumber("shooterPixy Height Value: " + i, packet2[i - 1].Width);
			SmartDashboard.putString("shooterPixy Error: " + i, "False");
		}

	}

	// Get blocks that represent the vision tape on either side of the peg. This
	// can return 0,1, or 2 blocks depending what is found in a frame.
	public PixyPacket[] getPegPosition() {
		// System.out.println("in getPegPosition... ");

		PixyPacket[] blocks = gearPixy.readBlocks();

		if (blocks == null) {
			// System.out.println("blocks are null");
			SmartDashboard.putBoolean("Peg Blocks Array is null?", blocks == null);
			return null;
		}
		SmartDashboard.putString("Peg Block 0", (blocks[0] == null) ? "null" : blocks[0].toString());
		SmartDashboard.putString("Peg Block 1", (blocks[1] == null) ? "null" : blocks[1].toString());
		return blocks;
	}

	// Get blocks that represent the vision tape on the boiler stack. This
	// can return 0,1, or 2 blocks depending what is found in a frame.
	public PixyPacket[] getBolierPosition() {
		PixyPacket[] blocks = boilerPixy.readBlocks();
		// System.out.println("Shooter Blocks Array is null"+ blocks == null);
		if (blocks == null)
			return null;
		SmartDashboard.putString("Shooter Block 0", (blocks[0] == null) ? "null" : blocks[0].toString());
		SmartDashboard.putString("Shooter Block 1", (blocks[1] == null) ? "null" : blocks[1].toString());
		return blocks;
	}

	// Using blocks from pixy, create a target class that will do all the magic
	// math we need to determine angle and distance to peg.
	public GearTarget getGearTarget() {
		// System.out.println("in getGearTarget ");
		PixyPacket[] packets = getPegPosition();
		// System.out.println("back in getGearTarget ");

		if (packets == null || (packets[0] == null && packets[1] == null)) {
			// System.out.println("all packets are null ");
			return null;
		}
		return new GearTarget(packets[0], packets[1]);
	}

	// Using blocks from pixy, create a target class that will do all the magic
	// math we need to determine angle and distance to peg.
	public ShooterTarget getShooterTarget() {
		PixyPacket[] packets = getBolierPosition();
		if (packets == null || (packets[0] == null && packets[1] == null))
			return null;
		return new ShooterTarget(packets[0], packets[1]);
	}

	public GearTarget getGearTargetFiltered() {
		for (int i = 0; i < 5; i++) {
			GearTarget t = getGearTarget();
			if (t != null) {
				return t;
			}
		}
		return null;
	}

	public ShooterTarget getShooterTargetFiltered() {
		for (int i = 0; i < 5; i++) {
			ShooterTarget s = getShooterTarget();
			if (s != null) {
				return s;
			}
		}
		return null;
	}
}
