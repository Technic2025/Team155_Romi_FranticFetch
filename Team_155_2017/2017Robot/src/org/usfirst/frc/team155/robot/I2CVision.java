package org.usfirst.frc.team155.robot;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

import edu.wpi.first.wpilibj.I2C;

public class I2CVision {
	I2C I2CBus;
	short cX = 0, cY = 0, cW = 0, cH = 0;
	int pixyData[];
	
	public I2CVision(){
		
		
	}

	public void readPixy() {

		byte[] dataBuffer = new byte[8];

		I2CBus = new I2C(I2C.Port.kOnboard, 0x54);	
		
			I2CBus.read(0x03, 8, dataBuffer);
			ByteBuffer compBuffer = ByteBuffer.wrap(dataBuffer);
			compBuffer.order(ByteOrder.LITTLE_ENDIAN);
			cX=compBuffer.getShort(1);
			cY=compBuffer.getShort(3);
			cW=compBuffer.getShort(5);
			cH=compBuffer.getShort(7);
			
			System.out.println("cX=" + cX);
			System.out.println("cY=" + cY);
			System.out.println("cW=" + cW);
			System.out.println("cH=" + cH);
	
				} 
		

	}