// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutonomousDistance extends SequentialCommandGroup {
  /**
   * Creates a new Autonomous Drive based on distance. This will drive out for a specified distance,
   * turn around and drive back.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   */
  public AutonomousDistance(Drivetrain drivetrain) {
    addCommands(
      new DriveDistance(0.5, 16, drivetrain),
      new TurnDegrees(-0.5, 63, drivetrain),
      new DriveDistance(0.5, 17.25, drivetrain),
      new TurnDegrees(-0.5, 90.43, drivetrain),//yellow
      new DriveDistance(.5, 27.75, drivetrain),
      new TurnDegrees(0.5, 97, drivetrain),//blue
      new DriveDistance(0.5, 18.5, drivetrain),
      new TurnDegrees(0.5, 67.5, drivetrain),
      new DriveDistance(0.5, 15, drivetrain));


       /* new DriveDistance(-0.5, 10, drivetrain),
        new TurnDegrees(-0.5, 180, drivetrain),
        new DriveDistance(-0.5, 10, drivetrain),
        new TurnDegrees(0.5, 180, drivetrain)); */
  }
}
