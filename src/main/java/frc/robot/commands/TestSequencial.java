// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestSequencial extends SequentialCommandGroup {
  /** Creates a new TestSequencial. */
  DrivetrainSubsystem drivetrainSubsystem;
  DriveForward DriveForward1, DriveForward2, DriveForward3, DriveForward4;
  public TestSequencial(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    DriveForward1 = new DriveForward(drivetrainSubsystem, 1.0, 0.0, 0.0);
    DriveForward2 = new DriveForward(drivetrainSubsystem, 0.0, 1.0, 0.0);
    DriveForward3 = new DriveForward(drivetrainSubsystem, 0.0, -1.0, 0.0);
    DriveForward4 = new DriveForward(drivetrainSubsystem, -1.0, 0.0, 0.0);
    this.drivetrainSubsystem = drivetrainSubsystem;
    drivetrainSubsystem.zeroGyroscope();
    addCommands(DriveForward1, DriveForward2, DriveForward4, DriveForward3);
  }
}
