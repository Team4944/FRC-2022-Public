// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestSequencial extends SequentialCommandGroup {
  /** Creates a new TestSequencial. */
  DrivetrainSubsystem drivetrainSubsystem;
  DriveForward DriveForward1, DriveForward2, DriveForward3, DriveForward4;
  Shoot shoot1, shoot2;
  IntakeOn intakeOn;
  IntakeOff intakeOff;
  LowerBall lowerBall;
  DoubleSupplier e;
  public TestSequencial(DrivetrainSubsystem drivetrainSubsystem) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    DriveForward1 = new DriveForward(drivetrainSubsystem, 4.0, 0.0, 0.0);
    DriveForward2 = new DriveForward(drivetrainSubsystem, -6.0, 0.0, 0.0);
    shoot1 = new Shoot();
    shoot2 = new Shoot();
    intakeOn = new IntakeOn();
    intakeOff = new IntakeOff();
    lowerBall = new LowerBall();
    this.drivetrainSubsystem = drivetrainSubsystem;
    drivetrainSubsystem.zeroGyroscope();
    addCommands(shoot1, intakeOn, DriveForward1, DriveForward2, intakeOff, lowerBall, shoot2);
  }
}
