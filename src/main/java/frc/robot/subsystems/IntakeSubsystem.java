// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  CANSparkMax intakeMotor;
  public IntakeSubsystem() {
    intakeMotor = new CANSparkMax(Constants.INTAKE, MotorType.kBrushless);
  }

  public void turnOn() {
    intakeMotor.set(-0.8);
    Robot.frontIntakeSol.set(true);
  }

  public void reverse() {
    intakeMotor.set(0.8);
    Robot.frontIntakeSol.set(true);
  }

  public void turnOff() {
    intakeMotor.set(0.0);
    Robot.frontIntakeSol.set(false);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
