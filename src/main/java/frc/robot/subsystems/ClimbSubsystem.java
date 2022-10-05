// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class ClimbSubsystem extends SubsystemBase {
  /** Creates a new ClimbSubsystem. */
  TalonFX climbMotor;
  public ClimbSubsystem() {
    climbMotor = new TalonFX(Constants.CLIMB);
  }

  public void moveUp() {
    climbMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void moveDown() {
    climbMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void zero() {
    climbMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void turn() {
    Robot.climbSol.set(true);
  }

  public void straight() {
    Robot.climbSol.set(false);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
