// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystm. */
  TalonFX shooter1, shooter2, feeder1;
  CANSparkMax feeder2;
  public ShooterSubsystem() {
    shooter1 = new TalonFX(Constants.SHOOTER_1);
    shooter2 = new TalonFX(Constants.SHOOTER_2);
    feeder1 = new TalonFX(Constants.FEEDER_1);
    feeder2 = new CANSparkMax(Constants.FEEDER_2, MotorType.kBrushless);
  }

  public void zero() {
    shooter1.set(ControlMode.PercentOutput, 0.0);
    shooter2.set(ControlMode.PercentOutput, 0.0);
    feeder1.set(ControlMode.PercentOutput, 0.0);
    feeder2.set(0.0);
  }

  public void feedIntake() {
    feeder2.set(-0.3);
    feeder1.set(ControlMode.PercentOutput, -0.3);
  }

  public void reverse() {
    feeder2.set(0.3);
    feeder1.set(ControlMode.PercentOutput, 0.3);
  }

  public void shoot() {
    shooter1.set(ControlMode.PercentOutput, -.5);
    shooter2.set(ControlMode.PercentOutput, -.5);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
