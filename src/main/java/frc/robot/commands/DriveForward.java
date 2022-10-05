// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveForward extends CommandBase {
  /** Creates a new DriveForward. */
  DrivetrainSubsystem drivetrainSubsystem;
  double innitTime = 0;
  ChassisSpeeds chassisSpeeds;
  double x = 0;
  double y = 0;
  double rotation = 0;

  double startingRotation = 0;
  double targetRotation = 0;

  PIDController rotationPID;

  public SwerveDriveOdometry swerveDriveOdometry;
  
  //ChassisSpeeds zeroChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
  public DriveForward(DrivetrainSubsystem drivetrainSubsystem, double dX, double dY, double rotation) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.x = dX;
    this.y = dY;
    this.rotation = rotation;
    this.drivetrainSubsystem = drivetrainSubsystem;
    this.swerveDriveOdometry = drivetrainSubsystem.swerveDriveOdometry;
    this.rotationPID = new PIDController(0.1, 0.0, 0.0);
    this.startingRotation = -drivetrainSubsystem.getGyroscopeRotation().getRadians();
    this.targetRotation = rotation;
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    innitTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x, y, rotationPID.calculate(-drivetrainSubsystem.getGyroscopeRotation().getRadians(), targetRotation), drivetrainSubsystem.getGyroscopeRotation());
    this.swerveDriveOdometry.update(drivetrainSubsystem.getGyroscopeRotation(), drivetrainSubsystem.m_kinematics.toSwerveModuleStates(chassisSpeeds));
    // System.out.println(drivetrainSubsystem.getGyroscopeRotation());
    // System.out.println(chassisSpeeds);
    System.out.println(-DrivetrainSubsystem.m_navx.getFusedHeading());
    System.out.println(Timer.getFPGATimestamp() + "," + swerveDriveOdometry.getPoseMeters().getX() + "," + swerveDriveOdometry.getPoseMeters().getY() + ", " + swerveDriveOdometry.getPoseMeters().getRotation() + ", " + swerveDriveOdometry.getPoseMeters().getRotation().getRadians());
    drivetrainSubsystem.drive(chassisSpeeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, drivetrainSubsystem.getGyroscopeRotation()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(Timer.getFPGATimestamp() - innitTime > 1 ) {
      return true;
    } else {      return false;
    }
  }
}
