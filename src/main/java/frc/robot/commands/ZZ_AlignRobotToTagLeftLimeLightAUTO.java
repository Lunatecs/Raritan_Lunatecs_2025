// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SetDoubleSupplier;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ScoringLimeLightSubSystemLeft;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZZ_AlignRobotToTagLeftLimeLightAUTO extends Command {
  /** Creates a new AlignRobotToTag. */
  PIDController pidStrafe;
  PIDController pidTranslate;
  PIDController pidRotation;
  ScoringLimeLightSubSystemLeft limelight;
  CommandSwerveDrivetrain drivetrain;
  SwerveRequest.RobotCentric drive;
  double MaxSpeed;
  double MaxAngularRate;
  boolean isFinished;
  PPHolonomicDriveController driveController;
  SetDoubleSupplier rotationDoubleSupplier = new SetDoubleSupplier();
  SetDoubleSupplier translationDoubleSupplier = new SetDoubleSupplier();
  SetDoubleSupplier strafeDoubleSupplier = new SetDoubleSupplier();

  public ZZ_AlignRobotToTagLeftLimeLightAUTO(ScoringLimeLightSubSystemLeft limelight, CommandSwerveDrivetrain drivetrain, SwerveRequest.RobotCentric robotCentric, double MaxSpeed, double MaxAngularRate, PPHolonomicDriveController driveController) {
    // Use addRequirements() here to declare subsystem dependencies.
    pidStrafe = new PIDController(.55, 0, 0); // Horizontal PID (NEEDS TO BE TUNED BETTER)
    pidStrafe.setSetpoint(-0.045);
    pidStrafe.setTolerance(0.001);
    pidTranslate = new PIDController(.5, 0, 0); // Forward/Backward PID (NEEDS TO BE TUNED BETTER)
    pidTranslate.setSetpoint(-0.40);
    pidTranslate.setTolerance(0.1);
    pidRotation = new PIDController(.03, 0, 0); // Rotation PID
    pidRotation.setSetpoint(0);
    pidRotation.setTolerance(0.0);
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.drive = robotCentric;
    this.MaxSpeed = MaxSpeed;
    this.MaxAngularRate = MaxAngularRate;
    driveController.overrideRotationFeedback(rotationDoubleSupplier);
    driveController.overrideXFeedback(translationDoubleSupplier);
    driveController.overrideYFeedback(strafeDoubleSupplier);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isFinished = false;
    this.rotationDoubleSupplier.set(0);
    this.strafeDoubleSupplier.set(0);
    this.translationDoubleSupplier.set(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double strafe = pidStrafe.calculate(limelight.getTranslationX());
    final double strafeSpeed = MathUtil.clamp(strafe, -1.0, 1.0) * MaxSpeed;
    SmartDashboard.putNumber("strafe", strafeSpeed);

    double translate = pidTranslate.calculate(limelight.getTranslationY());
    final double translateSpeed = -MathUtil.clamp(translate, -1.0, 1.0) * MaxSpeed;
    SmartDashboard.putNumber("translate", translateSpeed);

    double rotation = pidRotation.calculate(limelight.getYaw());
    final double rotationSpeed = MathUtil.clamp(rotation, -1.0, 1.0) * MaxAngularRate;
    SmartDashboard.putNumber("rotation", rotationSpeed);

    SmartDashboard.putBoolean("RIGHT POLE TRANSLATION AT SETPOINT", pidTranslate.atSetpoint());
    SmartDashboard.putBoolean("RIGHT POLE STRAFE AT SETPOINT", pidStrafe.atSetpoint());

    if(pidRotation.atSetpoint()){
      //drive.withRotationalRate(0);
      this.rotationDoubleSupplier.set(0);
    } else {
      this.rotationDoubleSupplier.set(-rotationSpeed);
      //drive.withRotationalRate(-rotationSpeed);
    }
    if(pidStrafe.atSetpoint()) {
      //drive.withVelocityY(0);
      this.strafeDoubleSupplier.set(0);
    } else {
      //drive.withVelocityY(-strafeSpeed);
      this.strafeDoubleSupplier.set(-strafeSpeed);
    }
    if(pidTranslate.atSetpoint()) {
      //drive.withVelocityX(0);
      this.translationDoubleSupplier.set(0);
    } else {
      //drive.withVelocityX(-translateSpeed);
      this.translationDoubleSupplier.set(-translateSpeed);
    }
    //drivetrain.setControl(drive);

    if (pidTranslate.atSetpoint() && pidStrafe.atSetpoint()) {
      isFinished = true;
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}
