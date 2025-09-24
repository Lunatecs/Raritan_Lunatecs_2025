// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ZZ_CoralGroundIntakePivotSubSystem;


/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ZZ_RaiseIntakeCommand extends Command {
  private ZZ_CoralGroundIntakePivotSubSystem pivot;
  PIDController controller;
  /** Creates a new DropIntakeCommand. */
  public ZZ_RaiseIntakeCommand(ZZ_CoralGroundIntakePivotSubSystem pivot) {
    this.pivot = pivot;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(pivot);
    controller = new PIDController(0.0125, 0, 0.001); 
    controller.setSetpoint(25);
    controller.setTolerance(2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(pivot.getDegreesOfPivot());
    if(Math.abs(speed)> 0.8){
      speed = Math.signum(speed) * 0.8;
    }
    pivot.setSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
