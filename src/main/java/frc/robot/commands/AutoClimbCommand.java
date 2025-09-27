// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimberSubSystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoClimbCommand extends Command {
  private ClimberSubSystem climber;
  PIDController controller;
  /** Creates a new AutoClimbCommand. */
  public AutoClimbCommand(ClimberSubSystem climber) {
    this.climber=climber;
    addRequirements(climber);
    controller = new PIDController(0.012, 0, 0); //UNTUNED
    controller.setSetpoint(-95); //DON'T KNOW THE VALUE OF THIS
    controller.setTolerance(1); //DON'T KNOW THIS EITHER


    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.calculate(climber.getEncoder());
    if(Math.abs(speed)> 0.5){
      speed = Math.signum(speed) * 0.5;
    }
    climber.setSpeed(speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climber.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
