// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.CoralOutakeSubSystem;
import frc.robot.subsystems.ElevatorSubSystem;
import frc.robot.subsystems.ScoringLimeLightSubSystemLeft;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZZ_AutoTargetScoreRightPoleL4Sequence extends SequentialCommandGroup {
  /** Creates a new AutoTargetScoreL4Sequence. */
  public ZZ_AutoTargetScoreRightPoleL4Sequence(ScoringLimeLightSubSystemLeft limelightLeft, CommandSwerveDrivetrain drivetrain, ElevatorSubSystem elevator, CoralOutakeSubSystem coralOutake, SwerveRequest.RobotCentric robotCentric, double MaxSpeed, double MaxAngularRate) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ZZ_FullAlignLeftLimeLight(limelightLeft, drivetrain, robotCentric, MaxSpeed, MaxAngularRate),
      new ZZ_AutoDeliverCommand(new ElevatorLevelFourCommand(elevator), elevator, coralOutake, 70.65) //70.5
    );
  }
}