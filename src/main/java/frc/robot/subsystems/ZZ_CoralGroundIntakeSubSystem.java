// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ZZ_CoralGroundIntakeSubSystem extends SubsystemBase {
  /** Creates a new CoralGroundIntake. */

  public TalonFX intakeMotor;

  public ZZ_CoralGroundIntakeSubSystem() {
    intakeMotor=new TalonFX(Constants.CoralGroundIntakeSubSystemConstants.CAN_ID_INTAKE);

  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }
  public double getEncoder(){
    return intakeMotor.getPosition().getValueAsDouble();
  }

  @Override 
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
