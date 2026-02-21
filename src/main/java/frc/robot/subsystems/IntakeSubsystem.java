// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runInCmd() {
    return Commands.run(() -> System.out.println("Intake"), this);
  }

  public Command stopCmd() {
    return Commands.runOnce(() -> System.out.println("Stop Intake"), this);
  }

  public Command deployCmd() {
    return Commands.runOnce(() -> System.out.println("deploy Intake"), this);
  }

  public Command retractCmd(){
    return Commands.runOnce(() -> System.out.println("retractIntake"), this);
  }
}
