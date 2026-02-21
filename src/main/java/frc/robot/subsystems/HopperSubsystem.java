// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  private Servo m_gate;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {

    m_gate = new Servo(HopperConstants.kGatePort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command gateOpencmd() {
    return Commands.run(() -> {System.out.print("open gate"); 
    m_gate.setAngle(HopperConstants.kGateOpenDegrees);}
    , this);
  }

  public Command gateClosecmd() {
    return Commands.runOnce(() -> {System.out.print("close gate"); m_gate.setAngle(HopperConstants.kGateClosedDegrees);}
    , this);
  }  
}
