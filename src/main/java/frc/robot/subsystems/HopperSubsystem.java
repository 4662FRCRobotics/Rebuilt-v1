// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

//import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HopperConstants;

public class HopperSubsystem extends SubsystemBase {

  //private Servo m_gate;
  private TalonSRX m_agitator;
  private TalonSRXConfiguration m_agitatorConfig;
  /** Creates a new HopperSubsystem. */
  public HopperSubsystem() {

   // m_gate = new Servo(HopperConstants.kGatePort);
    m_agitator = new TalonSRX(HopperConstants.kAgitatorCanID);
    m_agitatorConfig = new TalonSRXConfiguration();
    m_agitatorConfig.continuousCurrentLimit = 20;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command gateOpencmd() {
    return Commands.run(() -> { 
   // m_gate.setAngle(HopperConstants.kGateOpenDegrees); 
    m_agitator.set(TalonSRXControlMode.PercentOutput , 0.75);}
    , this);
  }

  public Command gateClosecmd() {
    return Commands.runOnce(() -> {//m_gate.setAngle(HopperConstants.kGateClosedDegrees); 
      m_agitator.set(TalonSRXControlMode.PercentOutput, 0);}
    , this);
  }  

  public Command reverseSpindexer() {
    return Commands.run(() -> { m_agitator.set(TalonSRXControlMode.PercentOutput , (0.75) * -1);} , this);
  }
}
