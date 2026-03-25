// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {

  private SparkMax m_drawbridgeController;
  private SparkMaxConfig m_drawbridgeConfig;
  private SparkLimitSwitch m_drawbridgeForwardLimit;
  private SparkLimitSwitch m_drawbridgeReverseLimit;
  private SparkMax m_spinner;
  private SparkMaxConfig m_spinnerConfig;
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

    m_drawbridgeController = new SparkMax(IntakeConstants.kDrawbridgeControllerCanID , MotorType.kBrushless);
    m_drawbridgeConfig = new SparkMaxConfig();
    m_drawbridgeConfig.inverted(true);
    m_drawbridgeConfig.openLoopRampRate(IntakeConstants.kDrawbridgeRampRate);
    m_drawbridgeConfig.idleMode(IdleMode.kBrake);
    m_drawbridgeConfig.limitSwitch
      .forwardLimitSwitchType(Type.kNormallyOpen)
      .forwardLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor)
      .reverseLimitSwitchType(Type.kNormallyOpen)
      .reverseLimitSwitchTriggerBehavior(Behavior.kStopMovingMotor);

    m_drawbridgeController.configure(m_drawbridgeConfig , ResetMode.kNoResetSafeParameters , PersistMode.kNoPersistParameters);

    m_drawbridgeForwardLimit = m_drawbridgeController.getForwardLimitSwitch();
    m_drawbridgeReverseLimit = m_drawbridgeController.getReverseLimitSwitch();

    m_spinner = new SparkMax(IntakeConstants.kSpinnerControllerCanID, MotorType.kBrushless);
    m_spinnerConfig = new SparkMaxConfig();
    m_spinnerConfig.inverted(false);
    m_spinnerConfig.openLoopRampRate(IntakeConstants.kSpinnerRampRate);
    m_spinnerConfig.smartCurrentLimit(50);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command runInCmd() {
    return Commands.run(() -> m_spinner.setVoltage(IntakeConstants.kSpinnerVoltage) , this);
  }

  public Command stopCmd() {
    return Commands.run(() -> {m_drawbridgeController.setVoltage(0);
      m_spinner.setVoltage(0);} , this);
  } 

  public Command deployCmd() {
    return Commands.run(() -> m_drawbridgeController.setVoltage(IntakeConstants.kDrawbridgeVoltage * -1) , this)
    .until(() -> m_drawbridgeReverseLimit.isPressed());
  }

  public Command retractCmd(){
    return Commands.run(() -> m_drawbridgeController.setVoltage(IntakeConstants.kDrawbridgeVoltage) , this)
    .until(() -> m_drawbridgeForwardLimit.isPressed());
  }
}
