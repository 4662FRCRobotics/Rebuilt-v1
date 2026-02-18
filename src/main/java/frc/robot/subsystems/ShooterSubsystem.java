// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private final TalonFX m_shooterController;
  private TalonFXConfiguration m_shooterConfig;
  private SparkMax m_backwheelController;
  private SparkMaxConfig m_backwheelConfig;
  private double m_adjustedThrottle;
  private DoubleSupplier m_distanceToHub;

  public ShooterSubsystem(DoubleSupplier distanceToHub) {

    m_distanceToHub = distanceToHub;
    m_shooterController = new TalonFX(ShooterConstants.kFlywheelControllerCanId);
    m_shooterConfig = new TalonFXConfiguration();
    m_shooterConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    m_shooterConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod(ShooterConstants.kShooterRampRate);

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    //try config up to five times, print if failure
    for (int i = 0; i < 5; ++i) {
      status = m_shooterController.getConfigurator().apply(m_shooterConfig);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply Configs, err code: " + status.toString());
    }

    m_backwheelController = new SparkMax(ShooterConstants.kBackwheelControllerCanId , MotorType.kBrushless);
    m_backwheelConfig = new SparkMaxConfig();
    m_backwheelConfig.inverted(false);
    m_backwheelConfig.openLoopRampRate(ShooterConstants.kShooterRampRate);

    m_backwheelController.configure(m_backwheelConfig , ResetMode.kNoResetSafeParameters , PersistMode.kNoPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //formula below is using knob on console, will need to be changed when we get distance and camera working
     m_adjustedThrottle = (((m_distanceToHub.getAsDouble() +1.0) / 2.0) * ShooterConstants.kShooterRange) + ShooterConstants.kShooterMinVoltage;
    SmartDashboard.putBoolean("Hub Active" , isHubActive());
    SmartDashboard.putNumber("Shooter Voltage" , m_adjustedThrottle);
    SmartDashboard.putNumber("Shooter Velocity" , m_shooterController.getVelocity().getValueAsDouble());
  }

  public Command shoot() {
    //return Commands.print("Shoot pew pew");
    return Commands.run(() -> setVoltage() , this);

  }

  private void setVoltage() {
  //  m_adjustedThrottle = (((voltageSupplier.getAsDouble() +1.0) / 2.0) * ShooterConstants.kShooterRange) + ShooterConstants.kShooterMinVoltage;
    m_shooterController.setVoltage(m_adjustedThrottle);
    m_backwheelController.setVoltage(m_adjustedThrottle);
  }

  public Command stopShoot() {
   // return Commands.print("Stop shooting");
   return Commands.run(() -> {
    m_shooterController.setVoltage(0);
    m_backwheelController.setVoltage(0);
    //m_adjustedThrottle = (((voltageSupplier.getAsDouble() +1.0) / 2.0) * ShooterConstants.kShooterRange) + ShooterConstants.kShooterMinVoltage;
   } , this);
  }

  private boolean isHubActive() {
    var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              String myAlliance = alliance.get() == DriverStation.Alliance.Red? "R":"B";
              String firstAlliance = DriverStation.getGameSpecificMessage();
              Double matchTime = DriverStation.getMatchTime();
              if (matchTime < 30) {
                return true;
              }
              if (matchTime < 55) {
                return myAlliance.equals(firstAlliance);
              }
              if (matchTime < 80) {
                return !myAlliance.equals(firstAlliance);
              }
              if (matchTime < 105) {
                return myAlliance.equals(firstAlliance);
              }
              if (matchTime < 130) {
                return !myAlliance.equals(firstAlliance);
              }
              return true;
            }
            return false;
  }
}
