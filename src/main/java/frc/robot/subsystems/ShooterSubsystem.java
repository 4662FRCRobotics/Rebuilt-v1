// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private final TalonFX m_shooterController = new TalonFX(20);

  private final VelocityVoltage m_shooterVelVolt = new VelocityVoltage(0).withSlot(0);
  private final NeutralOut m_shooterBrake = new NeutralOut();

  private SparkMax m_backwheelController;
  private SparkMaxConfig m_backwheelConfig;
  private SparkClosedLoopController m_backwheelLoopControl;
  private RelativeEncoder m_backwheelEncoder;

  public ShooterSubsystem() {
    TalonFXConfiguration shooterConfigs = new TalonFXConfiguration();

    /*
     * Voltage-based velocity requires a velocity feed forward to account for the
     * back-emf of the motor
     */
    shooterConfigs.Slot0.kS = 0.1; // To account for friction, add 0.1 V of static feedforward
    shooterConfigs.Slot0.kV = 0.12; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12
    // volts / rotation per second
    shooterConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    shooterConfigs.Slot0.kI = 0; // No output for integrated error
    shooterConfigs.Slot0.kD = 0; // No output for error derivative
    // Peak output of 8 volts
    shooterConfigs.Voltage.withPeakForwardVoltage(Volts.of(8))
        .withPeakReverseVoltage(Volts.of(-8));

    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_shooterController.getConfigurator().apply(shooterConfigs);
      if (status.isOK())
        break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    // SparkMAX controls
    m_backwheelController = new SparkMax(21, MotorType.kBrushless);
    m_backwheelLoopControl = m_backwheelController.getClosedLoopController();
    m_backwheelEncoder = m_backwheelController.getEncoder();
    m_backwheelConfig = new SparkMaxConfig();
    m_backwheelConfig.encoder
        .velocityConversionFactor(1);

    m_backwheelConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .p(0.4)
        .i(0)
        .d(0)
        .outputRange(-1, 1).feedForward
        .kV(12.0 / 5767, ClosedLoopSlot.kSlot0);

    m_backwheelConfig.closedLoop.maxMotion
        .cruiseVelocity(1000)
        .maxAcceleration(1000)
        .allowedProfileError(1);

    m_backwheelController.configure(m_backwheelConfig, ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Hub Active", isHubActive());
    SmartDashboard.putNumber("Backwheel Target", m_backwheelController.getClosedLoopController().getSetpoint());

  }

  private void setShooterVelocity(double desiredRotPerSec) {
    m_shooterController.setControl(m_shooterVelVolt.withVelocity(desiredRotPerSec));
  }

  private void stopShooter() {
    m_shooterController.setControl(m_shooterBrake);
  }

  private void setBackwheelVelocity(double targetVelocity) {
    m_backwheelLoopControl.setSetpoint(targetVelocity, ControlType.kMAXMotionVelocityControl,
        ClosedLoopSlot.kSlot0);
  }

  public Command shoot() {
    // return Commands.print("Shoot pew pew");
    return run(() -> {
      setShooterVelocity(1000);
      setBackwheelVelocity(800);
    });
  }

  public Command stopShoot() {
    // return Commands.print("Stop shooting");
    return run(() -> {
      stopShooter();
      setBackwheelVelocity(0);
    });
  }

  private boolean isHubActive() {
    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      String myAlliance = alliance.get() == DriverStation.Alliance.Red ? "R" : "B";
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
