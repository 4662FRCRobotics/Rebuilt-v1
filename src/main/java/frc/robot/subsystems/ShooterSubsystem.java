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

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Ounces;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Amps;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase {

  private SmartMotorControllerConfig m_flywheelSMCConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    //Pid constants
    .withClosedLoopController(50 , 0 , 0 , DegreesPerSecond.of(90) , DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(50 , 0 , 0 , DegreesPerSecond.of(90) , DegreesPerSecondPerSecond.of(45))
    //feed forward constants
    .withFeedforward(new SimpleMotorFeedforward(0 , 0 , 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0 , 0 , 0))
    //telemetry name and telemetry verbosity (recording and feeding back)
    .withTelemetry("flywheel Motor " , TelemetryVerbosity.HIGH)
    //gearing from motor to shaft
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
    //motor proerties
    .withMotorInverted(true)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(60));

  private SmartMotorControllerConfig m_backwheelSMCConfig = new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    //Pid constants
    .withClosedLoopController(50 , 0 , 0 , DegreesPerSecond.of(90) , DegreesPerSecondPerSecond.of(45))
    .withSimClosedLoopController(50 , 0 , 0 , DegreesPerSecond.of(90) , DegreesPerSecondPerSecond.of(45))
    //feed forward constants
    .withFeedforward(new SimpleMotorFeedforward(0 , 0 , 0))
    .withSimFeedforward(new SimpleMotorFeedforward(0 , 0 , 0))
    //telemetry name and telemetry verbosity (recording and feeding back)
    .withTelemetry("backwheel Motor " , TelemetryVerbosity.HIGH)
    //gearing from motor to shaft
    .withGearing(new MechanismGearing(GearBox.fromReductionStages(2)))
    //motor proerties
    .withMotorInverted(false)
    .withIdleMode(MotorMode.COAST)
    .withStatorCurrentLimit(Amps.of(40));


  private TalonFX m_flywheelController = new TalonFX(ShooterConstants.kFlywheelControllerCanId);

  private SparkMax m_backwheelController = new SparkMax(ShooterConstants.kBackwheelControllerCanId, MotorType.kBrushless);

  private SmartMotorController m_smartBackwheelController =
    new SparkWrapper(m_backwheelController, DCMotor.getNEO(1), m_backwheelSMCConfig);

  private SmartMotorController m_smartFlywheelController = 
    new TalonFXWrapper(m_flywheelController , DCMotor.getKrakenX60(1) , m_flywheelSMCConfig);

  private final FlyWheelConfig m_shooterFlyWheelConfig = new FlyWheelConfig(m_smartFlywheelController)
    .withDiameter(Inches.of(3))
    .withMass(Ounces.of(10.9))
    .withUpperSoftLimit(RPM.of(4000))
    .withTelemetry("Shooter Mech " , TelemetryVerbosity.HIGH);

  private final FlyWheelConfig m_backwheelwheelConfig = new FlyWheelConfig(m_smartBackwheelController)
    .withDiameter(Inches.of(2))
    .withMass(Ounces.of(8))
    .withUpperSoftLimit(RPM.of(4000))
    .withTelemetry("Backwheel Mech", TelemetryVerbosity.HIGH);

  private FlyWheel m_flywheel = new FlyWheel(m_shooterFlyWheelConfig);
  private FlyWheel m_backwheel = new FlyWheel(m_backwheelwheelConfig);

  private double m_adjustedThrottle;
  private DoubleSupplier m_distanceToHub;

  public AngularVelocity getFlywheelVelocity() {
    return m_flywheel.getSpeed();
  }

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem(DoubleSupplier distanceToHub) {
    m_distanceToHub = distanceToHub;
  }



  public Command setVelocitycmd(AngularVelocity speed) {
    //return m_flywheel.setSpeed(speed);
    return Commands.run(() -> {
      m_smartFlywheelController.setVelocity(speed);
      m_smartBackwheelController.setVelocity(speed);
    } , this)
    .withName("CustomShooterVelocity");
  }

  public Command setcmd(double dutyCycle) {
    //return m_flywheel.set(dutyCycle);
    return Commands.startRun(() -> {
      m_smartBackwheelController.stopClosedLoopController();
      m_smartFlywheelController.stopClosedLoopController();
    } , () -> {
      m_smartFlywheelController.setDutyCycle(dutyCycle);
      m_smartBackwheelController.setDutyCycle(dutyCycle);
    } , this)
    .finallyDo(() -> {
      m_smartFlywheelController.startClosedLoopController();
      m_smartBackwheelController.startClosedLoopController();
    })
    .withName("CustomShooterDutyCycle");
  }
 
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //formula below is using knob on console, will need to be changed when we get distance and camera working
     m_adjustedThrottle = (((m_distanceToHub.getAsDouble() +1.0) / 2.0) * ShooterConstants.kShooterRange) + ShooterConstants.kShooterMinVoltage;
    SmartDashboard.putBoolean("Hub Active" , isHubActive());
    m_flywheel.updateTelemetry();
    m_backwheel.updateTelemetry();
    m_smartFlywheelController.updateTelemetry();
    m_smartBackwheelController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    m_flywheel.simIterate();
    m_backwheel.simIterate();
    m_smartFlywheelController.simIterate();
    m_smartBackwheelController.simIterate();
  }

  //public Command shoot() {
  //  //return Commands.print("Shoot pew pew");
  //  return Commands.run(() -> setVoltage() , this);

  //}

  //private void setVoltage() {
  //  m_adjustedThrottle = (((voltageSupplier.getAsDouble() +1.0) / 2.0) * ShooterConstants.kShooterRange) + ShooterConstants.kShooterMinVoltage;
  //  m_shooterController.setVoltage(m_adjustedThrottle);
  //  m_backwheelController.setVoltage(m_adjustedThrottle);
  //}

  //public Command stopShoot() {
   // return Commands.print("Stop shooting");
   //return Commands.run(() -> {
   // m_shooterController.setVoltage(0);
   // m_backwheelController.setVoltage(0);
    //m_adjustedThrottle = (((voltageSupplier.getAsDouble() +1.0) / 2.0) * ShooterConstants.kShooterRange) + ShooterConstants.kShooterMinVoltage;
   //} , this);
  //}

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
