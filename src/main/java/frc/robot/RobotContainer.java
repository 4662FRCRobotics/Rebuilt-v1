// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.HopperConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.libraries.ConsoleAuto;
import frc.robot.subsystems.AutonomousSubsystem;
import frc.robot.subsystems.CameraApriltag;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.CameraApriltag.CameraName;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CommandGenericHID m_ConsoleTeleop = new CommandGenericHID(OperatorConstants.kConsoleTeleopPort);

  private final CameraApriltag m_CameraFront = new CameraApriltag(CameraName.CAMERA_ONE);
  private final DriveSubsystem m_DriveSubsystem = new DriveSubsystem(m_CameraFront);
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final HopperSubsystem m_HopperSubsystem = new HopperSubsystem();
  // private final ShooterSubsystem m_ShooterSubsystem = new
  // ShooterSubsystem(m_DriveSubsystem.getDistanceToHub());
  private final ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem(
      () -> m_ConsoleTeleop.getHID().getRawAxis(0));
  private final ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();
  private final ConsoleAuto m_ConsoleAuto = new ConsoleAuto(OperatorConstants.kAutoConsolePort);
  private final AutonomousSubsystem m_AutonomousSubsystem = new AutonomousSubsystem(m_ConsoleAuto, this,
      m_DriveSubsystem, m_ShooterSubsystem, m_IntakeSubsystem, m_ClimberSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // private final CommandGenericHID m_ConsoleTeleop =
  // new CommandGenericHID(OperatorConstants.kConsoleTeleopPort);

  private static boolean m_runAutoConsole;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    WebServer.start(5800 , Filesystem.getDeployDirectory().getPath());
    
    registerNamedCommands();
    // Configure the trigger bindings
    configureBindings();
    m_DriveSubsystem.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_DriveSubsystem.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OperatorConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OperatorConstants.kDriveDeadband),
                m_ConsoleTeleop.getHID().getRawButton(OperatorConstants.kFieldDriveButton)),
            m_DriveSubsystem));

    m_ShooterSubsystem.setDefaultCommand(m_ShooterSubsystem.stopShoot());

    m_IntakeSubsystem.setDefaultCommand(m_IntakeSubsystem.stopCmd());

    m_HopperSubsystem.setDefaultCommand(m_HopperSubsystem.gateClosecmd());


  }

  // Pathplanner Events
  private void registerNamedCommands() {
    NamedCommands.registerCommand("Camera Pose Reset", m_CameraFront.cmdUseCameraPose());
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    runAutoConsoleFalse();
    // new Trigger(DriverStation::isDisabled)
    // new Trigger(RobotModeTriggers.disabled())
    new Trigger(trgAutoSelect())
        .whileTrue(m_AutonomousSubsystem.selectAuto());
    runAutoConsoleTrue();

    new Trigger(RobotModeTriggers.disabled())
        .onTrue(Commands.runOnce(this::runAutoConsoleTrue)
            .ignoringDisable(true));

    new Trigger(RobotModeTriggers.disabled())
        .onFalse(Commands.runOnce(this::runAutoConsoleFalse));

    m_driverController.x()
        .whileTrue(new RunCommand(
            () -> m_DriveSubsystem.setX(),
            m_DriveSubsystem));

    /*
     * m_driverController.y()
     * .onTrue(new InstantCommand(
     * () -> m_DriveSubsystem.zeroHeading(),
     * m_DriveSubsystem));
     */

    m_driverController.leftTrigger()
        .whileTrue(m_IntakeSubsystem.deployCmd().andThen(m_IntakeSubsystem.runInCmd()));

    m_driverController.leftBumper()
      .whileTrue(m_IntakeSubsystem.retractCmd());

    m_driverController.rightTrigger()
        .whileTrue(m_DriveSubsystem.cmdTurnToHub()
            .unless(() -> m_driverController.getHID().getBButton())
            .andThen(Commands.parallel(m_ShooterSubsystem.shoot() , 
            Commands.sequence(Commands.waitSeconds(HopperConstants.kGateOpenLagSeconds) , m_HopperSubsystem.gateOpencmd()))));

    m_driverController.povUp()
        .whileTrue(m_ClimberSubsystem.extend().andThen(m_ClimberSubsystem.stop()));

    m_driverController.povDown()
        .whileTrue(m_ClimberSubsystem.retract().andThen(m_ClimberSubsystem.stop()));

    m_ConsoleTeleop.button(2)
        .onTrue(m_CameraFront.cmdUseCameraPose());

    m_ConsoleTeleop.button(3)
        .onTrue(m_CameraFront.cmdUseCameraYaw());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_AutonomousSubsystem.runAuto();
  }

  private static Trigger trgAutoSelect() {
    // System.out.println("bool auto console" + m_runAutoConsole);
    return new Trigger(() -> m_runAutoConsole);
  }

  private void runAutoConsoleTrue() {
    m_runAutoConsole = true;
  }

  private void runAutoConsoleFalse() {
    m_runAutoConsole = false;
  }
}
