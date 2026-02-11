// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  private final ADIS16470_IMU m_gyro = new ADIS16470_IMU(ADIS16470_IMU.IMUAxis.kZ, ADIS16470_IMU.IMUAxis.kX,
      ADIS16470_IMU.IMUAxis.kY);

  // Odometry class for tracking robot pose
  /*
   * SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
   * DriveConstants.kDriveKinematics,
   * Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
   * new SwerveModulePosition[] {
   * m_frontLeft.getPosition(),
   * m_frontRight.getPosition(),
   * m_rearLeft.getPosition(),
   * m_rearRight.getPosition()
   * });
   */

  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      getHeading(),
      getModulePositions(),
      new Pose2d());
  private final Field2d m_field = new Field2d();

  private final CameraApriltag m_CameraFront;

  private double m_robotPoseToHubAngle;

  private PIDController m_turnPIDCntrl = 
    new PIDController(DriveConstants.kTurnP, DriveConstants.kTurnI, DriveConstants.kTurnD);

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(CameraApriltag m_CameraApriltag) {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    m_CameraFront = m_CameraApriltag;

    RobotConfig config = null;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE
                                                              // ChassisSpeeds
        new PPHolonomicDriveController( // HolonomicPathFollowerConfig
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
        ),
        config,
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    SmartDashboard.putData("Field", m_field);
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    /*
     * m_odometry.update(
     * Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
     * new SwerveModulePosition[] {
     * m_frontLeft.getPosition(),
     * m_frontRight.getPosition(),
     * m_rearLeft.getPosition(),
     * m_rearRight.getPosition()
     * });
     */

    SmartDashboard.putBoolean("use Camera Pose", m_CameraFront.useCameraPose());
    SmartDashboard.putBoolean("use Camera Yaw", m_CameraFront.useCameraYaw());

    if (m_CameraFront.useCameraPose()) {
      updatePose();
    }

    if (m_CameraFront.useCameraYaw()) {
      setYaw();
    }

    m_poseEstimator.update(
        getHeading(),
        getModulePositions());

    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    SmartDashboard.putNumber("Gyro", getHeading().getDegrees());
    SmartDashboard.putBoolean("Have Apriltag", m_CameraFront.hasTarget());
    SmartDashboard.putNumber("Tag ID", m_CameraFront.getTargetID());

    double robotPoseX = m_poseEstimator.getEstimatedPosition().getX();
    double robotPoseY = m_poseEstimator.getEstimatedPosition().getY();
    double robotPoseDeg = m_poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    

    SmartDashboard.putNumber("Robot Pose X", robotPoseX);
    SmartDashboard.putNumber("Robot Pose Y", robotPoseY);
    SmartDashboard.putNumber("Robot Pose Degrees", robotPoseDeg);

    double hubX = 0;
    double hubY = 0;
    double hubPose = 0;

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()) {
      if (alliance.get() == DriverStation.Alliance.Red) {
        hubX = DriveConstants.kHubRedX;
        hubY = DriveConstants.kHubRedY;
        hubPose = DriveConstants.kHubRedPose;
      } else {
        hubX = DriveConstants.kHubBlueX;
        hubY = DriveConstants.kHubBlueY;
        hubPose = DriveConstants.kHubBluePose;
      }
    }

    double robotToHubX = m_poseEstimator.getEstimatedPosition().getX() - hubX;
    double robotToHubY = m_poseEstimator.getEstimatedPosition().getY() - hubY;
    double robotToHubDistance = Math.sqrt(robotToHubX * robotToHubX + robotToHubY * robotToHubY);
    double robotToHubAngle = Math.toDegrees(Math.atan(robotToHubY / robotToHubX));
    m_robotPoseToHubAngle = Math.abs(robotPoseDeg - robotToHubAngle) - hubPose;

    boolean isShootAngle = Math.abs(m_robotPoseToHubAngle) < 5;

    SmartDashboard.putNumber("To Hub X", robotToHubX);
    SmartDashboard.putNumber("To Hub Y", robotToHubY);
    SmartDashboard.putNumber("To Hub Angle", robotToHubAngle);
    SmartDashboard.putNumber("To Hub Distance", robotToHubDistance);
    SmartDashboard.putNumber("Pose to Hub Angle" , m_robotPoseToHubAngle);
    SmartDashboard.putBoolean("Shooting Angle", isShootAngle);

    /*
     * Pose2d hubPose2d = new Pose2d(hubX , hubY , new Rotation2d());
     * Transform2d toHub = new Transform2d(m_poseEstimator.getEstimatedPosition() ,
     * hubPose2d);
     * 
     * SmartDashboard.putNumber("To Hub X" , toHub.getX());
     * SmartDashboard.putNumber("To Hub Y" , toHub.getY());
     * SmartDashboard.putNumber("To Hub Angle" , toHub.getRotation().getDegrees());
     */
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        getHeading(),
        getModulePositions(),
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain

    xSpeed = squareAxis(xSpeed);
    ySpeed = squareAxis(ySpeed);
    rot = squareAxis(rot);

    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // squares value of drive joystick
  private double squareAxis(double axis) {
    return Math.copySign(axis * axis, axis);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  private void setYaw() {
    if (m_CameraFront.hasTarget()) {
      m_gyro.setGyroAngle(m_gyro.getYawAxis(),
          m_CameraFront.getVisionPose().get().estimatedPose.toPose2d().getRotation().getDegrees());
      m_CameraFront.setUseCameraYaw(false);
      System.out.println("Update Yaw ");
    } else {
      System.out.println("Update Yaw Failed ");
    }
  }

  private SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    };
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle());
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  private void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds, false);
  }

  private void drive(ChassisSpeeds speeds, boolean fieldRelative) {
    if (fieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());
    // speeds = ChassisSpeeds.discretize(speeds, LoggedRobot.defaultPeriodSecs);
    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }

  private ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  private SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
    };
  }

  // PATHPLANNER

  public Command getPathStep(String pathName) {
    // return Commands.print("drive path " + pathName);
    return new PathPlannerAuto(pathName);
  }

  public void addVisionMeasurment() {
    if (m_CameraFront.hasTarget()) {
      m_poseEstimator.addVisionMeasurement(m_CameraFront.getVisionPose2d(),
          m_CameraFront.getVisionTmst(),
          m_CameraFront.getEstimationStdDevs());
    }
  }

  private void updatePose() {
    if (m_CameraFront.hasTarget()) {
      m_poseEstimator.resetPose(m_CameraFront.getVisionPose().get().estimatedPose.toPose2d());
      m_CameraFront.setUseCameraPose(false);
      System.out.println("Update Pose ");
    } else {
      System.out.println("Update Pose Failed");
    }
  }

  /*
   * public Command updatePosecmd() {
   * return Commands.runOnce(() -> updatePose(), this)
   * .ignoringDisable(true);
   * }
   */

  private void turnInit() {
    m_turnPIDCntrl.setTolerance(DriveConstants.kTurnTolerance);
    double turnGoalAngle = m_gyro.getAngle() + m_robotPoseToHubAngle;
    m_turnPIDCntrl.reset();
    m_turnPIDCntrl.setSetpoint(turnGoalAngle);
  }

  private void turnExec() {
    double omegaDegPerSec = MathUtil.clamp(m_turnPIDCntrl.calculate(m_gyro.getAngle()), -22.5, 22.5);
    omegaDegPerSec = (omegaDegPerSec < 0) ?
      omegaDegPerSec - DriveConstants.kMinOmega:
      omegaDegPerSec + DriveConstants.kMinOmega;
    drive(0, 0, omegaDegPerSec, false);
  }

  private void turnEnd(boolean interrupted) {
    System.out.println("Align Turn Ended");
  }

  private boolean turnIsFinished() {
    return m_turnPIDCntrl.atSetpoint();
  }

  public Command cmdTurnToHub() {
    return new FunctionalCommand(
      () -> turnInit(),
      () -> turnExec(),
      (Interrupted) -> turnEnd(Interrupted),
      () -> turnIsFinished(),
      this);
    }
}
