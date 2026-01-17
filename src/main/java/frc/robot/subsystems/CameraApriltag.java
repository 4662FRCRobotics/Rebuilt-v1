// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraApriltag extends SubsystemBase {

  public enum CameraName {
    CAMERA_ONE("Arducam_OV9281_USB_Camera" , 0 , 0 , 0)
    ;

    private final String m_cameraName;
    private final double m_x;
    private final double m_y;
    private final double m_z;

    private CameraName(String CameraName , double x , double y , double z) {
      this.m_cameraName = CameraName;
      this.m_x = x;
      this.m_y = y;
      this.m_z = z;
    }

    public String getCameraName() {
      return m_cameraName;
    }

    public double getX() {
      return m_x;
    }
    
    public double getY() {
      return m_y;
    }

    public double getZ() {
      return m_z;
    }
  }

  PhotonCamera m_aprilCameraOne;
  boolean m_hasTargets = false;
  PhotonTrackedTarget m_target = new PhotonTrackedTarget();
  private static final AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
  private Transform3d m_robotToCam;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Optional<EstimatedRobotPose> m_visionEst = Optional.empty();
  private int m_targetID = 0;

  /** Creates a new CameraApriltag. */
  public CameraApriltag(CameraName name) {
    m_aprilCameraOne = new PhotonCamera(name.getCameraName());
    m_aprilCameraOne.setDriverMode(false);
    m_robotToCam = new Transform3d(new Translation3d(name.getX() , name.getY() , name.getZ()) , new Rotation3d(0 , 0 , 0));
    m_photonPoseEstimator = new PhotonPoseEstimator(m_fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, m_robotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double yaw = 0;
    double pitch = 0;
    double area = 0;
    double skew = 0;
    Optional<Pose3d> target3d;
    //Optional<EstimatedRobotPose> visionEst = Optional.empty();
    var april1Result = m_aprilCameraOne.getLatestResult();
    m_hasTargets = april1Result.hasTargets();
    if (m_hasTargets) {
      m_target = april1Result.getBestTarget();
      m_visionEst = m_photonPoseEstimator.update(april1Result);
      yaw = m_target.getYaw();
      pitch = m_target.getPitch();
      area = m_target.getArea();
      skew = m_target.getSkew();
      m_targetID = m_target.getFiducialId();
      target3d = m_fieldLayout.getTagPose(m_targetID);
    /*  if (visionEst.isPresent()) {
        target3d = Optional.of(visionEst.get().estimatedPose);
      }*/
    } else { 
      target3d = Optional.empty();
      m_visionEst = Optional.empty();
    }
  }
  //method to return hastarget
  public boolean hasTarget() {
    return m_hasTargets;
  }
  //method to return target id
  public int targetID() {
    return m_targetID;
  }
  //method to return the vision estimate
  public Optional<EstimatedRobotPose> getVisionPose() {
    return m_visionEst;
  }
}
