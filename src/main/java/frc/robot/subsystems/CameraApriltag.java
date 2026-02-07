// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;

public class CameraApriltag extends SubsystemBase {

  public enum CameraName {
    CAMERA_ONE("Arducam_OV9281_USB_Camera", 0, 0, 0, Math.toRadians(25));

    private final String m_cameraName;
    private final double m_x;
    private final double m_y;
    private final double m_z;
    private final double m_pitch;

    private CameraName(String CameraName, double x, double y, double z, double pitch) {
      this.m_cameraName = CameraName;
      this.m_x = x;
      this.m_y = y;
      this.m_z = z;
      this.m_pitch = pitch;
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

    public double getPitch() {
      return m_pitch;
    }
  }

  private PhotonCamera m_aprilCameraOne;
  private boolean m_hasTargets = false;
  private PhotonTrackedTarget m_target = new PhotonTrackedTarget();
  private static final AprilTagFieldLayout m_fieldLayout = AprilTagFieldLayout
      .loadField(AprilTagFields.k2026RebuiltWelded);
  private Transform3d m_robotToCam;
  private PhotonPoseEstimator m_photonPoseEstimator;
  private Optional<EstimatedRobotPose> m_visionEst = Optional.empty();
  private int m_targetID = 0;
  private Matrix<N3, N1> m_curStdDevs;
  private boolean m_useCameraPose = false;
  private boolean m_useCameraYaw = false;

  /** Creates a new CameraApriltag. */
  public CameraApriltag(CameraName name) {
    m_aprilCameraOne = new PhotonCamera(name.getCameraName());
    m_aprilCameraOne.setDriverMode(false);
    m_robotToCam = new Transform3d(new Translation3d(name.getX(), name.getY(), name.getZ()),
        new Rotation3d(0, name.getPitch(), 0));
    m_photonPoseEstimator = new PhotonPoseEstimator(m_fieldLayout,
        m_robotToCam);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //double yaw = 0;
    //double pitch = 0;
    //double area = 0;
    //double skew = 0;
    //Optional<Pose3d> target3d;
    // Optional<EstimatedRobotPose> visionEst = Optional.empty();
    /*
     * var april1Result = m_aprilCameraOne.getLatestResult();
     * m_hasTargets = april1Result.hasTargets();
     * if (m_hasTargets) {
     * m_target = april1Result.getBestTarget();
     * m_visionEst = m_photonPoseEstimator.update(april1Result);
     * yaw = m_target.getYaw();
     * pitch = m_target.getPitch();
     * area = m_target.getArea();
     * skew = m_target.getSkew();
     * m_targetID = m_target.getFiducialId();
     * target3d = m_fieldLayout.getTagPose(m_targetID);
     * /* if (visionEst.isPresent()) {
     * target3d = Optional.of(visionEst.get().estimatedPose);
     * }
     */
    /*
     * } else {
     * target3d = Optional.empty();
     * m_visionEst = Optional.empty();
     * }
     */
    for (var result : m_aprilCameraOne.getAllUnreadResults()) {
      m_target = result.getBestTarget();
      if (m_target != null) {
        m_targetID = m_target.getFiducialId();
        m_visionEst = m_photonPoseEstimator.estimateCoprocMultiTagPose(result);
        if (m_visionEst.isEmpty()) {
          m_visionEst = m_photonPoseEstimator.estimateLowestAmbiguityPose(result);
        }
        updateEstimationStdDevs(m_visionEst, result.getTargets());
        if (m_visionEst.isPresent()) {
          m_hasTargets = true;
        } else {
          m_hasTargets = false;
        }
      } else {
        m_hasTargets = false;
        m_targetID = 0;
      }
    }

  }

  /**
   * Calculates new standard deviations This algorithm is a heuristic that creates
   * dynamic standard
   * deviations based on number of tags, estimation strategy, and distance from
   * the tags.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for.
   * @param targets       All targets in this camera frame
   */
  private void updateEstimationStdDevs(
      Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
    if (estimatedPose.isEmpty()) {
      // No pose input. Default to single-tag std devs
      m_curStdDevs = CameraConstants.kSingleTagStdDevs;

    } else {
      // Pose present. Start running Heuristic
      var estStdDevs = CameraConstants.kSingleTagStdDevs;
      int numTags = 0;
      double avgDist = 0;

      // Precalculation - see how many tags we found, and calculate an
      // average-distance metric
      for (var tgt : targets) {
        var tagPose = m_photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
        if (tagPose.isEmpty())
          continue;
        numTags++;
        avgDist += tagPose
            .get()
            .toPose2d()
            .getTranslation()
            .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
      }

      if (numTags == 0) {
        // No tags visible. Default to single-tag std devs
        m_curStdDevs = CameraConstants.kSingleTagStdDevs;
      } else {
        // One or more tags visible, run the full heuristic.
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (numTags > 1)
          estStdDevs = CameraConstants.kMultiTagStdDevs;
        // Increase std devs based on (average) distance
        if (numTags == 1 && avgDist > 4)
          estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else
          estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
        m_curStdDevs = estStdDevs;
      }
    }
  }

  /**
   * Returns the latest standard deviations of the estimated pose from {@link
   * #getEstimatedGlobalPose()}, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
   * SwerveDrivePoseEstimator}. This should
   * only be used when there are targets visible.
   */
  public Matrix<N3, N1> getEstimationStdDevs() {
    return m_curStdDevs;
  }

  // method to return hastarget
  public boolean hasTarget() {
    return m_hasTargets;
  }

  // method to return target id
  public int getTargetID() {
    return m_targetID;
  }

  // method to return the vision estimate
  public Optional<EstimatedRobotPose> getVisionPose() {
    return m_visionEst;
  }

  public Pose2d getVisionPose2d() {
    if (m_visionEst.isPresent()) {
      return m_visionEst.get().estimatedPose.toPose2d();
    } else {
      return null;
    }
  }

  public double getVisionTmst() {
    if (m_visionEst.isPresent()) {
      return m_visionEst.get().timestampSeconds;
    } else {
      return 0;
    }
  }

  public boolean useCameraPose() {
    return this.m_useCameraPose;
  }

  public void setUseCameraPose(boolean useCameraPose) {
    this.m_useCameraPose = useCameraPose;
  }

  public Command cmdUseCameraPose() {
    return Commands.runOnce(() -> setUseCameraPose(true), this)
        .ignoringDisable(true);
  }

  public boolean useCameraYaw() {
    return this.m_useCameraYaw;
  }

  public void setUseCameraYaw(boolean useCameraYaw) {
    this.m_useCameraYaw = useCameraYaw;
  }

  public Command cmdUseCameraYaw() {
    return Commands.runOnce(() -> setUseCameraYaw(true), this)
        .ignoringDisable(true);
  }
}
