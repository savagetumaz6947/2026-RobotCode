package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import java.util.Optional;
import java.util.function.Consumer;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/** Simple PhotonVision wrapper that can optionally apply estimated poses to RobotState. */
public class PhotonVision extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator photonEstimator;
  private Distance maxDistance;
  private double maxAmbiguity;
  private AprilTagFieldLayout fieldLayout;
  private Transform3d robotToCamera;
  // Optional RobotState reference to apply vision updates
  private RobotState robotState = null;
  // Optional callback to apply a Pose2d to drivetrain odometry (e.g., swerveIO::setPose)
  private Consumer<Pose2d> poseApplier = null;

  public PhotonVision(String cameraName) {
    this(cameraName, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
  }

  public PhotonVision(String cameraName, Transform3d robotToCamera) {
    this(
        cameraName,
        robotToCamera,
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
        Meters.of(1),
        0.15);
  }

  /**
   * Construct with an explicit RobotState reference so this subsystem can apply pose updates in
   * periodic(). Passing null disables automatic application.
   */
  public PhotonVision(String cameraName, Transform3d robotToCamera, RobotState robotState) {
    this(cameraName, robotToCamera);
    this.robotState = robotState;
  }

  public PhotonVision(
      String cameraName,
      Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout,
      Distance maxDistance,
      double maxAmbiguity) {
    camera = new PhotonCamera(cameraName);
    photonEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    this.maxDistance = maxDistance;
    this.maxAmbiguity = maxAmbiguity;
    this.fieldLayout = fieldLayout;
    this.robotToCamera = robotToCamera;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  /**
   * Provide a callback that will be called with a Pose2d when PhotonVision applies an estimate.
   * This is used to reset drivetrain odometry so vision isn't overwritten by telemetry.
   */
  public void setPoseApplier(Consumer<Pose2d> applier) {
    this.poseApplier = applier;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean overrideCheck) {
    // Use the latest result (more reliable than unread-results iteration for continuous
    // polling). If you prefer to consume unread results, revert to the unread loop.
    var res = camera.getLatestResult();
    if (res == null || !res.hasTargets()) {
      return Optional.empty();
    }

    Optional<EstimatedRobotPose> photonPose = photonEstimator.estimateCoprocMultiTagPose(res);
    if (photonPose.isPresent()) {
      var target = res.getBestTarget();
      double targetDistance = target.getBestCameraToTarget().getTranslation().getNorm();
      if (overrideCheck
          || (target.getPoseAmbiguity() < maxAmbiguity
              && targetDistance < maxDistance.in(Meters))) {
        return photonPose;
      }
    }

    return Optional.empty();
  }

  @Override
  public void periodic() {
    // If no RobotState was provided at construction, do nothing.
    if (robotState == null) {
      return;
    }

    try {
      // minimal logging: whether targets exist

      var res = camera.getLatestResult();
      boolean hasTargets = res != null && res.hasTargets();
      Logger.recordOutput("Vision/HasTargets", hasTargets);
      if (!hasTargets) {
        return;
      }

      // keep logs minimal to avoid excess telemetry

      Optional<EstimatedRobotPose> est = photonEstimator.estimateCoprocMultiTagPose(res);
      Logger.recordOutput("Vision/HasEstimate", est.isPresent());

      if (est.isPresent()) {
        var e = est.get();
        try {
          edu.wpi.first.math.geometry.Pose3d pose3d =
              (edu.wpi.first.math.geometry.Pose3d) e.estimatedPose;
          double x = pose3d.getX();
          double y = pose3d.getY();
          double yaw = pose3d.getRotation().getZ();
          var pose2d =
              new edu.wpi.first.math.geometry.Pose2d(
                  x, y, new edu.wpi.first.math.geometry.Rotation2d(yaw));
          // Use FPGA timestamp so RobotState keeps observations in the same timebase.
          robotState.addFieldToRobot(Timer.getFPGATimestamp(), pose2d);
          Logger.recordOutput("Vision/AppliedPose", pose2d);
          if (poseApplier != null) {
            try {
              poseApplier.accept(pose2d);
              Logger.recordOutput("Vision/AppliedToDrive", true);
            } catch (Exception ignored) {
            }
          }
        } catch (Exception ignored) {
          // ignore individual estimate errors
        }
      } else {
        // Fallback: try to compute robot pose locally from the best target and the AprilTag
        // field layout. This helps if the coprocessor estimator isn't available.
        try {
          var tag = res.getBestTarget();
          int fid = tag.getFiducialId();
          var tagPoseOpt = fieldLayout.getTagPose(fid);
          if (tagPoseOpt.isPresent()) {
            var tagPose = tagPoseOpt.get();
            var cameraToTarget = tag.getBestCameraToTarget();
            var cameraToTagInv = cameraToTarget.inverse();
            var robotPose3d = tagPose.transformBy(cameraToTagInv);
            Logger.recordOutput("Vision/FallbackUsed", true);
            double x = robotPose3d.getX();
            double y = robotPose3d.getY();
            double yaw = robotPose3d.getRotation().getZ();
            var pose2d =
                new edu.wpi.first.math.geometry.Pose2d(
                    x, y, new edu.wpi.first.math.geometry.Rotation2d(yaw));
            robotState.addFieldToRobot(Timer.getFPGATimestamp(), pose2d);
            Logger.recordOutput("Vision/AppliedPose", pose2d);
            if (poseApplier != null) {
              try {
                poseApplier.accept(pose2d);
                Logger.recordOutput("Vision/AppliedToDrive", true);
              } catch (Exception ignored) {
              }
            }
          } else {
            // no-op
          }
        } catch (Exception ignored) {
          // ignore fallback errors
        }
      }
    } catch (Exception ignored) {
      // keep periodic safe
    }
  }
}
