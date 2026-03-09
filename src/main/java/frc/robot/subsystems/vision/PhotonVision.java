package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final Distance maxDistance;
  private final double maxAmbiguity;

  /** Convenience ctor: identity transform */
  public PhotonVision(String cameraName) {
    this(cameraName, new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
  }

  /** Convenience ctor: default field + default filters */
  public PhotonVision(String cameraName, Transform3d robotToCamera) {
    this(
        cameraName,
        robotToCamera,
        AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
        Meters.of(5.0), // reasonable default distance limit
        0.20 // reasonable default ambiguity limit
        );
  }

  /** Full ctor */
  public PhotonVision(
      String cameraName,
      Transform3d robotToCamera,
      AprilTagFieldLayout fieldLayout,
      Distance maxDistance,
      double maxAmbiguity) {

    this.camera = new PhotonCamera(cameraName);

    this.photonEstimator =
        new PhotonPoseEstimator(
            fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

    this.maxDistance = maxDistance;
    this.maxAmbiguity = maxAmbiguity;
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  /**
   * Returns the newest acceptable EstimatedRobotPose found in unread results.
   *
   * @param overrideCheck if true, skips ambiguity+distance filtering
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean overrideCheck) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    Logger.recordOutput("Vision/UnreadResults", camera.getAllUnreadResults().size());
    Logger.recordOutput(
        "Vision/HasTargets", false); // default to false, set to true if any results have targets

    for (var res : camera.getAllUnreadResults()) {
      Logger.recordOutput("Vision/CameraName", camera.getName());
      Logger.recordOutput("Vision/PipelineIndex", camera.getPipelineIndex());
      Logger.recordOutput("Vision/HasTargets", res.hasTargets());
      Logger.recordOutput("Vision/TargetCount", res.getTargets().size());

      Optional<EstimatedRobotPose> photonPose = photonEstimator.estimateCoprocMultiTagPose(res);

      if (photonPose.isPresent()) {
        var target = res.getBestTarget();
        double ambiguity = target.getPoseAmbiguity();
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();

        if (overrideCheck || (ambiguity < maxAmbiguity && distance < maxDistance.in(Meters))) {
          // Keep the latest acceptable measurement from unread results
          visionEst = photonPose;
        }
      }
    }

    return visionEst;
  }

  /**
   * Call periodically to push vision into the Phoenix swerve estimator.
   *
   * <p>This does NOT overwrite pose; it fuses as a measurement via
   * DriveSwerveDrivetrain.addVisionMeasurement().
   */
  public void updateVision(DriveSwerveDrivetrain drivetrain) {
    Logger.recordOutput("Vision/PeriodicRunning", true);
    Logger.recordOutput("Vision/CameraConnected", camera.isConnected());

    Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose(false);
    Logger.recordOutput("Vision/EstimatePresent", visionEst.isPresent());
    boolean addedMeasurement = false;

    if (visionEst.isPresent()) {
      EstimatedRobotPose estimatedPose = visionEst.get();
      Logger.recordOutput("Vision/EstimatedPose", estimatedPose.estimatedPose);

      drivetrain.addVisionMeasurement(
          estimatedPose.estimatedPose.toPose2d(), estimatedPose.timestampSeconds);
      addedMeasurement = true;
    }
    Logger.recordOutput("Vision/AddedMeasurement", addedMeasurement);
  }
}
