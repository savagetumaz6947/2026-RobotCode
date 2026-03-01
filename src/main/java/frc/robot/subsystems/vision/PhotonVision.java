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

    // IMPORTANT: This constructor signature matches your installed PhotonVision API
    // (no PhotonCamera parameter in constructor).
    this.photonEstimator =
        new PhotonPoseEstimator(fieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

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
    Optional<EstimatedRobotPose> best = Optional.empty();

    for (var res : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> estimate = photonEstimator.estimateCoprocMultiTagPose(res);

      if (estimate.isEmpty()) continue;

      var target = res.getBestTarget();
      double ambiguity = target.getPoseAmbiguity();
      double distance = target.getBestCameraToTarget().getTranslation().getNorm();

      if (overrideCheck
          || (ambiguity < maxAmbiguity && distance < maxDistance.in(Meters))) {
        // Keep the latest acceptable measurement from unread results
        best = estimate;
      }
    }

    return best;
  }

  /**
   * Call periodically to push vision into the Phoenix swerve estimator.
   *
   * This does NOT overwrite pose; it fuses as a measurement via
   * DriveSwerveDrivetrain.addVisionMeasurement().
   */
  public void updateVision(DriveSwerveDrivetrain drivetrain) {
    // Iterate unread results so we don't build up latency
    for (var res : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> estimate = photonEstimator.estimateCoprocMultiTagPose(res);
      if (estimate.isEmpty()) continue;

      var e = estimate.get();

      // Extra safety
      if (e.targetsUsed == null || e.targetsUsed.isEmpty()) continue;

      var bestTarget = res.getBestTarget();
      double ambiguity = bestTarget.getPoseAmbiguity();
      double distance = bestTarget.getBestCameraToTarget().getTranslation().getNorm();

      if (ambiguity > maxAmbiguity) continue;
      if (distance > maxDistance.in(Meters)) continue;

      drivetrain.addVisionMeasurement(e.estimatedPose.toPose2d(), e.timestampSeconds);
    }
  }
}