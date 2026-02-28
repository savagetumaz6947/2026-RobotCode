package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVision extends SubsystemBase {

  private PhotonCamera camera;
  private PhotonPoseEstimator photonEstimator;
  private Distance maxDistance;
  private double maxAmbiguity;

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
  }

  public PhotonCamera getCamera() {
    return camera;
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(boolean overrideCheck) {
    Optional<EstimatedRobotPose> visionEst = Optional.empty();

    for (var res : camera.getAllUnreadResults()) {
      Optional<EstimatedRobotPose> photonPose = photonEstimator.estimateCoprocMultiTagPose(res);

      if (photonPose.isPresent()) {
        var target = res.getBestTarget();
        double targetDistance = target.getBestCameraToTarget().getTranslation().getNorm();
        if (overrideCheck
            || (target.getPoseAmbiguity() < maxAmbiguity
                && targetDistance < maxDistance.in(Meters))) {
          visionEst = photonPose;
        }
      }
    }

    return visionEst;
  }
}
