package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.drive.DriveSwerveDrivetrain;
import java.util.Optional;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class PhotonVision extends SubsystemBase {

  private final PhotonCamera camera;
  private final PhotonPoseEstimator photonEstimator;
  private final Distance maxDistance;
  private final double maxAmbiguity;

  // Simulation support
  private VisionSystemSim visionSim;
  private PhotonCameraSim cameraSim;
  // END simulation support

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

    this.photonEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);

    this.maxDistance = maxDistance;
    this.maxAmbiguity = maxAmbiguity;

    if (Robot.isSimulation()) {
      visionSim = new VisionSystemSim(cameraName);
      visionSim.addAprilTags(fieldLayout);

      // You could put these properties in constants to be pretty but it's comp and not worth it imo
      SimCameraProperties cameraProp = new SimCameraProperties();
      cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(90));
      // Approximate detection noise with average and standard deviation error in pixels.
      cameraProp.setCalibError(0.25, 0.08);
      // Set the camera image capture framerate (Note: this is limited by robot loop rate).
      cameraProp.setFPS(60);
      // The average and standard deviation in milliseconds of image data latency.
      cameraProp.setAvgLatencyMs(35);
      cameraProp.setLatencyStdDevMs(5);

      cameraSim = new PhotonCameraSim(camera, cameraProp);
      visionSim.addCamera(cameraSim, robotToCamera);
    }
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

    for (var res : camera.getAllUnreadResults()) {

      visionEst = photonEstimator.estimateCoprocMultiTagPose(res);
      if (visionEst.isEmpty()) {
        visionEst = photonEstimator.estimateLowestAmbiguityPose(res);
      }

      if (visionEst.isPresent()) {
        var target = res.getBestTarget();
        double ambiguity = target.getPoseAmbiguity();
        double distance = target.getBestCameraToTarget().getTranslation().getNorm();

        if (!(overrideCheck || (ambiguity < maxAmbiguity && distance < maxDistance.in(Meters)))) {
          visionEst = Optional.empty();
        }
      }
    }

    return visionEst;
  }

  @Override
  public void simulationPeriodic() {
    // this is an arbitrary pose that allows the simulated camera to see tags on the field
    // so we can test vision updates in simulation.
    // The default pose is (2, 2), and with vision the bot's pose should be updated to (10, 10)
    if (RobotState.isDisabled()) {
      visionSim.update(new Pose2d(2.5, 2.5, new Rotation2d()));
    }
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

    if (visionEst.isPresent()) {
      EstimatedRobotPose estimatedPose = visionEst.get();
      Logger.recordOutput("Vision/EstimatedPose", estimatedPose.estimatedPose);

      drivetrain.addVisionMeasurement(
          estimatedPose.estimatedPose.toPose2d(),
          Utils.fpgaToCurrentTime(estimatedPose.timestampSeconds));
    }
  }
}
