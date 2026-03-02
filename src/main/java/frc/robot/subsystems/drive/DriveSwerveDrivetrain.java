// Copyright (c) 2021-2026 Littleton Robotics
// Adapted from Team 254's 2025 code
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.subsystems.drive;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class DriveSwerveDrivetrain extends SubsystemBase {

  private final DriveIOHardware driveIO;
  private final RobotState robotState;

  // SwerveRequest objects for different drive modes
  private final SwerveRequest.FieldCentric fieldCentricDrive = new SwerveRequest.FieldCentric();
  private final SwerveRequest.RobotCentric robotCentricDrive = new SwerveRequest.RobotCentric();

  public DriveSwerveDrivetrain(DriveIOHardware driveIO, RobotState robotState) {
    this.driveIO = driveIO;
    this.robotState = robotState;

    configureAutoBuilder();
  }

  @Override
  public void periodic() {
    // Update chassis speeds in RobotState
    var robotRelativeSpeed = driveIO.getState().Speeds;
    var fieldRelativeSpeed =
        ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeed, getPose().getRotation());
    robotState.updateChassisSpeeds(fieldRelativeSpeed, robotRelativeSpeed);

    // Log robot state
    robotState.log();

    // Log simulated pose if using DriveIOSim
    if (driveIO instanceof DriveIOSim) {
      ((DriveIOSim) driveIO).logSimulatedPose();
    }
  }

  /** Get the current robot pose from RobotState (matches 254) */
  public Pose2d getPose() {
    if (robotState.getLatestFieldToRobot() != null) {
      return robotState.getLatestFieldToRobot().getValue();
    }
    return driveIO.getState().Pose; // Fallback
  }

  /** Get RobotState object for commands */
  public RobotState getRobotState() {
    return robotState;
  }

  /** Reset the robot's pose. */
  public void setPose(Pose2d pose) {
    driveIO.resetOdometry(pose);
  }

  public void resetPose(Pose2d pose) {
    setPose(pose);
  }

  /** Inject a vision measurement into Phoenix's internal estimator */
  public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
    driveIO.addVisionMeasurement(pose, timestampSeconds);
  }

  /**
   * Drive the robot in field-relative mode.
   *
   * @param vx Forward velocity in m/s
   * @param vy Sideways velocity in m/s
   * @param omega Rotational velocity in rad/s
   */
  public void driveFieldRelative(double vx, double vy, double omega) {
    driveIO.setControl(
        fieldCentricDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  /**
   * Drive the robot in robot-relative mode.
   *
   * @param vx Forward velocity in m/s
   * @param vy Sideways velocity in m/s
   * @param omega Rotational velocity in rad/s
   */
  public void driveRobotRelative(double vx, double vy, double omega) {
    driveIO.setControl(
        robotCentricDrive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega));
  }

  /** Stop the drivetrain. */
  public void stop() {
    driveRobotRelative(0.0, 0.0, 0.0);
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return driveIO.getState().Speeds;
  }

  /** Drive using chassis speeds. */
  public void runVelocity(ChassisSpeeds chassisSpeeds) {
    driveRobotRelative(
        chassisSpeeds.vxMetersPerSecond,
        chassisSpeeds.vyMetersPerSecond,
        chassisSpeeds.omegaRadiansPerSecond);
  }

  /** Apply a custom swerve request. */
  public void applyRequest(SwerveRequest request) {
    driveIO.setControl(request);
  }

  /** Overload for FieldCentric request (older command files may call this directly). */
  public void applyRequest(SwerveRequest.FieldCentric request) {
    driveIO.setControl(request);
  }

  /**
   * Overload for ApplyRobotSpeeds request (older command files may call this directly). If your
   * CTRE version doesn't have ApplyRobotSpeeds, VSCode will underline this. In that case, replace
   * ApplyRobotSpeeds with the correct request type your CTRE version exposes (autocomplete will
   * show the real name).
   */
  public void applyRequest(SwerveRequest.ApplyRobotSpeeds request) {
    driveIO.setControl(request);
  }

  /** Get the underlying DriveIO instance. */
  public DriveIOHardware getDriveIO() {
    return driveIO;
  }

  private void configureAutoBuilder() {
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose,
          this::resetPose,
          this::getRobotRelativeSpeeds,
          this::runVelocity,
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // translation
              new PIDConstants(5.0, 0.0, 0.0) // rotation
              ),
          config,
          () ->
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
                      == DriverStation.Alliance.Red,
          this);

      System.out.println("AutoBuilder configured successfully");
    } catch (Exception e) {
      System.err.println("AutoBuilder configuration failed");
      e.printStackTrace();
    }
  }
}
