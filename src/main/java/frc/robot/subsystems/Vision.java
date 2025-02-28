// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import java.lang.annotation.Target;
import java.lang.invoke.VolatileCallSite;
import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.jni.AHRSJNI;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import java.util.List;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.Vision.*;

import java.util.function.Consumer;

public class Vision extends SubsystemBase {
  
  //
  //PhotonCamera camera = new PhotonCamera("rpi-cam");
  PhotonCamera camera;
  PhotonPoseEstimator photonPoseEstimator;

  private Matrix<N3, N1> curStdDevs = VecBuilder.fill(0, 0, 0);


  public double targetYaw = 0;
  public double targetRange = 0;
  public double targetArea = 0;
  public boolean targetVisible = false;

  //TODO Camera position on robot
   

  /** Creates a new DriveSubsystem. */
  public Vision() {
    //TODO add Vision Constants
    camera  = new PhotonCamera(kCameraName);
    // Construct PhotonPoseEstimator
    photonPoseEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
    photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

  }

  @Override
  public void periodic() {
    // Read in relevant data from the Camera
    var results = camera.getAllUnreadResults();
    //Optional<EstimatedRobotPose> 3dResults = getEstimatedGlobalPose();

    /* if (!3dresults.isEmpty()) {
      var 3dresults =  */
    //SmartDashboard.putString("Vision estimated glolbal pose", getEstimatedGlobalPose());
    //getEstimatedGlobalPose()

    if (!results.isEmpty()) {
      SmartDashboard.putString("Vision estimated glolbal pose", getEstimatedGlobalPose().toString());
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 8) {
                    // Found Tag 8, record its information
                    targetVisible = true;
                    targetYaw = target.getYaw();
                    targetArea = target.getArea();
                    SmartDashboard.putNumber("PC yaw", targetYaw);
                    SmartDashboard.putNumber("PC area", targetArea);
                    SmartDashboard.putString("PC best to target", target.getBestCameraToTarget().toString());
                    SmartDashboard.putString("PC alt to target", target.getAlternateCameraToTarget().toString());
                    SmartDashboard.putNumber("PC pitch",target.getPitch());
                    SmartDashboard.putNumber("PC skew", target.getSkew());

                    targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                      0.5, // Height off floor, measured with a tape measure, or in CAD.
                      1.435, // From 2024 game manual for ID 7
                      Math.toRadians(-30.0),// Measured with a protractor, or in CAD.
                      Math.toRadians(target.getPitch()));

                    SmartDashboard.putNumber("PC range", targetRange);
                    
                } else {
                  targetYaw = 0;
                  targetRange = 0;
                  targetArea = 18;
                }
                SmartDashboard.putBoolean("results.isEmpty", results.isEmpty());
                //SmartDashboard.putArray("getTargets", result.getTargets);
                SmartDashboard.putNumber("target's Yaw", targetYaw);
            }
        } else {
          targetYaw = 0;
          targetRange = 0;
          targetArea = 18;
        }
    }
    
  }

 /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : camera.getAllUnreadResults()) {
            visionEst = photonPoseEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        return visionEst;
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonPoseEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
        SmartDashboard.putNumber("curStdDevs 1", curStdDevs.get(1, 0));
        SmartDashboard.putNumber("curStdDevs 2", curStdDevs.get(2, 0));
        SmartDashboard.putNumber("curStdDevs 0", curStdDevs.get(0, 0));
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
  public double targetYaw(){    
    SmartDashboard.putNumber("V TargetYaw", targetYaw);
    return targetYaw;
    //-1.0 * targetYaw * VisionConstants.turningP * DriveConstants.kMaxAngularSpeed   

  }

  public double targetRange(){
    return targetRange;
  }

  public double targetArea(){
    return targetArea;
  }



  
}
