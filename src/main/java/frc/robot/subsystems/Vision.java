// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;

import java.lang.invoke.VolatileCallSite;
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

import org.photonvision.PhotonCamera;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

public class Vision extends SubsystemBase {
  
  //
  PhotonCamera camera = new PhotonCamera("rpi-cam");
  int timesThrough = 0;

  /** Creates a new DriveSubsystem. */
  public Vision() {
    
    

  }

  @Override
  public void periodic() {
    
  }

  public double targetYaw(){
    // Read in relevant data from the Camera
    boolean targetVisible = false;
    double targetYaw = 0.0;
    var results = camera.getAllUnreadResults();
    timesThrough++;

    if (!results.isEmpty()) {
        // Camera processed a new frame since last
        // Get the last one in the list.
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) {
            // At least one AprilTag was seen by the camera
            for (var target : result.getTargets()) {
                if (target.getFiducialId() == 8) {
                    // Found Tag 8, record its information
                    targetYaw = target.getYaw();
                    targetVisible = true;
                }
            }
        }
    }
    SmartDashboard.putBoolean("results.isEmpty", results.isEmpty());
    //SmartDashboard.putArray("getTargets", result.getTargets);
    SmartDashboard.putNumber("timesThrough", timesThrough);
    SmartDashboard.putNumber("target's Yaw", targetYaw);
    return targetYaw;
    //-1.0 * targetYaw * VisionConstants.turningP * DriveConstants.kMaxAngularSpeed   

  }

  //public void return



  
}
