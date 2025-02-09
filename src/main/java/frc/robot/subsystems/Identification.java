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
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

public class Identification extends SubsystemBase {

  // SysId stuff
 /*  private final Config sysIdConfig = new Config();
  private final Consumer<Voltage> c_drive;
  private final Consumer<SysIdRoutineLog> c_log; */
  
  //TODO fix this stuff
  //private final Mechanism sysIdMechanism = new Mechanism(this::voltageDrive, this::logMotors, this);
  /* private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
    sysIdConfig, 
    new SysIdRoutine.Mechanism(this::voltageDrive, this::logMotors, this)
  ); */

  /** Creates a new DriveSubsystem. */
  public Identification() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    

  }

  @Override
  public void periodic() {
    
  }
  /* public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  }
  //Set turning encoders based on abs
  public void setEncoder(){
    
  }


  public ChassisSpeeds getRobotRelativeSpeeds(){
 
  } */

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  //Warning this equals resetPose basically for PathPlanner
  public void resetOdometry(Pose2d pose) {
    
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    
  }

  public void driveRobotRelative(ChassisSpeeds botRelChassisSpeeds){
    //TODO
    // 

    //System.out.println(xSpeed);
    //System.out.println(ySpeed);
    //System.out.println(rot);
    
    
  }
  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.reset();
   
    //TODO does this reset work?
    // try zero yaw?
  }


}
