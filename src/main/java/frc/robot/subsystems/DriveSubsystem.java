// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

import java.lang.invoke.VolatileCallSite;
import java.security.PublicKey;
import java.util.List;
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

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftSteeringCanId,
      DriveConstants.kFrontLeftEncoderNum,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightSteeringCanId,
      DriveConstants.kFrontRightEncoderNum,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftSteeringCanId,
      DriveConstants.kRearLeftEncoderNum,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightSteeringCanId,
      DriveConstants.kRearRightEncoderNum,
      DriveConstants.kBackRightChassisAngularOffset);

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();

  Timer time;
  AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  double yaw = 0;
  double yawOffset = 0;
  Pose2d testPose2d = new Pose2d(0,0, Rotation2d.fromDegrees(0));

  boolean visionEstIsPresent = false;

  boolean blue = true;
  boolean allianceYet = false;
  
  private final Vision piCam = new Vision();

  // The robot pose estimator for tracking swerve odometry and applying vision corrections.
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
    setEncoder();

    time = new Timer();
    // Standard deviations for odometry: X, Y, rotation
    var stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
    //Starting standard deviations for vision: X, Y, rotation
    var visionStdDevs = VecBuilder.fill(1, 1, 1);

    poseEstimator =
                new SwerveDrivePoseEstimator(
                        DriveConstants.kDriveKinematics,
                        Rotation2d.fromDegrees(yaw),
                        new SwerveModulePosition[] {
                          m_frontLeft.getPosition(),
                          m_frontRight.getPosition(),
                          m_rearLeft.getPosition(),
                          m_rearRight.getPosition()},
                        new Pose2d(),
                        stateStdDevs,
                        visionStdDevs);

    //AutoBuider Config for PathPlanner
    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(2.5, 0.0, 0.0) // Rotation PID constants
        ),
        config, // The robot configuration
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          //TODO field side probally needs to be in auto init instead not when we turn on?
          //TODO should this even flip on red?
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
      //config = RobotConfig.fromGUISettings();
      
    }
    
    Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          blue = false;
          allianceYet = true;
        }
        if (ally.get() == Alliance.Blue) {
          blue = true;
          allianceYet = true;
        }
      } else {
        blue = true;
        allianceYet = false;
      }

  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    yaw = gyro.getYaw()*-1;
    //Displays module angle in radians
    SmartDashboard.putNumber("FL Encoder Relative", m_frontLeft.returnModuleAngle());
    SmartDashboard.putNumber("BL Encoder Relative", m_rearLeft.returnModuleAngle());
    SmartDashboard.putNumber("FR Encoder Relative", m_frontRight.returnModuleAngle());
    SmartDashboard.putNumber("BR Encoder Relative", m_rearRight.returnModuleAngle());
    
    SmartDashboard.putString("robotOnField Odom", poseEstimator.getEstimatedPosition().toString());
   
    poseEstimator.update(
        //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        Rotation2d.fromDegrees(yaw),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    
      
    //poseEstimator.addVisionMeasurement(visionMeasurement, time.getFPGATimestamp());
    //addVisionMeasurement(piCam.getEstimatedGlobalPose(), time.getFPGATimestamp());

    // Correct pose estimate with vision measurements
    var visionEst = piCam.getEstimatedGlobalPose();
    visionEst.ifPresent(
            // est seems to be standing in for EstimatedRobotPose gotten from getEstimatedGlobalPose()
            est -> {
              System.out.println("Vision Est is present");
                visionEstIsPresent = true;
                testPose2d = est.estimatedPose.toPose2d();
                SmartDashboard.putString("V Est pose", testPose2d.toString());
                // Change our trust in the measurement based on the tags we can see
                var estStdDevs = piCam.getEstimationStdDevs();
                addVisionMeasurement(
                  est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs
                );
                /* poseEstimator.addVisionMeasurement(
                        est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs); */
            }
            );
    
    if(visionEst.isEmpty()){
      visionEstIsPresent = false;
    }

    SmartDashboard.putNumber("PoseEstimator_X", poseEstimator.getEstimatedPosition().getX());
    SmartDashboard.putNumber("PoseEstimator_Y", poseEstimator.getEstimatedPosition().getY());
    
  }

  public void checkAllianceColor(){
    Optional<Alliance> ally = DriverStation.getAlliance();
      if (ally.isPresent()) {
        if (ally.get() == Alliance.Red) {
          blue = false;
          allianceYet = true;
        }
        if (ally.get() == Alliance.Blue) {
          blue = true;
          allianceYet = true;
        }
      } else {
        blue = true;
        allianceYet = false;
      }
  }

  public boolean isVisionEstPresent(){
    return visionEstIsPresent;
  }

  public Pose2d returnVisionEst(){
    return testPose2d;
  }

 
  public void logMotors(){
    
  }

  public void voltageDrive(Voltage volts){
    m_frontLeft.voltageControl(volts);
    m_frontRight.voltageControl(volts);
    m_rearLeft.voltageControl(volts);
    m_rearRight.voltageControl(volts);
  }

  //Set turning encoders based on abs
  public void setEncoder(){
    m_frontLeft.setEncoder(DriveConstants.kFrontLeftAbsOffset);
    m_rearLeft.setEncoder(DriveConstants.kRearLeftAbsOffset);
    m_frontRight.setEncoder(DriveConstants.kFrontRightAbsOffset);
    m_rearRight.setEncoder(DriveConstants.kRearRightAbsOffset);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds(){
    //TODO finish this I dont know how its relative
    //Must return robot relative not field relative
    return DriveConstants.kDriveKinematics.toChassisSpeeds(
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
  );
  }

  

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  //Warning this equals resetPose basically for PathPlanner
  public void resetOdometry(Pose2d pose) {
    poseEstimator.resetPosition(
        //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)),
        Rotation2d.fromDegrees(yaw),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  public void humanDrive(double xSpeed, double ySpeed, double rot, boolean fieldRelative){
    if(blue){
      drive(xSpeed, ySpeed, rot, fieldRelative);
    } else {
      drive(-xSpeed, -ySpeed, rot, fieldRelative);
    }
  
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
    System.out.println(xSpeed);
    System.out.println(ySpeed);
    System.out.println(rot);
    System.out.println(fieldRelative);
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                Rotation2d.fromDegrees(yaw))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    //System.out.println("zero"+ swerveModuleStates[0].toString());
    //System.out.println("one"+ swerveModuleStates[1].toString());
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void driveToPosition(Pose2d positionWanted){
    double desiredXSpeed = positionWanted.getX() - getPose().getX();
    double desiredYSpeed = positionWanted.getY() - getPose().getY();
    double desiredRotSpeed = positionWanted.getRotation().getDegrees() - getPose().getRotation().getDegrees();

    desiredXSpeed = desiredXSpeed * 5;
    desiredYSpeed = desiredYSpeed * 5;
    desiredRotSpeed = desiredRotSpeed * 2.5;

     drive(desiredXSpeed, desiredYSpeed, desiredRotSpeed, true);
  }

  public void driveRobotRelative(ChassisSpeeds botRelChassisSpeeds){
    //TODO
    // 

    //System.out.println(xSpeed);
    //System.out.println(ySpeed);
    //System.out.println(rot);
    
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = botRelChassisSpeeds.vxMetersPerSecond * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = botRelChassisSpeeds.vyMetersPerSecond * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = botRelChassisSpeeds.omegaRadiansPerSecond * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    //System.out.println("zero"+ swerveModuleStates[0].toString());
    //System.out.println("one"+ swerveModuleStates[1].toString());
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  // Subtracts two angles
  public double angleSubtractor (double firstAngle, double secondAngle) {
    // 
     double result = ((firstAngle - secondAngle) + 360180)%360 - 180;
     return result;

  }
  public void humanTurnDrive(double xSpeed, double ySpeed, double desiredAngle, boolean fieldRelative){
    if(blue){
      turnDrive(xSpeed, ySpeed, desiredAngle, fieldRelative);
    } else {
      turnDrive(-xSpeed, -ySpeed, angleSubtractor(desiredAngle, 180), fieldRelative);
    }
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param desiredAngleDeg           Angle of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void turnDrive(double xSpeed, double ySpeed, double desiredAngleDeg, boolean fieldRelative) {
  /*   System.out.println(xSpeed);
    System.out.println(ySpeed);
    System.out.println(desiredAngle);
    System.out.println(fieldRelative); */
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = (angleSubtractor(desiredAngleDeg, yaw))* 0.01 * DriveConstants.kMaxAngularSpeed;

    /* public void optimize(Rotation2d currentAngle) {
      var delta = angle.minus(currentAngle);
      if (Math.abs(delta.getDegrees()) > 90.0) {
        speedMetersPerSecond *= -1;
        angle = angle.rotateBy(Rotation2d.kPi);
      }
    } */

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                Rotation2d.fromDegrees(yaw))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    /* System.out.println("zero"+ swerveModuleStates[0].toString());
    System.out.println("one"+ swerveModuleStates[1].toString()); */
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }


  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void lineUpDrive(double targetArea, double ySpeed, double rot, boolean fieldRelative, double desiredArea){
    /* System.out.println(targetArea);
    System.out.println(ySpeed);
    System.out.println(rot);
    System.out.println(fieldRelative); */

//-MathUtil.applyDeadband(piCam.targetRange, OIConstants.kDriveDeadband)
//                -MathUtil.applyDeadband(piCam.targetYaw(), OIConstants.kDriveDeadband),

    // Convert the commanded speeds into the correct units for the drivetrain
    double rotDelivered = rot * VisionConstants.turningP * DriveConstants.kMaxAngularSpeed;
    SmartDashboard.putNumber("vision rotDelivered", rotDelivered);
    //double xSpeedDelivered = (desiredDistance - targetRange) * VisionConstants.strafeP * DriveConstants.kMaxSpeedMetersPerSecond;
    double xSpeedDelivered = (desiredArea-targetArea) * -0.1;
    //System.out.println("xspeedDelivered" + xSpeedDelivered);
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;

    rotDelivered = 0.25 * rotDelivered;
    xSpeedDelivered = 0.5 * xSpeedDelivered;
    ySpeedDelivered = 0.5 * ySpeedDelivered;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                //Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)))
                Rotation2d.fromDegrees(yaw))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    /* System.out.println("zero"+ swerveModuleStates[0].toString());
    System.out.println("one"+ swerveModuleStates[1].toString()); */
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    //m_gyro.reset();
    gyro.reset();
    //TODO does this reset work?
    // try zero yaw?
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    //return Rotation2d.fromDegrees(m_gyro.getAngle(IMUAxis.kZ)).getDegrees();
    return Rotation2d.fromDegrees(yaw).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    //return m_gyro.getRate(IMUAxis.kZ) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    return yaw * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double)}. */
  public void addVisionMeasurement(Pose2d visionMeasurement, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds);
  }

  /** See {@link SwerveDrivePoseEstimator#addVisionMeasurement(Pose2d, double, Matrix)}. */
  public void addVisionMeasurement(
          Pose2d visionMeasurement, double timestampSeconds, Matrix<N3, N1> stdDevs) {
      poseEstimator.addVisionMeasurement(visionMeasurement, timestampSeconds, stdDevs);
  }


  public void displayAbsValues(){
    SmartDashboard.putNumber("FL Encoder abs", m_frontLeft.returnAbsAngle());
    SmartDashboard.putNumber("BL Encoder abs", m_rearLeft.returnAbsAngle());
    SmartDashboard.putNumber("FR Encoder abs", m_frontRight.returnAbsAngle());
    SmartDashboard.putNumber("BR Encoder abs", m_rearRight.returnAbsAngle());
  }

/*   //look into Command file for this
  public void lineUpWith18(){
    //TODO may need to tune auto loops

    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // A trajectory to follow. All units in meters.
    Trajectory to18Trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        getPose(),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(getPose().getX(), getPose().getY())),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);


    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        to18Trajectory,
        getPose(), // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
       etModuleStates,
        this);

    SwerveControllerCommand mySwerveControllerCommand = new SwerveControllerCommand(
        testTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,
    
        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    resetOdometry(to18Trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return mySwerveControllerCommand.andThen(() -> drive(0, 0, 0, false));
 
  } */
  public double inToMeter(double measurement){
    return measurement * 0.0254;
  }


}
