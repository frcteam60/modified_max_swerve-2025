// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
//import edu.wpi.first.wpilibj2.command
import edu.wpi.first.math.spline.QuinticHermiteSpline;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.button.POVButton;

//TODO figure out where I need vision, drive or container

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  //TODO add back in
  private final Vision piCam = new Vision();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;
    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    piCam.getEstimatedGlobalPose();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));

    //TODO register commands also I think ? only if I use other commands?
    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.resetOdometry(new Pose2d(10, 10, Rotation2d.fromDegrees(0))), m_robotDrive));
          
    // Lines
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        .whileTrue(goTo(new Pose2d(2,2, Rotation2d.fromDegrees(0))));

    new POVButton(m_driverController, 0)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.turnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
          0, 
          true), 
          m_robotDrive));

    new POVButton(m_driverController, 270)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.turnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
          90, 
          true), 
          m_robotDrive));

    new POVButton(m_driverController, 90)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.turnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
          270, 
          true), 
          m_robotDrive));

    new POVButton(m_driverController, 180)
          .whileTrue(new RunCommand(
            () -> m_robotDrive.turnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
            180, 
            true), 
            m_robotDrive));

    //TODO add back in
    /* new JoystickButton(m_driverController, Button.kL1.value)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.lineUpDrive(
          piCam.targetArea,
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
          piCam.targetYaw(),
          false,
          9//18
        ), 
        m_robotDrive));  */
  }
  //Method for displaying abs encoder values for finding offset
  public void displayAbsoluteAngle(){
    m_robotDrive.displayAbsValues();
  }

  public Command lineUp18(){
    double startPoseX;
    double startPoseY;
    //TODO only when vision is reliable
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);
    
    /* // A trajectory to follow. All units in meters.
    Trajectory to18Trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        /* m_robotDrive.isVisionEstPresent() 
          ? m_robotDrive.returnVisionEst()
          : m_robotDrive.getPose() //comment out ends here //TODO
        m_robotDrive.getPose(),
        // Pass through these two interior waypoints, making an 's' curve path
        //List.of(new Translation2d(m_robotDrive.getPose().getX(), m_robotDrive.getPose().getY())),
        List.of(m_robotDrive.getPose().getTranslation()),

        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(2.66, 4.03, new Rotation2d(0)),
        config); */
    
    // 2018 cross scale auto waypoints.
    var sideStart = new Pose2d(Units.feetToMeters(1.54), Units.feetToMeters(23.23),
        Rotation2d.fromDegrees(-180));
    var crossScale = new Pose2d(Units.feetToMeters(23.7), Units.feetToMeters(6.8),
        Rotation2d.fromDegrees(-160));

    System.out.println(m_robotDrive.isVisionEstPresent() 
        ? m_robotDrive.returnVisionEst().getTranslation()
        : m_robotDrive.getPose().getTranslation() + "see it prints");
    
    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(m_robotDrive.isVisionEstPresent() 
      ? m_robotDrive.returnVisionEst().getTranslation()
      : m_robotDrive.getPose().getTranslation());
    
    var startPose = m_robotDrive.isVisionEstPresent() 
    ? m_robotDrive.returnVisionEst()
    : m_robotDrive.getPose();    
    
  
    //If x or y are 0 or 0.001 change to 0.01
    if(Math.abs(startPose.getX()) < 0.01){
      startPoseX = 0.01; 
    } else {
      startPoseX = startPose.getX();
    };

    if(Math.abs(startPose.getY()) < 0.01){
      startPoseY = 0.01; 
    } else {
      startPoseY = startPose.getY();
    };

    startPose = new Pose2d(startPoseX, startPoseY, startPose.getRotation());
    
    System.out.println("startPose " + startPose.toString());

    var trajectory = TrajectoryGenerator.generateTrajectory(
      startPose,
      interiorWaypoints,
      crossScale,
      config);
    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

  }
  public Command goTo(Pose2d desiredPose){
    double startPoseX;
    double startPoseY;
    Pose2d endPose = new Pose2d(2.01, 2, Rotation2d.fromDegrees(1));
    //TODO only when vision is reliable
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    
    
    var startPose = m_robotDrive.getPose();    
    
    //If x or y are 0 or 0.001 change to 0.01
    if(Math.abs(startPose.getX()) < 0.01){
      startPoseX = 0.01; 
    } else {
      startPoseX = startPose.getX();
    };

    if(Math.abs(startPose.getY()) < 0.01){
      startPoseY = 0.01; 
    } else {
      startPoseY = startPose.getY();
    };

    startPose = new Pose2d(startPoseX, startPoseY, startPose.getRotation());

    //TODO take following line out
    startPose = new Pose2d(0.01, 0.01, Rotation2d.fromDegrees(1));

    var interiorWaypoints = new ArrayList<Translation2d>();
    interiorWaypoints.add(new Translation2d(1, 1));
    
    System.out.println("startPose " + startPose.toString());

    var trajectory = TrajectoryGenerator.generateTrajectory(
      startPose,
      interiorWaypoints,
      endPose,
      config);
    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        trajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

    // Reset odometry to the starting pose of the trajectory.
    m_robotDrive.resetOdometry(trajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // Create config for trajectory
    TrajectoryConfig config = new TrajectoryConfig(
        AutoConstants.kMaxSpeedMetersPerSecond,
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(DriveConstants.kDriveKinematics);

    // An example trajectory to follow. All units in meters.
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
        // Pass through these two interior waypoints, making an 's' curve path
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3, 0, new Rotation2d(0)),
        config);


    // Test Trajectory
    Trajectory testTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
        List.of(new Translation2d(0.5, 0.5)), 
        new Pose2d(new Translation2d(1, 1), new Rotation2d(0)), 
        config);
    
    var thetaController = new ProfiledPIDController(
        AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
        exampleTrajectory,
        m_robotDrive::getPose, // Functional interface to feed supplier
        DriveConstants.kDriveKinematics,

        // Position controllers
        new PIDController(AutoConstants.kPXController, 0, 0),
        new PIDController(AutoConstants.kPYController, 0, 0),
        thetaController,
        m_robotDrive::setModuleStates,
        m_robotDrive);

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
    m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    //return swerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
    return mySwerveControllerCommand.andThen(() -> m_robotDrive.drive(0, 0, 0, false));
  }

  public Command  getSecondAutoCommand(){
    //-TO-DO- load this when code starts not when this called here //I think this should be fine because its like the coconuts last year
    //return new PathPlannerAuto("Test Auto");
    return autoChooser.getSelected();
  }
}
