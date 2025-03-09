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
import frc.robot.Configs.ElevatorConfigures;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FeederSubsystem;
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


/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  //TODO add in for Water Tight
  /* private final Elevator lift = new Elevator();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final FeederSubsystem feeder = new FeederSubsystem(); */


  // The driver's controller
  //port zero
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The secondary controller
  //port one
  //TODO add back in for Water Tight
  //XboxController secondXboxController = new XboxController(OIConstants.kSecondControllerPort);

  private final SendableChooser<Command> autoChooser;
    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    //TODO add back in for Water Tight
    //configureSecondaryButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.humanDrive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
        //TODO add back in for Water Tight
        /* lift.setDefaultCommand(
          new RunCommand(
            () -> lift.runElevator(-secondXboxController.getLeftY()), 
            lift)
        );*/

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
    // put wheels in X
    //X Button
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));

    // resets Odom to 1, 1, 0
    //Right Bumper
    new JoystickButton(m_driverController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.resetOdometry(new Pose2d(1, 1, Rotation2d.fromDegrees(0))), m_robotDrive));
          
    /* // Line up 12in before tag 18
    new JoystickButton(m_driverController, Button.kLeftBumper.value)
        //.whileTrue(goTo(new Pose2d(3.66-(RobotConstants.robotWidthWithBumpers/2) + inToMeter(12), 4.03, Rotation2d.fromDegrees(0))));
        .whileTrue(new RunCommand(goTo(null, null), null)); */

/*     // Line up 12in before tag 18
    //B Button
    new JoystickButton(m_driverController, Button.kB.value)
        //.whileTrue(goTo(new Pose2d(3.66-(RobotConstants.robotWidthWithBumpers/2) + inToMeter(12), 4.03, Rotation2d.fromDegrees(0))));
        .whileTrue(new RunCommand( 
          () -> m_robotDrive.driveToPosition(new Pose2d(3.3552, 4.03, Rotation2d.fromDegrees(0))), m_robotDrive)); */

    /* //Goes to zero-zero-zero
    //Button A
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToPosition(new Pose2d(0, 0, Rotation2d.fromDegrees(0))), 
          m_robotDrive)); */

///////////////////////////////////////////////////////
/// 
    //Processor
    new JoystickButton(m_driverController, Button.kA.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToProcessor(), 
          m_robotDrive));

    //Left Reef
    new JoystickButton(m_driverController, Button.kX.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToReef(true), 
          m_robotDrive));

    //Right Reef
    new JoystickButton(m_driverController, Button.kB.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToReef(false), 
          m_robotDrive));
    //Coral Station
    new JoystickButton(m_driverController, Button.kY.value)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.turnToCoralStation(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband)), 
          m_robotDrive));
    //North
    new POVButton(m_driverController, 0)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.humanTurnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
          0, 
          true), 
          m_robotDrive));
    //West
    new POVButton(m_driverController, 270)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.humanTurnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
          90, 
          true), 
          m_robotDrive));
    //East
    new POVButton(m_driverController, 90)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.humanTurnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
          -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
          270, 
          true), 
          m_robotDrive));
    //South
    new POVButton(m_driverController, 180)
          .whileTrue(new RunCommand(
            () -> m_robotDrive.humanTurnDrive(-MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband), 
            -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband), 
            180, 
            true), 
            m_robotDrive));

  }

  //TODO add back in for Water Tight
  /* private void configureSecondaryButtonBindings() {

    new JoystickButton(secondXboxController, Button.kA.value)
        .whileTrue(new RunCommand(() -> algae.algaeIntake(), algae))
        .onFalse(new RunCommand(()->algae.algaeStop(), algae));

    new JoystickButton(secondXboxController, Button.kX.value)
        .whileTrue(new RunCommand(() -> algae.algaeExpel(), algae))
        .onFalse(new RunCommand(() -> algae.algaeStop(), algae));

        new JoystickButton(secondXboxController, Button.kB.value)
            .whileTrue(new RunCommand(() -> coral.coralIntake(), coral))
            .onFalse(new RunCommand(()->coral.coralStop(), coral));
    
        new JoystickButton(secondXboxController, Button.kY.value)
            .whileTrue(new RunCommand(() -> coral.coralExpel(), coral))
            .onFalse(new RunCommand(() -> coral.coralStop(), coral));

    new JoystickButton(secondXboxController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> feeder.runFeeder(), feeder))
        .onFalse(new RunCommand(() -> feeder.stopFeeder(), feeder));

    new JoystickButton(secondXboxController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> feeder.reverseFeeder(), feeder))
        .onFalse(new RunCommand(() -> feeder.stopFeeder(), feeder));

  }
 */


  //Method for displaying abs encoder values for finding offset
  public void displayAbsoluteAngle(){
    m_robotDrive.displayAbsValues();
  }

  

  /* public Command goToReef(){

  } */
  

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

  public Command getSecondAutoCommand(){
    //-TO-DO- load this when code starts not when this called here //I think this should be fine because its like the coconuts last year
    //return new PathPlannerAuto("Test Auto");
    return autoChooser.getSelected();
  }

  public double inToMeter(double measurement){
    return measurement * 0.0254;
  }

  public void setUp(){
    m_robotDrive.checkAllianceColor();
    m_robotDrive.setEncoder();
  }

  double applySensitivity(double orignalValue, double sensitivity){
    //absolute value of value raised to 1/sensitivity, then sign reaplied
    double newValue = Math.abs(orignalValue);
    newValue = Math.pow(newValue, sensitivity/1);
    newValue = Math.copySign(newValue, orignalValue);
    //System.out.println(orignalValue + "orignal value");
    //System.out.println(newValue + "newValue");
    return newValue;
  }
}
