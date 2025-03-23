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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Joystick.ButtonType;
import edu.wpi.first.wpilibj.Joystick.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import frc.robot.Configs.ElevatorConfigures;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TiltSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command
import edu.wpi.first.math.spline.QuinticHermiteSpline;
import edu.wpi.first.math.util.Units;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.NamedCommands;

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
  //TODO add in for OYDS
  private final Elevator lift = new Elevator();
  private final AlgaeSubsystem algae = new AlgaeSubsystem();
  private final CoralSubsystem coral = new CoralSubsystem();
  private final TiltSubsystem tiltCoral = new TiltSubsystem();
  // TODO add feeder back in
  //private final FeederSubsystem feeder = new FeederSubsystem();
  private final Climber climber = new Climber();


  // The driver's controller
  //port zero
  //XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // The secondary controller
  //port one
  //TODO add back in for OYDS
  XboxController secondXboxController = new XboxController(OIConstants.kSecondControllerPort);

  Joystick flightJoystick = new Joystick(2);
  Joystick steeringWheel = new Joystick(3);

  private final SendableChooser<Command> autoChooser;
    
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //Auto Commands
    // To means stop when reached
    // At means runs forever keeping it there

    //runs until within 1 from height
    NamedCommands.registerCommand("Elevator to L4", 
      Commands.run(() -> lift.lineUpL4(), lift)
        .until(() -> lift.checkCorrectHeight()));
    // goes to L4 height and stays there
    NamedCommands.registerCommand("Elevator at L4", 
      Commands.run(() -> lift.lineUpL4(), lift));
    //runs until within 1 from angle
    NamedCommands.registerCommand("Coral tilt to L4", 
    Commands.run(() -> tiltCoral.lineUpL4(), tiltCoral)
    .until(() -> tiltCoral.atDesiredTilt()));

    NamedCommands.registerCommand("Coral tilt to L0", 
    Commands.run(() -> tiltCoral.lineUpL0(), tiltCoral)
      .until(() -> tiltCoral.atDesiredTilt()));

    //expels coral for 2 seconds
    NamedCommands.registerCommand("release coral", 
    Commands.run(() -> coral.coralExpel(), coral)
      .withTimeout(2));

    // expels coral until out of robot
    NamedCommands.registerCommand("drop coral", 
      Commands.run(() -> coral.runOut(), coral)
      .until(() -> !coral.getBeamBreak()));
    // intakes coral until sensor triggered
    NamedCommands.registerCommand("intake coral", 
      Commands.run(() -> coral.runIn(), coral)
      .until(() -> coral.getBeamBreak()));

    // TODO add feeder back in
/*     // runs feeder until sensor triggered
    NamedCommands.registerCommand("feed coral", 
      Commands.run(() -> feeder.runFeeder(), feeder)
      .until(() -> coral.getBeamBreak())); */
    //
    NamedCommands.registerCommand("Elevator to home", 
      Commands.run(() -> lift.setAtHeight(3), lift)
        .until(() -> lift.checkCorrectHeight()));

    NamedCommands.registerCommand("Elevator at home", 
      Commands.run(() -> lift.setAtHeight(3), lift));


    // Configure the button bindings
    configureButtonBindings();
    //TODO add back in for OYDS
    configureSecondaryButtonBindings();

/*     m_robotDrive.setDefaultCommand(
        new RunCommand(
            () -> m_robotDrive.humanDrive(
                -MathUtil.applyDeadband(flightJoystick.getY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(flightJoystick.getX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(steeringWheel.getX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive)); */

    // Configure default commands

    //Teleop drive
    m_robotDrive.setDefaultCommand(
      new RunCommand(
          () -> m_robotDrive.humanDrive(
              -MathUtil.applyDeadband(flightJoystick.getY(), OIConstants.kDriveDeadband),
              -MathUtil.applyDeadband(flightJoystick.getX(), OIConstants.kDriveDeadband),
              Math.abs(-MathUtil.applyDeadband(steeringWheel.getX(), OIConstants.kDriveDeadband)) > Math.abs(-MathUtil.applyDeadband(0.5*flightJoystick.getZ(), OIConstants.kDriveDeadband))?
              -MathUtil.applyDeadband(steeringWheel.getX(), OIConstants.kDriveDeadband) : -MathUtil.applyDeadband(0.5*flightJoystick.getZ(), OIConstants.kDriveDeadband),
              true),
          m_robotDrive)); 

    // Configure default commands
    climber.setDefaultCommand(
        new RunCommand(
            () -> climber.stopClimber(),
            climber));

    //TODO add back in for OYDS
    //Elevator
    lift.setDefaultCommand(
      new RunCommand(
        () -> lift.runElevator(MathUtil.applyDeadband(-secondXboxController.getLeftY(), 0.05)), 
        lift)
    );

    //TODO add back in for OYDS
    // tilt coral
    tiltCoral.setDefaultCommand(
      new RunCommand(
        () -> tiltCoral.tiltEndEffector(MathUtil.applyDeadband(-secondXboxController.getRightY(), 0.05)), 
        tiltCoral)
    );

    //TODO add back in for OYDS
    // coral wheel
    coral.setDefaultCommand(
      new RunCommand(
        () -> coral.runCoral(secondXboxController.getLeftTriggerAxis() > secondXboxController.getRightTriggerAxis()?
        secondXboxController.getLeftTriggerAxis() : -secondXboxController.getRightTriggerAxis()), coral)
    );


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

    //reset odom
    new JoystickButton(flightJoystick, 11)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.resetOdometry(new Pose2d(m_robotDrive.getBlue() ?
        1 : 17, 4, Rotation2d.fromDegrees(m_robotDrive.getBlue() ?
          0 : 180)))));

/* 
    new JoystickButton(flightJoystick, 1)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.setSlowDrive(true)))
      .onFalse(new RunCommand(
        () -> m_robotDrive.setSlowDrive(false))); */
    //slow speed
    new JoystickButton(flightJoystick, 1)
      .onTrue(new InstantCommand(
        () -> m_robotDrive.setSlowDrive(true)))
      .onFalse(new InstantCommand(
        () -> m_robotDrive.setSlowDrive(false)));

    //robot relative drive    
    new JoystickButton(flightJoystick,3)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.humanDrive(
          -MathUtil.applyDeadband(flightJoystick.getY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(flightJoystick.getX(), OIConstants.kDriveDeadband),
          Math.abs(-MathUtil.applyDeadband(steeringWheel.getX(), OIConstants.kDriveDeadband)) > Math.abs(-MathUtil.applyDeadband(flightJoystick.getZ(), OIConstants.kDriveDeadband))?
          -MathUtil.applyDeadband(steeringWheel.getX(), OIConstants.kDriveDeadband) : -MathUtil.applyDeadband(flightJoystick.getZ(), OIConstants.kDriveDeadband), false))); 
    //Robot down
    new JoystickButton(flightJoystick, 4)
      .whileTrue(new RunCommand(
        () -> climber.climberIn(), climber));
    //Line up algae reef
    new JoystickButton(flightJoystick, 5)
      .whileTrue(new RunCommand(
        () -> m_robotDrive.driveToAlgaeReef(), m_robotDrive));
    //Robot up
    new JoystickButton(flightJoystick, 6)
      .whileTrue(new RunCommand(
        () -> climber.climberOut(), climber));


    //Processor
    new POVButton(flightJoystick, 180)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToProcessor(), 
          m_robotDrive));

    //Left Reef
    new POVButton(flightJoystick, 270)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToReef(true), 
          m_robotDrive));

    //Right Reef
    new POVButton(flightJoystick, 90)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.driveToReef(false), 
          m_robotDrive));
          
    //Coral Station
    new POVButton(flightJoystick, 0)
        .whileTrue(new RunCommand(
          () -> m_robotDrive.turnToCoralStation(-MathUtil.applyDeadband(flightJoystick.getY(), OIConstants.kDriveDeadband),
          -MathUtil.applyDeadband(flightJoystick.getX(), OIConstants.kDriveDeadband)), 
          m_robotDrive)); 
  }

  //TODO add back in for OYDS
  private void configureSecondaryButtonBindings() {
    // algae in
    new JoystickButton(secondXboxController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> algae.algaeIntake(), algae))
        .onFalse(new RunCommand(()->algae.algaeStop(), algae));
    // algae out
    new JoystickButton(secondXboxController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> algae.algaeExpel(), algae))
        .onFalse(new RunCommand(() -> algae.algaeStop(), algae));
//TODO add feeder back in
/*     //feeder to front
    new JoystickButton(secondXboxController, Button.kStart.value)
        .whileTrue(new RunCommand(() -> feeder.runFeeder(), feeder))
        .onFalse(new RunCommand(() -> feeder.stopFeeder(), feeder));
    // reverse feeder
    new JoystickButton(secondXboxController, Button.kBack.value)
        .whileTrue(new RunCommand(() -> feeder.reverseFeeder(), feeder))
        .onFalse(new RunCommand(() -> feeder.stopFeeder(), feeder)); */

/*     // set Algae mode
    new JoystickButton(secondXboxController, Button.kStart.value)
        .whileTrue(new RunCommand( () -> lift.setAlgaeMode(true), lift)); */
  
        
/*     //tilt low
    new JoystickButton(secondXboxController, Button.kLeftBumper.value)
        .whileTrue(new RunCommand(() -> tiltCoral.tiltTo(22), tiltCoral));
    // tilt high
    new JoystickButton(secondXboxController, Button.kRightBumper.value)
        .whileTrue(new RunCommand(() -> tiltCoral.tiltTo(95), tiltCoral)); */

    //L4 Elevator
    new POVButton(secondXboxController, 0)
        .whileTrue(new RunCommand(
          () -> lift.lineUpL4(), 
          lift));
    //L3 Elevator
    new POVButton(secondXboxController, 270)
      .whileTrue(new RunCommand(
        () -> lift.lineUpL3(), 
        lift));
    //L1 Elevator
    new POVButton(secondXboxController, 90)
      .whileTrue(new RunCommand(
        () -> lift.lineUpL1(), 
        lift));
    //L2 Elevator
    new POVButton(secondXboxController, 180)
      .whileTrue(new RunCommand(
        () -> lift.lineUpL2(), 
        lift));

    //L4 tilt
    new POVButton(secondXboxController, 0)
        .whileTrue(new RunCommand(
          () -> tiltCoral.lineUpL4(), 
          tiltCoral));
    //L3 tilt
    new POVButton(secondXboxController, 270)
      .whileTrue(new RunCommand(
        () -> tiltCoral.lineUpL3(), 
        tiltCoral));
    //L1 tilt
    new POVButton(secondXboxController, 90)
      .whileTrue(new RunCommand(
        () -> tiltCoral.lineUpL1(), 
        tiltCoral));
    //L2 tilt
    new POVButton(secondXboxController, 180)
      .whileTrue(new RunCommand(
        () -> tiltCoral.lineUpL2(), 
        tiltCoral));


    //algae levels elevator
    //Barge
    new JoystickButton(secondXboxController, Button.kY.value)
      .whileTrue(new RunCommand(() -> lift.bargeLineUp(), lift));
    //Processor
    new JoystickButton(secondXboxController, Button.kB.value)
      .whileTrue(new RunCommand(() -> lift.processorLineUp(), lift));
    //lower algae
    new JoystickButton(secondXboxController, Button.kA.value)
      .whileTrue(new RunCommand(() -> lift.lowerAlgae(), lift));
    // upper algae
    new JoystickButton(secondXboxController, Button.kX.value)
      .whileTrue(new RunCommand(() -> lift.upperAlgae(), lift));

    //algae tilt
    //Barge
    new JoystickButton(secondXboxController, Button.kY.value)
      .whileTrue(new RunCommand(() -> tiltCoral.lineUpBarge(), tiltCoral));
    //Processor
    new JoystickButton(secondXboxController, Button.kB.value)
      .whileTrue(new RunCommand(() -> tiltCoral.lineUpProcessor(), tiltCoral));
    //lower algae
    new JoystickButton(secondXboxController, Button.kA.value)
      .whileTrue(new RunCommand(() -> tiltCoral.lineUpLowerAlgae(), tiltCoral));
    // upper algae
    new JoystickButton(secondXboxController, Button.kX.value)
      .whileTrue(new RunCommand(() -> tiltCoral.lineUpUpperAlgae(), tiltCoral));
  }


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

}
