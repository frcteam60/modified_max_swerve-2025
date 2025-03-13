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
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

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
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.jni.AHRSJNI;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

public class Elevator extends SubsystemBase {
  

  Timer time;
  private final SparkMax elevatorOneMax;
  private final SparkMax elevatorTwoMax;
  private final RelativeEncoder elevatorOneEncoder;
  private final RelativeEncoder elevatorTwoEncoder;

  private final SparkClosedLoopController elevatorOneClosedLoopController;
  private final SparkClosedLoopController elevatorTwoClosedLoopController;

  
  double L4Height = 27.36;
  double L3Height = 17.50;
  double L2Height = 10.47;
  double L1Height = 6.08;
  double L0Height = 0;


  //TODO need new limits
  double lowerLimit = 1;
  double upperLimit = 28.5;
  // 28.886920
  //double upperLimit = 19.90485;

  double runSpeed = 1;
  double downMultiplier = 0.05;
  //9.904806
  //three something at board height
  /*
    0-21.190568 largest range
    0-19.90485 safe range

   9.404799:
    upper limit
    -11.785769:
    lowerlimit

    -10.500051    biggest lift approx. in above wood

    -6.738101 encoder rotations
    bottom of elevator 12 1/4 in above wood
    bottom of smaller section 5 5/8 in above wood
   */

  

  /** Creates a new DriveSubsystem. */
  public Elevator() {
    elevatorOneMax = new SparkMax(ElevatorConstants.elevatorOneCANID, MotorType.kBrushless);
    elevatorTwoMax = new SparkMax(ElevatorConstants.elevatorTwoCANID, MotorType.kBrushless);

    elevatorOneEncoder = elevatorOneMax.getEncoder();
    elevatorTwoEncoder = elevatorTwoMax.getEncoder();

    elevatorOneClosedLoopController = elevatorOneMax.getClosedLoopController();
    elevatorTwoClosedLoopController = elevatorTwoMax.getClosedLoopController();

    elevatorOneMax.configure(Configs.ElevatorConfigures.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorTwoMax.configure(Configs.ElevatorConfigures.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //elevatorOneMax.configure(null, null, null);


    /* m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); */


    time = new Timer();

  }

  @Override
  public void periodic() {
    showEncoders();
    
    
  }

 
  public void logMotors(){
    
  }

  public void showEncoders(){
    SmartDashboard.putNumber("elevatorOne", elevatorOneEncoder.getPosition());
    SmartDashboard.putNumber("elevatorTwo", elevatorTwoEncoder.getPosition());
  }


  public double inToMeter(double measurement){
    return measurement * 0.0254;
  }

  public void runElevator(double speed){
    if(elevatorOneEncoder.getPosition() <= lowerLimit && (Math.signum(speed) == -1)){ 
      elevatorOneMax.stopMotor();
      elevatorTwoMax.stopMotor();
    } else if (elevatorOneEncoder.getPosition() >= upperLimit && (Math.signum(speed) == 1)){
      elevatorOneMax.stopMotor();
      elevatorTwoMax.stopMotor();
    } else if(speed < 0){
      elevatorOneMax.set(runSpeed*speed*downMultiplier);
      elevatorTwoMax.set(runSpeed*speed*downMultiplier);
    }else{
      elevatorOneMax.set(runSpeed*speed);
      elevatorTwoMax.set(runSpeed*speed);
    }
  //  System.out.println("elevatorspeed" + speed);

  }

  public void setAtHeight(double desiredHeight){
    elevatorOneClosedLoopController.setReference(desiredHeight, ControlType.kPosition);
    elevatorTwoClosedLoopController.setReference(desiredHeight, ControlType.kPosition);
         /* if(tiltEncoder.getPosition()>desiredPosition){
      tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
     } else {
      tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
     } */
  }

  
  public void lineUpL4(){
    setAtHeight(L4Height);
  }
  public void lineUpL3(){
    setAtHeight(L3Height);
  }
  public void lineUpL2(){
    setAtHeight(L2Height);
  }
  public void lineUpL1(){
    setAtHeight(L1Height);
  }
  public void lineUpL0(){
    setAtHeight(L0Height);
  }

}
