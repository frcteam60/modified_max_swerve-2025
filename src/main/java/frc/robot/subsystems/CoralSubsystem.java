// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

  private final SparkMax coralWheel;
  private final RelativeEncoder coralWheelEncoder;

  private final double runSpeed = 0.8;

  //double lowerLimit = 7;
  double lowerLimit = 10;
  //35
  double upperLimit = 95;
  //95
  //19.5945

  //TODO get these angles
  double L4Angle = 90;
  double L3Angle = 95;
  double L2Angle = 95;
  double L1Angle = 90;
  double L0Angle = 5;
 
  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    
    coralWheel = new SparkMax(CoralConstants.coralCANID, MotorType.kBrushless);
    coralWheelEncoder = coralWheel.getEncoder();


    coralWheel.configure(Configs.CoralConfig.coralWheelConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showEncoders();
  }

  public void coralIntake() {
    coralWheel.set(runSpeed);
  }

  public void coralExpel() {
    coralWheel.set(-1*runSpeed);
  }

  public void coralStop() {
    coralWheel.stopMotor();
  }

  public void runCoral(double speed){
    coralWheel.set(speed);
  }


  public void showEncoders(){
    SmartDashboard.putNumber("Coral Wheel", coralWheelEncoder.getVelocity());
  }
}
