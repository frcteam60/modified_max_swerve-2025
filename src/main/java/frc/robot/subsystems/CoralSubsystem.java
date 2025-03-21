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

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

  private final SparkMax coralWheel;
  private final RelativeEncoder coralWheelEncoder;
  private final DigitalInput beamBreak;

  private final double runSpeed = 0.8;
 
  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    beamBreak = new DigitalInput(5);
        
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

  public boolean getBeamBreak(){
    return beamBreak.get();
  }

  public void runIn(){
    if(getBeamBreak()){
      coralWheel.set(runSpeed);
    } else {
      coralWheel.stopMotor();
    }
  }

  public void runOut(){
    if(!getBeamBreak()){
      coralWheel.set(-runSpeed);
    } else {
      coralWheel.stopMotor();
    }
  }


  public void showEncoders(){
    SmartDashboard.putNumber("Coral Wheel", coralWheelEncoder.getVelocity());
  }
}
