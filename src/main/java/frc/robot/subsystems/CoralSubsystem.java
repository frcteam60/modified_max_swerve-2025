// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
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
  private final SparkMax tiltMax;
  private final RelativeEncoder coralWheelEncoder;
  private final RelativeEncoder tiltEncoder;

  private final SparkClosedLoopController tiltClosedLoopController;

  private final double runSpeed = 0.8;

  //double lowerLimit = 7;
  double lowerLimit = 16;
  double upperLimit = 19;
  //19.5945

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    
    coralWheel = new SparkMax(CoralConstants.coralCANID, MotorType.kBrushless);
    coralWheelEncoder = coralWheel.getEncoder();

    tiltMax = new SparkMax(CoralConstants.tiltCANID, MotorType.kBrushless);
    tiltEncoder = tiltMax.getEncoder();

    tiltClosedLoopController = tiltMax.getClosedLoopController();

    coralWheel.configure(Configs.CoralConfig.coralWheelConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    tiltMax.configure(Configs.CoralConfig.coralTiltConfig, ResetMode.kResetSafeParameters,
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

  public void tiltEndEffector(double speed){
   if(tiltEncoder.getPosition() <= lowerLimit && (Math.signum(speed) == -1)){ 
      tiltMax.stopMotor();
    } else if (tiltEncoder.getPosition() >= upperLimit && (Math.signum(speed) == 1)){
      tiltMax.stopMotor();
    } else{
      tiltMax.set(speed);
    } 
    //tiltMax.set(speed);
  }

  public void tiltStop(){
    tiltMax.stopMotor();
  }

  public void tiltTo(double desiredPosition){
    tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition);
  }

  public void showEncoders(){
    SmartDashboard.putNumber("End Effector angle", tiltEncoder.getPosition());
    SmartDashboard.putNumber("Coral Wheel", coralWheelEncoder.getVelocity());
  }
}
