// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.FeederConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {

  private final SparkMax feederMax;
  private final RelativeEncoder feederEncoder;

  private final double runSpeed = 0.5;

  /** Creates a new FeederSubsystem. */
  public FeederSubsystem() {
    
    feederMax = new SparkMax(FeederConstants.feederCANID, MotorType.kBrushless);

    feederEncoder = feederMax.getEncoder();

    SparkMaxConfig feederConfig = new SparkMaxConfig();
    feederConfig.inverted(true);
    feederMax.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void runFeeder() {
    feederMax.set(runSpeed);
  }

  public void reverseFeeder() {
    feederMax.set(-1*runSpeed);
  }

  public void stopFeeder() {
    feederMax.stopMotor();
  }
}
