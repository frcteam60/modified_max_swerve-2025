// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;

public class CoralSubsystem extends SubsystemBase {

  private final SparkMax CoralMax;
  private final RelativeEncoder CoralEncoder;

  private final double runSpeed = 0.8;

  /** Creates a new CoralSubsystem. */
  public CoralSubsystem() {
    
    CoralMax = new SparkMax(CoralConstants.coralCANID, MotorType.kBrushless);

    CoralEncoder = CoralMax.getEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void coralIntake() {
    CoralMax.set(runSpeed);
  }

  public void coralExpel() {
    CoralMax.set(-1*runSpeed);
  }

  public void coralStop() {
    CoralMax.stopMotor();
  }
}
