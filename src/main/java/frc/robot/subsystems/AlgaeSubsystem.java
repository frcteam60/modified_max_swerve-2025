// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants.AlgaeConstants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeSubsystem extends SubsystemBase {

  private final double runSpeed = 0.8;

  private final SparkMax leftAlgaeMax,rightAlgaeMax;
  private final RelativeEncoder leftAlgaeEncoder,rightAlgaeEncoder;
  /** Creates a new AlgaeSubsystem. */
  public AlgaeSubsystem() {
    leftAlgaeMax = new SparkMax(AlgaeConstants.algaeLeftCANID, MotorType.kBrushless);
    rightAlgaeMax = new SparkMax(AlgaeConstants.algaeRightCANID, MotorType.kBrushless);

    leftAlgaeEncoder = leftAlgaeMax.getEncoder();
    rightAlgaeEncoder = rightAlgaeMax.getEncoder();

    SparkMaxConfig invertedConfig = new SparkMaxConfig();
    invertedConfig.inverted(true);
    leftAlgaeMax.configure(invertedConfig, null, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showEncoders();
  }

  public void showEncoders() {
    SmartDashboard.putNumber("leftAlgae",leftAlgaeEncoder.getPosition());
    SmartDashboard.putNumber("rightAlgae",rightAlgaeEncoder.getPosition());
  }

  public void algaeIntake() {
    leftAlgaeMax.set(-1*runSpeed);
    rightAlgaeMax.set(-1*runSpeed);
  }

  public void algaeExpel() {
    leftAlgaeMax.set(runSpeed);
    rightAlgaeMax.set(runSpeed);
  }

  public void algaeStop(){
    leftAlgaeMax.stopMotor();
    rightAlgaeMax.stopMotor();

  }
}
