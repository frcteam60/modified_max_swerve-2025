// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import frc.robot.Configs;

public class MAXSwerveModule {
  private final SparkMax m_drivingSpark;
  private final SparkMax m_steeringSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final RelativeEncoder m_steeringEncoder;
  private final DutyCycleEncoder m_absoluteEncoder;
  //TODO abs encoder

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_steeringClosedLoopController;

  private double m_chassisAngularOffset = 0;
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
  public int ID;

  /**
   * Constructs a MAXSwerveModule and configures the driving and steering motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore`
   * Encoder.
   */
  public MAXSwerveModule(int drivingCANId, int steeringCANId, int encoderNumber, double chassisAngularOffset) {
    ID = drivingCANId;
    m_drivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless);
    m_steeringSpark = new SparkMax(steeringCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_steeringEncoder = m_steeringSpark.getEncoder();
    //m_steeringEncoder = m_steeringSpark.getEncoder();
    m_absoluteEncoder = new DutyCycleEncoder(encoderNumber);

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_steeringClosedLoopController = m_steeringSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_steeringSpark.configure(Configs.MAXSwerveModule.steeringConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_chassisAngularOffset = chassisAngularOffset;
    m_desiredState.angle = new Rotation2d(m_steeringEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  public void setEncoder(double offset){
    //Sets the relative steering encoder by the absolute encoder
    m_steeringEncoder.setPosition(Math.toRadians(m_absoluteEncoder.get() * 360) - offset);
    //System.out.println(m_steeringEncoder.getPosition());
    
  }
  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_steeringEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_steeringEncoder.getPosition() - m_chassisAngularOffset));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    // Optimize the reference state to avoid spinning further than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_steeringEncoder.getPosition()));

    // Command driving and steering SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_steeringClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;

    //System.out.println(ID + getState().toString());
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }

  public double returnModuleAngle(){
    return m_steeringEncoder.getPosition();
  }
  public double returnAbsAngle(){
    return m_absoluteEncoder.get();
  }

  public void voltageControl(Voltage voltage){
    m_drivingSpark.setVoltage(voltage);
  }
}
