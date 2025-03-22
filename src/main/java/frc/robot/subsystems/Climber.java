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

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;

import java.lang.invoke.VolatileCallSite;
import java.security.PublicKey;
import java.util.List;
import java.util.function.Consumer;


import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.BaseTalonConfiguration;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.*;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;



import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

public class Climber extends SubsystemBase {

  private final SparkMax climberMotor;
  private final RelativeEncoder climberEncoder;

  double runSpeed = 0.25;

  double lowerLimit = 0;
  double upperLimit = 0.5;

  /** Creates a new DriveSubsystem. */
  public Climber() {
    climberMotor = new SparkMax(ClimberConstants.climberOneCANID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();

    climberMotor.configure(Configs.ClimberConfigure.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {

  }

  public void climberIn(){
    climberMotor.set(-runSpeed);
  }
  public void climberOut(){
    climberMotor.set(runSpeed);
  }

  public void runClimber(double speed){
 /*    if(climberEncoder.getPosition() <= lowerLimit && (Math.signum(speed) == -1)){ 
      climberMotor.stopMotor();
    } else if (climberEncoder.getPosition() >= upperLimit && (Math.signum(speed) == 1)){
      climberMotor.stopMotor();
    } else{
      climberMotor.set(runSpeed*speed);
    }  */
    climberMotor.set(runSpeed*speed);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
  }



}
