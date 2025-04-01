// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants;
import frc.robot.Constants.ClimberConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;


import edu.wpi.first.wpilibj.Timer;




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
  Timer matchTimer;

  double runSpeed = 1;
  //166 range
  //higher positive
  //lower negative
  double lowerLimit = -671;
  double upperLimit = 0;

  double readyToClimbAngle = -380;

  /** Creates a new DriveSubsystem. */
  public Climber() {
    matchTimer = new Timer();
    climberMotor = new SparkMax(ClimberConstants.climberCANID, MotorType.kBrushless);
    climberEncoder = climberMotor.getEncoder();

    climberMotor.configure(Configs.ClimberConfigure.climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("ClimberPosition", climberEncoder.getPosition());

  }

  public void climberIn(){
    runClimber(-runSpeed);
  }
  public void climberOut(){
    runClimber(runSpeed);
  }

  public void runClimber(double speed){
    if(climberEncoder.getPosition() <= lowerLimit && (Math.signum(speed) == -1)){ 
      climberMotor.stopMotor();
    } else if (climberEncoder.getPosition() >= upperLimit && (Math.signum(speed) == 1)){
      climberMotor.stopMotor();
    } else{
      climberMotor.set(speed);
    } 
    //climberMotor.set(speed);
  }

  public void stopClimber(){
    climberMotor.stopMotor();
  }

  public double setClimberPosition(double desiredPosition){
    double error = (desiredPosition-climberEncoder.getPosition());
    climberMotor.set(error*0.05);
    return error;
  }

  public void noLimitClimber(double speed){
    climberMotor.set(speed);
  }

  public void defaultClimber(){
    if(matchTimer.getMatchTime() < 20 && matchTimer.getMatchTime() > 10){
      setClimberPosition(readyToClimbAngle);
    } else {
      climberMotor.stopMotor();
    }
  }


}
