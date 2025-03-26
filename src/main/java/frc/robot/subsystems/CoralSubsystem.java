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
  double beginningPosition = 0;
  boolean coralAllIn = false;
  boolean coralAllOut = false;
  //TODO what way does this default
  boolean lastBeamBreak = false;


  //79.6 revolutions after see it to all in //39.8
  //to run it flush front, instead do half

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
    //showEncoders();
    SmartDashboard.putBoolean("beam break", getBeamBreak());
  }

  public void coralIntake() {
    coralWheel.set(runSpeed);
  }

  public void coralExpel() {
    coralWheel.set(-runSpeed);
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

  public boolean runIn(){
    boolean currentBeamBreak = getBeamBreak();
    //beam break true assumed beam broken

    if(currentBeamBreak && !lastBeamBreak){// if beamBreak true and last beam break false i.e. beam break changed to true
      beginningPosition = coralWheelEncoder.getPosition();//sets this position as reference positions
    } 

    if((beginningPosition - coralWheelEncoder.getPosition()) >= 79.6 && currentBeamBreak){//if coral has traveled 79.6 rotations since breaking beam
      coralAllIn = true;//then coral is all in
    } else{
      coralAllIn = false;//else coral is not all in
    }

    if(coralAllIn){//stops wheel if coral all in 
      coralWheel.stopMotor();
    } else {//runs wheel if coral not all in
      coralWheel.set(runSpeed);
    }

    boolean lastBeamBreak = currentBeamBreak;//current beam break = last beam break

    //return 
    return coralAllIn;
  }

  public boolean runOut(){
    boolean currentBeamBreak = getBeamBreak();
    //beam break true assumed beam broken

    if(!currentBeamBreak && lastBeamBreak){// if beamBreak false and last beam break true i.e. beam break changed to false
      beginningPosition = coralWheelEncoder.getPosition();//sets this position as reference positions
    } 
    //TODO need different #
    if((coralWheelEncoder.getPosition() - beginningPosition) >= 79.6 && !currentBeamBreak){//if coral has traveled -79.6 rotations since leaving beam break
      coralAllOut = true;//then coral is all out
    } else{
      coralAllIn = false;//else coral is not all out
    }

    if(coralAllOut){//stops wheel if coral all out 
      coralWheel.stopMotor();
    } else {//runs wheel if coral not all out
      coralWheel.set(-runSpeed);
    }

    boolean lastBeamBreak = currentBeamBreak;//current beam break = last beam break

    //return 
    return coralAllOut;
  }


  public void showEncoders(){
    SmartDashboard.putNumber("Coral Wheel", coralWheelEncoder.getVelocity());
  }
}
