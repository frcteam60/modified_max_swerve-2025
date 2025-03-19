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

public class TiltSubsystem extends SubsystemBase {

  private final SparkMax tiltMax;
  private final RelativeEncoder tiltEncoder;

  private final SparkClosedLoopController tiltClosedLoopController;

  private final double runSpeed = 0.8;

  //double lowerLimit = 7;
  double lowerLimit = 3;
  //35
  double upperLimit = 95;
  //95
  //19.5945

  boolean atDesiredTilt = false;

  //TODO get these angles
  double L4Angle = 90;
  double L3Angle = 95;
  double L2Angle = 95;
  double L1Angle = 90;
  double L0Angle = 5;
 
  /** Creates a new CoralSubsystem. */
  public TiltSubsystem() {

    tiltMax = new SparkMax(CoralConstants.tiltCANID, MotorType.kBrushless);
    tiltEncoder = tiltMax.getEncoder();

    tiltClosedLoopController = tiltMax.getClosedLoopController();
    tiltMax.configure(Configs.CoralConfig.coralTiltConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    showEncoders();
  }


  public void tiltEndEffector(double speed){
   //System.out.println(speed);
   if(tiltEncoder.getPosition() <= lowerLimit && (Math.signum(speed) == -1)){ 
    //System.out.println("A");
      tiltMax.stopMotor();
    } else if (tiltEncoder.getPosition() >= upperLimit && (Math.signum(speed) == 1)){
      //System.out.println("B");
      tiltMax.stopMotor();
    } else{
      //System.out.println("C");
      tiltMax.set(speed);
    }
    //tiltMax.set(speed*0.25);
  }

  public void tiltStop(){
    tiltMax.stopMotor();
  }

  public void tiltTo(double desiredPosition){
    SmartDashboard.putNumber("desiredtilt", desiredPosition);
    //System.out.println("desired position" + desiredPosition);
    if(tiltEncoder.getPosition() <= (lowerLimit) && (desiredPosition <=(lowerLimit))){ 
     //System.out.println("A");
       tiltMax.stopMotor();
     } else if (tiltEncoder.getPosition() >= (upperLimit) && (desiredPosition >= (upperLimit))){
       //System.out.println("B");
       tiltMax.stopMotor();
     } else{
       //System.out.println("C");
       if (desiredPosition > tiltEncoder.getPosition()){
        //Up 
        tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
       }else{
        //down
        tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
       }
     }

     

     /* if(tiltEncoder.getPosition()>desiredPosition){
      tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
     } else {
      tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
     } */
  
  }

  public void showEncoders(){
    SmartDashboard.putNumber("End Effector angle", tiltEncoder.getPosition());
    SmartDashboard.putNumber("EndEffectorSpeed", tiltEncoder.getVelocity());
  }

  public void stopTilt(){
    tiltMax.stopMotor();
  }

  public void lineUpL4(){
    tiltTo(L4Angle);
  }
  public void lineUp3(){
    tiltTo(L3Angle);
  }
  public void lineUpL2(){
    tiltTo(L2Angle);
  }
  public void lineUpL1(){
    tiltTo(L1Angle);
  }
  public void lineUpL0(){
    tiltTo(L0Angle);
  }

  
}
