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
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;

public class TiltSubsystem extends SubsystemBase {

  private final SparkMax tiltMax;
  private final RelativeEncoder tiltEncoder;

  private final SparkClosedLoopController tiltClosedLoopController;

  private final double runSpeed = 0.8;

  //double lowerLimit = 7;
  double lowerLimit = 0;
  //35
  double upperLimit = 105;//95
  //95
  //19.5945

  boolean atDesiredTilt = false;

  //TODO test 1-3
  double L4Angle = 100;//90
  double L3Angle = 110;//95
  double L2Angle = 108;//95
  double L1Angle = 100;
  double L0Angle = 0;
  //double L0Angle = -4.33;
  //double L0Angle = 5;


  //TODO fix these angles
  double upperAlgae = 27;//36
  double lowerAlgae = 27.5;//36
  double barge = 105;//95
  double processor = 27;//8.30954?

  double currentTilt;

  double elevatorHeight = 0;
  double highestSafeElevatorHeight = 7;
  double minimumPivotWhenUp = 30;
 
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
    currentTilt = tiltEncoder.getPosition();
  }


  public void tiltEndEffector(double speed){

   //System.out.println(speed);
  if(elevatorHeight > highestSafeElevatorHeight && (currentTilt < minimumPivotWhenUp && speed <= 0)){
    tiltStop();
  }
   else if(currentTilt <= lowerLimit && (Math.signum(speed) == -1)){ 
    //System.out.println("A");
    tiltStop();
    } else if (currentTilt >= upperLimit && (Math.signum(speed) == 1)){
      //System.out.println("B");
      tiltStop();
    } else{
      //System.out.println("C");
      tiltMax.set(speed);
    }
    //tiltMax.set(speed*0.25);
  }

  public void tiltStop(){
    if(elevatorHeight > highestSafeElevatorHeight && currentTilt < minimumPivotWhenUp*.9) {
      tiltMax.set(runSpeed/2);
    } else {
      tiltMax.stopMotor();
    }
  }

  public void tiltTo(double desiredPosition){
    SmartDashboard.putNumber("desiredtilt", desiredPosition);
    if(elevatorHeight > highestSafeElevatorHeight && desiredPosition < minimumPivotWhenUp){
      desiredPosition = minimumPivotWhenUp;
    }
    double position = currentTilt;
    //System.out.println("desired position" + desiredPosition);
    if(position <= (lowerLimit) && (desiredPosition <=(lowerLimit))){ 
     //System.out.println("A");
       tiltMax.stopMotor();
     } else if (position >= (upperLimit) && (desiredPosition >= (upperLimit))){
       //System.out.println("B");
       tiltMax.stopMotor();
     } else{
       //System.out.println("C");
       
       if (desiredPosition > position){
        //Up 
        tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
       }else{
        //down
        tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
       }
     }

    if(Math.abs(desiredPosition-position) > 1){
      atDesiredTilt = false;
    } else {
      atDesiredTilt = true;
    }
     

     /* if(tiltEncoder.getPosition()>desiredPosition){
      tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0);
     } else {
      tiltClosedLoopController.setReference(desiredPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1);
     } */
  
  }

  public void showEncoders(){
    SmartDashboard.putNumber("End Effector angle", currentTilt);
    SmartDashboard.putNumber("EndEffectorSpeed", tiltEncoder.getVelocity());
  }

  public void stopTilt(){
    tiltMax.stopMotor();
  }

  public void lineUpL4(){
    tiltTo(L4Angle);
  }
  public void lineUpL3(){
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

  public void lineUpBarge(){
    tiltTo(barge);
  }
  public void lineUpProcessor(){
    tiltTo(processor);
  }
  public void lineUpLowerAlgae(){
    tiltTo(lowerAlgae);
  }
  public void lineUpUpperAlgae(){
    tiltTo(upperAlgae);
  }

  public boolean atDesiredTilt(){
    return atDesiredTilt;
  }

  public double returnTilt(){
    return currentTilt;
  }

  public void updateElevatorHeight(double height) {
    elevatorHeight = height;
  }
}
