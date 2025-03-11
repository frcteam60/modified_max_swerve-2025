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
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
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



import static edu.wpi.first.units.Units.Volts;

import java.util.function.Consumer;

public class Climber extends SubsystemBase {

  private final WPI_TalonSRX climberOne;
  private final WPI_TalonSRX climberTwo;  

  double runSpeed = 0.5;
  

  /** Creates a new DriveSubsystem. */
  public Climber() {

    climberOne = new WPI_TalonSRX(ClimberConstants.climberOneCANID);
    climberOne.setSafetyEnabled(true);
    climberOne.setNeutralMode(NeutralMode.Brake);

    climberTwo = new WPI_TalonSRX(ClimberConstants.climberTwoCANID);
    climberTwo.setSafetyEnabled(true);
    climberTwo.setNeutralMode(NeutralMode.Brake);
    climberTwo.follow(climberOne);

/*     //climberOne.configAllSettings(null);
    climberOne.enableCurrentLimit(true);
    climberOne.follow(climberOne);
    climberOne.getStatorCurrent();
    climberOne.setInverted(false);
    climberOne.setSafetyEnabled(true);
    climberOne.stopMotor();
    climberOne.setNeutralMode(null); */

  }

  @Override
  public void periodic() {

  }

  public void climberIn(){
    runClimber(1);
  }
  public void climberOut(){
    runClimber(-1);
  }

  public void runClimber(double speed){
/*     if(cli.getPosition() <= lowerLimit && (Math.signum(speed) == -1)){ 
      elevatorOneMax.stopMotor();
      elevatorTwoMax.stopMotor();
    } else if (elevatorOneEncoder.getPosition() >= upperLimit && (Math.signum(speed) == 1)){
      elevatorOneMax.stopMotor();
      elevatorTwoMax.stopMotor();
    } else{
      elevatorOneMax.set(0.5*speed);
      elevatorTwoMax.set(0.5*speed);
    } */

    climberOne.set(runSpeed*speed);
  }

  public void stopClimber(){
    climberOne.stopMotor();
    climberTwo.stopMotor();

  }



}
