package frc.robot;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class ElevatorConfigures{
        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();

        static {
            elevatorConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(45);
            /* elevatorConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second */

            elevatorConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // TODO tune PIDlop
                    //.pidf(drivingVelocityFeedForward, steeringFactor, drivingFactor, drivingVelocityFeedForward)
                    .pid(0.2, 0, 0)
                    //import edu.wpi.first.wpilibj.Preferences;
                    .outputRange(-0.07, 0.60, ClosedLoopSlot.kSlot0);
                    //.outputRange(-0.09, 0.60, ClosedLoopSlot.kSlot0);
                    //.pid(0, 0, 0, ClosedLoopSlot.kSlot0);
                    //.pid(0, 0, 0, ClosedLoopSlot.kSlot1);
                    //.pid(1, 0, 0)
                    //.velocityFF(drivingVelocityFeedForward)
       
        }
        
                
    }
    public static final class ClimberConfigure{
        public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

        static {
            climberConfig
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(45);
        }
        
                
    }
    public static final class CoralConfig{
        public static final SparkMaxConfig coralWheelConfig = new SparkMaxConfig();
        public static final SparkMaxConfig coralTiltConfig = new SparkMaxConfig();

        static {
            coralWheelConfig
                    .idleMode(IdleMode.kBrake)
                    //TODO check smart current limit
                    .smartCurrentLimit(45);
/*             coralWheelConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second */
/*             coralWheelConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    //.pid(1, 0, 0)
                    .velocityFF(0)
                    .outputRange(-1, 1); */

            coralTiltConfig  
                    .idleMode(IdleMode.kBrake)
                    .inverted(true)
                    .smartCurrentLimit(30);
            /* steeringConfig.encoder
                    .positionConversionFactor(steeringFactor) // radians
                    .velocityConversionFactor(steeringFactor / 60.0); // radians per second        */ 
            coralTiltConfig.closedLoop
                    //.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .iMaxAccum(0.0003)
                    //.pid(0.005, 0.0002, 2000)
                    .pid(0.02, 0.0002, 200, ClosedLoopSlot.kSlot0)
                    //.pid(0, 0, 0, ClosedLoopSlot.kSlot1);
                    .outputRange(-0.5, 1, ClosedLoopSlot.kSlot0)
                    //Slot 1 up
                    .pid(0.04, 0.0002, 100, ClosedLoopSlot.kSlot1)                    
                    .outputRange(-0.5, 0.7, ClosedLoopSlot.kSlot1);
                    //p 0.01
                    //.pid(0.0, 0, 0)
        }
    }


    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig steeringConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            //double steeringFactor = 2 * Math.PI;
            //double steeringFactor = (7.0/96)* 2 * Math.PI;
            double steeringFactor = 0.4575;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    //TODO check smart current limit
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    //.pid(1, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            steeringConfig
                    .inverted(true) 
                    //added invert here instead of encoder    
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            steeringConfig.encoder
                    .positionConversionFactor(steeringFactor) // radians
                    .velocityConversionFactor(steeringFactor / 60.0); // radians per second        
            /*steeringConfig.absoluteEncoder
                    // Invert the steering encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(steeringFactor) // radians
                    .velocityConversionFactor(steeringFactor / 60.0); // radians per second*/
            steeringConfig.closedLoop
                    //.feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    //.pid(0.0, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the steering motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    //.positionWrappingInputRange(0, steeringFactor);
                    .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}
