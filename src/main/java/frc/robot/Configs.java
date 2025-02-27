package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
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
