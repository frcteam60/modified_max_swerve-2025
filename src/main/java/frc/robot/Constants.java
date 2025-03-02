// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.Matrix;

import com.pathplanner.lib.config.RobotConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final double posJoystickSensitivity = 2;
  public static final double rotJoystickSensitivity = 2;
  public static final class RobotConstants {
    public static final double robotWidthWithBumpers = 0.0254; // 36in   
    
  }

  public static final class ElevatorConstants{
    public static final int elevatorOneCANID = 18;
    public static final int elevatorTwoCANID = 1;
    // elevator front one 18
    // rear two 19
    //public static final int
  }
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(22.625);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(22.625);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    /*
    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;
            //TODO abs offset?
    */
    //Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = 0;
    public static final double kFrontRightChassisAngularOffset = Math.PI;
    public static final double kBackLeftChassisAngularOffset = 0;
    public static final double kBackRightChassisAngularOffset = Math.PI;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 7;
    public static final int kRearLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 5;
    public static final int kRearRightDrivingCanId = 3;

    public static final int kFrontLeftSteeringCanId = 8;
    public static final int kRearLeftSteeringCanId = 2;
    public static final int kFrontRightSteeringCanId = 6;
    public static final int kRearRightSteeringCanId = 4;

    //public static final boolean kGyroReversed = false;
    public static final boolean kGyroReversed = false;


    //Absolute Encoder Port
    public static final int kFrontLeftEncoderNum = 3;
    public static final int kRearLeftEncoderNum = 0;
    public static final int kFrontRightEncoderNum = 2;
    public static final int kRearRightEncoderNum = 1;

    //Absolute Encoder offsets
    //Our relative encoders output radians after we apply the conversion factor
    /* public static final double kFrontLeftAbsOffset = Math.toRadians(0.009997 * 360);
    public static final double kRearLeftAbsOffset = Math.toRadians(0.771441 * 360);
    public static final double kFrontRightAbsOffset = Math.toRadians(0.110444 * 360);
    public static final double kRearRightAbsOffset = Math.toRadians(0.238739 * 360); */
    public static final double kFrontLeftAbsOffset = Math.toRadians(0.00 * 360);
    public static final double kRearLeftAbsOffset = Math.toRadians(0.67 * 360);
    public static final double kFrontRightAbsOffset = Math.toRadians(0.901 * 360);
    public static final double kRearRightAbsOffset = Math.toRadians(0.34 * 360);
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    //public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelDiameterMeters = (3.58 * 0.0254);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    //public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDrivingMotorReduction = (102/13);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class PathPlannerConstants{
    //public static final RobotConfig pathRobotConfig = RobotConfig;
    //TODO
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants{
    public static final double turningP = -0.1;
    public static final double strafeP = -0.3;
  
    
  }

  public static class Vision {
    //TODO use these in vision

        public static final String kCameraName = "Arducam_OV9782_USB_Camera";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                //new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
                new Transform3d(new Translation3d(0.32385, 0.0, 0.2032), new Rotation3d(0, 0, 0));
        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  
    }
}
