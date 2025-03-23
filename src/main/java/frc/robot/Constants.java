// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.NTDouble.NTD;
import frc.robot.sds.ModuleConfiguration;
import frc.robot.sds.SdsModuleConfigurations;
import frc.robot.subsystems.CoralArm;

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
  public static String ReefLimelightName = "limelight-front";
  public static String UpperLimelightName = "limelight-upper";
  public static String VisionLimelightName = "limelight-vision";

  public static final class AutoConstants {
    public static double kPYController = 3;
    public static double kPXController = 3;
    public static double kPThetaController = 1;
    public static double kMaxSpeedMetersPerSecond = DriveConstants.MaxVelocityMetersPerSecond * .50;
    public static double kMaxAccelerationMetersPerSecondSquared = 4;
    public static Constraints kThetaControllerConstraints = new Constraints(
        DriveConstants.MaxAngularVelocityRadiansPerSecond * .5, (Math.PI * 2) / 2);

    public static PathConstraints constantConstraints = new PathConstraints(2.5, 2, 2.5 * Math.PI, 3 * Math.PI);

  }

  public static final ModuleConfiguration ModuleType = SdsModuleConfigurations.MK4N_L2;

  public static void configMotor(SparkMax motor, boolean Inverted) {
  }

  public static final class DriveConstants {
    public static final double kDrivePeriod = TimedRobot.kDefaultPeriod;

    // Distance between left and right wheels
    public static final double kTrackWidthMeters = 0.5842 / 2;
    // Distance between front and back wheels
    public static final double kTrackBaseMeters = 0.6096 / 2;

    private static final Translation2d kFrontLeftLocation = new Translation2d(kTrackBaseMeters, kTrackWidthMeters);
    private static final Translation2d kFrontRightLocation = new Translation2d(kTrackBaseMeters, -kTrackWidthMeters);
    private static final Translation2d kBackLeftLocation = new Translation2d(-kTrackBaseMeters, kTrackWidthMeters);
    private static final Translation2d kBackRightLocation = new Translation2d(-kTrackBaseMeters, -kTrackWidthMeters);

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);

    // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
    // The formula for calculating the theoretical maximum velocity is:
    // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
    // pi
    // By default this value is setup for a Mk3 standard module using Falcon500s to
    // drive.
    // An example of this constant for a Mk4 L2 module with NEOs to drive is:
    // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
    // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
    /**
     * The maximum velocity of the robot in meters per second.
     * <p>
     * This is a measure of how fast the robot should be able to drive in a straight
     * line.
     */
    public static final double kNeoFreeSpinRpm = 5676;
    public static final double MaxVelocityMetersPerSecond = (kNeoFreeSpinRpm / 60.0) *
        ModuleType.getDriveReduction() *
        ModuleType.getWheelDiameter() * Math.PI;

    public static final double MaxAngularVelocityRadiansPerSecond = MaxVelocityMetersPerSecond
        / Math.hypot(kTrackWidthMeters / 2, kTrackBaseMeters / 2) * .75;

  }

  public static final class ModuleConstants {

    public static final double MaxModuleAngularSpeedRadiansPerSecond = 2 *
        Math.PI;
    public static final double MaxModuleAngularAccelerationRadiansPerSecondSquared = 2 * Math.PI;

    // public static final int kEncoderCPR = 1024;
    // public static final double kWheelDiameterMeters = 0.15;
    // public static final double kDriveEncoderDistancePerPulse =
    // // Assumes the encoders are directly mounted on the wheel shafts
    // (kWheelDiameterMeters * Math.PI) / (double) kEncoderCPR;

    // public static final double kTurningEncoderDistancePerPulse =
    // // Assumes the encoders are on a 1:1 reduction with the module shaft.
    // (2 * Math.PI) / (double) kEncoderCPR;
    public static final double TurningEncoderDegreesPerPulse = Math
        .toDegrees(2. * Math.PI * ModuleType.getSteerReduction());

    // public static final double kPModuleTurningController = 1;

    // public static final double kPModuleDriveController = 1;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class ABSPoseOffset {
    // should we have a bunch of constant offset transformations?
    // ooh maybe have an offset class then call an offset pose function that takes
    // an offset class
    // that offset pose function then takes an parameter of a constantoffsetclass
    // and a given pose
    // then applies that offset transformation to the pose
    // returns the resultant pose
    public static Transform3d l4LeftPoseOffset = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    public static Transform3d l4RightPoseOffset = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    public static Transform3d l3LeftPoseOffset = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    public static Transform3d l3RightPoseOffset = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    public static Transform3d l2LeftPoseOffset = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
    public static Transform3d l2RightPoseOffset = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());

    public static Pose3d offsetPose(Pose3d tagPose, Transform3d transform) {
      Pose3d resultPose = tagPose.transformBy(transform);
      return resultPose;
    }
  }

  public static final class WaypointConstants {

    public static Pose2d CoralStationLeft = new Pose2d(11.5, 6.5, new Rotation2d(125));
    public static Pose2d CoralStationRight = new Pose2d(2, 1.7, new Rotation2d(-125));
    public static Pose2d ReefN = new Pose2d(6.3, 4.1, new Rotation2d(125));
    public static Pose2d ReefS = new Pose2d(2.7, 4.1, new Rotation2d(-125));
    public static Pose2d ReefSE = new Pose2d(3.6, 2.5, new Rotation2d(58));
    public static Pose2d ReefSW = new Pose2d(3.6, 5.67, new Rotation2d(-58));
    public static Pose2d ReefNE = new Pose2d(5.5, 2.4, new Rotation2d(-122));
    public static Pose2d ReefNW = new Pose2d(5.5, 5.8, new Rotation2d(122));
    public static Pose2d Processor = new Pose2d(6.35, 1.55, new Rotation2d(-90));

  }

  public final class SetpointConstants {
    // here is where we keep all the setpoints for the subsystem pid

    public class ElevatorSetpoints {
      public static NTDouble l1 = new NTDouble(0, "l1", "ElevatorSetpoints");
      public static NTDouble l2 = new NTDouble(0.47, "l2", "ElevatorSetpoints");
      public static NTDouble l3 = new NTDouble(0.81, "l3", "ElevatorSetpoints");
      public static NTDouble l4 = new NTDouble(1.36, "l4", "ElevatorSetpoints");

      public static NTDouble groundLevel = new NTDouble(0, "groundLevel", "ElevatorSetpoints");
      public static NTDouble algaeLow = new NTDouble(0.85, "algaeLow", "ElevatorSetpoints");
      public static NTDouble algaeHigh = new NTDouble(1.17, "algaeHigh", "ElevatorSetpoints");
      public static NTDouble coralStation = new NTDouble(0.155, "coralStation", "ElevatorSetpoints");
      public static NTDouble processor = new NTDouble(0.1, "processor", "ElevatorSetpoints");
      public static NTDouble groundAlgae = new NTDouble(0.2, "algaeGround", "ElevatorSetpoints");

    }
    // lowest level
    // L1
    // L2
    // L3
    // L4
    // public double[] mElevatorSetpointArray = {setpoints}

    public class CoralPivotAngles {
      // angle of release
      public static NTDouble l1 = new NTDouble(-0.03, "l1", "CoralPivotAngles"); // reef l1
      public static NTDouble lmid = new NTDouble(0.084, "lmid", "CoralPivotAngles"); // reef l2 and l3
      public static NTDouble l4 = new NTDouble(0.1, "l4", "CoralPivotAngles"); // reef l4
      public static NTDouble CoralSt = new NTDouble(-0.122, "CoralSt", "CoralPivotAngles"); // pointing up to recieve
                                                                                            // coral from hp
      public static NTDouble up = new NTDouble(-0.24, "up", "CoralPivotAngles"); // out of the way for intaking algae
      public static NTDouble down = new NTDouble(-0.24, "down", "CoralPivotAngles");
    }

    public class StrafeOffsets {
      // static double l1 = 0;
      public static NTDouble leftReef = new NTDouble(0.18, "leftReef", "StrafeOffsets");
      public static NTDouble leftL4 = new NTDouble(0.15, "leftL4", "StrafeOffsets");
      public static NTDouble rightReef = new NTDouble(-0.16, "rightReef", "StrafeOffsets");
      public static NTDouble rightL4 = new NTDouble(-0.18, "rightL4", "StrafeOffsets");
      public static NTDouble centerReef = new NTDouble(0.00, "centerReef", "StrafeOffsets");
      public static NTDouble processor = new NTDouble(0.05, "processor", "StrafeOffsets");
      public static NTDouble l1Left = new NTDouble(0.10, "l1Left", "StrafeOffsets");
      public static NTDouble l1Right = new NTDouble(0.10, "l1Right", "StrafeOffsets");
      public static NTDouble csStrafe = new NTDouble(0.00, "Coral Station Strafe", "StrafeOffsets");
      // static double CoralSt = 0;
    }

    public class DistanceOffsets {
      public static NTDouble leftReefScore = new NTDouble(0.6, "leftReefScore", "DistanceOffsets");
      public static NTDouble rightReefScore = new NTDouble(0.6, "rightReefScore", "DistanceOffsets");
      // public static NTDouble L4score = new NTDouble(0.35, "L4score",
      // "DistanceOffsets");
      public static NTDouble algaeReefGrab = new NTDouble(0.42, "algaeReefGrab", "DistanceOffsets");
      public static NTDouble reefCoralConfigure = new NTDouble(0.8, "reefCoralConfigure", "DistanceOffsets");
      public static NTDouble reefAlgaeConfigure = new NTDouble(0.9, "reefAlgaeConfigure", "DistanceOffsets");
      public static NTDouble reefAlgaeStow = new NTDouble(1.3, "reefAlgaeStow", "DistanceOffsets");
      public static NTDouble processorInitial = new NTDouble(0.95, "processorInitial", "DistanceOffsets");
      public static NTDouble processorScore = new NTDouble(0.45, "processorScore", "DistanceOffsets");
      public static NTDouble l1 = new NTDouble(0.5, "l1", "DistanceOffsets");
      public static NTDouble csConfigure = new NTDouble(0.7, "CoralStationConfigure", "DistanceOffsets");
      public static NTDouble csIntake = new NTDouble(0.55, "CoralStationIntake", "DistanceOffsets");
    }

    public class RotOffsets {
      public static NTDouble l1Left = new NTDouble(30, "l1Left", "RotOffsets");
      public static NTDouble l1Right = new NTDouble(-30, "l1Right", "RotOffsets");
      public static NTDouble none = new NTDouble(0, "none", "RotOffsets");
      public static NTDouble CoralSt = new NTDouble(0.166, "CoralSt", "RotOffsets");
    }

    public class AlgaeArmAngles {
      public static NTDouble up = new NTDouble(0, "up", "AlgaeArmAngles"); // hold algae in
      public static NTDouble down = new NTDouble(-0.3, "down", "AlgaeArmAngles"); // intake or outtake algae
      public static NTDouble processor = new NTDouble(-0.13, "processor", "AlgaeArmAngles");
      public static NTDouble groundPickUp = new NTDouble(-0.25, "ground pick up", "AlgaeArmAngles");
      public static NTDouble barge = new NTDouble(-0.1, "barge", "AlgaeArmAngles");
    }
    // L2 and L3 Angle
    // L4 Angle
    // Hold angle
    // grab angle? like where it will be when grabbing coral from the station
    // public double[] mCoralSetpointArray = {setpoints}

    // Algae Angles
    // angle for grabbing and releasing algae
    // angle for holding Algae
    public static class ConfigOption {

      public NTDouble algaeAngle;
      public NTDouble coralAngle;
      public NTDouble elevatorSetpoint;

      public ConfigOption(NTDouble coralAngle, NTDouble elevatorSetpoint, NTDouble algaeAngle) {

        this.coralAngle = coralAngle;
        this.elevatorSetpoint = elevatorSetpoint;
        this.algaeAngle = algaeAngle;

      }

    }

    public class Options {

      public Options() {

      }

      // reef
      public static ConfigOption l1 = new ConfigOption(CoralPivotAngles.l1, ElevatorSetpoints.l1, AlgaeArmAngles.up);
      public static ConfigOption l2 = new ConfigOption(CoralPivotAngles.lmid, ElevatorSetpoints.l2,
          AlgaeArmAngles.up);
      public static ConfigOption l3 = new ConfigOption(CoralPivotAngles.lmid, ElevatorSetpoints.l3,
          AlgaeArmAngles.up);
      public static ConfigOption l4 = new ConfigOption(CoralPivotAngles.l4, ElevatorSetpoints.l4, AlgaeArmAngles.up);
      // public static ConfigOption l2Right = new ConfigOption(CoralPivotAngles.lmid,
      // ElevatorSetpoints.l2, AlgaeArmAngles.up);
      // public static ConfigOption l3Right = new ConfigOption(CoralPivotAngles.lmid,
      // ElevatorSetpoints.l3, AlgaeArmAngles.up);
      // public static ConfigOpt2ion l4Right = new ConfigOption(CoralPivotAngles.l4,
      // ElevatorSetpoints.l4, AlgaeArmAngles.up);
      public static ConfigOption algaeLow = new ConfigOption(CoralPivotAngles.down, ElevatorSetpoints.algaeLow,
          AlgaeArmAngles.down);
      public static ConfigOption algaeHigh = new ConfigOption(CoralPivotAngles.down, ElevatorSetpoints.algaeHigh,
          AlgaeArmAngles.down);
      public static ConfigOption algaeGround = new ConfigOption(CoralPivotAngles.down, ElevatorSetpoints.groundAlgae,
          AlgaeArmAngles.down);
      // different spots
      public static ConfigOption coralPrep = new ConfigOption(CoralPivotAngles.up,
          NTD.of(ElevatorSetpoints.coralStation.get() + .2),
          AlgaeArmAngles.up);
      public static ConfigOption coralStationManual = new ConfigOption(CoralPivotAngles.CoralSt,
          ElevatorSetpoints.coralStation,
          AlgaeArmAngles.up);
      public static ConfigOption coralStationAuto = new ConfigOption(CoralPivotAngles.CoralSt,
          NTD.of(0.13),
          AlgaeArmAngles.up);
      public static ConfigOption processor = new ConfigOption(CoralPivotAngles.down, ElevatorSetpoints.processor,
          AlgaeArmAngles.processor);
      public static ConfigOption driveConfig = new ConfigOption(CoralPivotAngles.up, ElevatorSetpoints.groundLevel,
          AlgaeArmAngles.up);
      public static ConfigOption barge = new ConfigOption(CoralPivotAngles.up, ElevatorSetpoints.l4,
          AlgaeArmAngles.barge);
      public static ConfigOption coralAlign = new ConfigOption(CoralPivotAngles.up, ElevatorSetpoints.l3,
          AlgaeArmAngles.up);

      // to hold everything in while sitting there

      // public static double[] l1 = { CoralPivotAngles.l1, ElevatorSetpoints.l1,
      // XOffset.l1, YOffset.l1, RotOffset.l1,

      // AlgaeArmAngles.down };
      // public static double[] l2Left = { CoralPivotAngles.lmid,
      // ElevatorSetpoints.l2,
      // XOffset.left, YOffset.Reef,
      // RotOffset.none, AlgaeArmAngles.down };
      // public static double[] l3Left = { CoralPivotAngles.lmid,
      // ElevatorSetpoints.l3, XOffset.left, YOffset.Reef,
      // RotOffset.none, AlgaeArmAngles.down };
      // public static double[] l4Left = { CoralPivotAngles.l4, ElevatorSetpoints.l4,
      // XOffset.left, YOffset.Reef,
      // RotOffset.none, AlgaeArmAngles.down };
      // public static double[] l2Right = { CoralPivotAngles.lmid,
      // ElevatorSetpoints.l2, XOffset.right, YOffset.Reef,
      // RotOffset.none, AlgaeArmAngles.down };
      // public static double[] l3Right = { CoralPivotAngles.lmid,
      // ElevatorSetpoints.l3, XOffset.right, YOffset.Reef,
      // RotOffset.none, AlgaeArmAngles.down };
      // public static double[] l4Right = { CoralPivotAngles.l4, ElevatorSetpoints.l4,
      // XOffset.right, YOffset.Reef,
      // RotOffset.none, AlgaeArmAngles.down };
      // public static double[] CoralStation = { CoralPivotAngles.CoralSt,
      // ElevatorSetpoints.l2, XOffset.CoralSt,
      // YOffset.CoralSt, RotOffset.none, AlgaeArmAngles.down };
      // public static double[] Processor = { CoralPivotAngles.Out,
      // ElevatorSetpoints.groundLevel, XOffset.none,
      // YOffset.Algae, RotOffset.none, AlgaeArmAngles.up };
      // public static double[] AlgaeLow = { CoralPivotAngles.Out,
      // ElevatorSetpoints.l3, XOffset.none, YOffset.Algae,
      // RotOffset.none, AlgaeArmAngles.up };
      // public static double[] AlgaeHigh = { CoralPivotAngles.Out,
      // ElevatorSetpoints.l4, XOffset.none, YOffset.Algae,
      // RotOffset.none, AlgaeArmAngles.up };
      // positionList.add(l2Left); // 1
      // positionList.add(l3Left); // 2
      // positionList.add(l4Left); // 3
      // positionList.add(l2Right);// 4
      // positionList.add(l3Right);// 5
      // positionList.add(l4Right);// 6
      // positionList.add(CoralStation); // 7
      // positionList.add(Processor); // 8
      // positionList.add(AlgaeLow); // 9
      // positionList.add(AlgaeHigh); // 10

    }
  }

}
