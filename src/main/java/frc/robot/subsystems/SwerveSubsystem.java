package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.swerve.SwerveModule;

//add motor channel numbers later
public class SwerveSubsystem extends SubsystemBase {

        private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(2);

        private final SwerveModule fLSwerve = new SwerveModule(15, 14, false, true,
                        null, 1.792);
        private final SwerveModule fRSwerve = new SwerveModule(13, 12, false, true,
                        null, 1.0);
        private final SwerveModule bLSwerve = new SwerveModule(17, 16, false, true,
                        null, 0.749);
        private final SwerveModule bRSwerve = new SwerveModule(11, 10, false, true,
                        null, 1.43);

        private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
                drive(xPercent, yPercent, rotPercent, fieldRelative, 0, 0);
        }

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative, double a,
                        double b) {

                var xSpeed = xRateLimiter.calculate(xPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yRateLimiter.calculate(yPercent) * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotRateLimiter.calculate(rotPercent)
                                * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                ChassisSpeeds chasSpeed = fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot);

                // TODO: DEFINE MAX SPEED
                var swerveModuleStates2 = DriveConstants.kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(chasSpeed, 0.2),
                                new Translation2d(DriveConstants.kTrackBaseMeters * a * 1.5,
                                                DriveConstants.kTrackWidthMeters * b * 1.5));

                driveStates(swerveModuleStates2);

        }

        public void driveAuto(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {

                var xSpeed = xPercent * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yPercent * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotPercent * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                ChassisSpeeds chasSpeed = fieldRelative
                                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getRotation())
                                : new ChassisSpeeds(xSpeed, ySpeed, rot);

                // TODO: DEFINE MAX SPEED
                var swerveModuleStates2 = DriveConstants.kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(chasSpeed, 0.2));

                driveStates(swerveModuleStates2);
        }

        public void stop() {
                SwerveModuleState[] states = getModuleStates();
                for (var state : states) {
                        state.speedMetersPerSecond = 0;
                }
                driveStates(states);
        }

        public void xpattern() {
                SwerveModuleState[] states = getModuleStates();
                for (var i = 0; i < states.length; i++) {
                        states[i].angle = DriveConstants.kinematics.getModules()[i].getAngle();
                        states[i].speedMetersPerSecond = 0;
                }
                driveStates(states);
        }

        private Rotation2d yawOffset = Rotation2d.k180deg;

        // have we ever seen a tag?
        private boolean seenMT;
        private final Field2d field = new Field2d();
        private final SwerveDrivePoseEstimator ometry = new SwerveDrivePoseEstimator(
                        Constants.DriveConstants.kinematics,
                        getRotation(),
                        new SwerveModulePosition[] {
                                        fLSwerve.getPosition(),
                                        fRSwerve.getPosition(),
                                        bLSwerve.getPosition(),
                                        bRSwerve.getPosition()
                        },
                        // Starting pos
                        new Pose2d(7.6, 4, Rotation2d.fromDegrees(0)),
                        VecBuilder.fill(0.1, 0.1, 0.4),
                        VecBuilder.fill(0.9, 0.9, 0.9));

        public Rotation2d getRotation() {
                return gyro.getRotation2d().minus(yawOffset);
        }

        public void zeroYaw() {
                if (gyro.getRotation2d() != null) {
                        yawOffset = gyro.getRotation2d();
                }
        }

        private void doMegatag(String limelight) {
                boolean useMegaTag2 = false; // set to false to use MegaTag1
                boolean doRejectUpdate = false;
                if (useMegaTag2 == false) {
                        LimelightHelpers.PoseEstimate mt1 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue(limelight);

                        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
                                if (mt1.rawFiducials[0] == null) {
                                        return;
                                }
                                if (mt1.rawFiducials[0].ambiguity > .7) {
                                        doRejectUpdate = true;
                                }
                                if (mt1.rawFiducials[0].distToCamera > 3) {
                                        doRejectUpdate = true;
                                }
                        }
                        if (mt1.tagCount == 0) {
                                doRejectUpdate = true;
                        }

                        if (!doRejectUpdate) {
                                seenMT = true;
                                ometry.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
                                ometry.addVisionMeasurement(
                                                mt1.pose,
                                                mt1.timestampSeconds);
                        }
                } else if (useMegaTag2 == true) {
                        LimelightHelpers.SetRobotOrientation(limelight,
                                        ometry.getEstimatedPosition().getRotation()
                                                        .getDegrees(),
                                        0, 0, 0, 0, 0);
                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
                        // heeeeeeelloooooo
                        // if our angular velocity is greater than 720 degrees per second,
                        // ignore vision updates
                        if (Math.abs(gyro.getRate()) > 720) {
                                doRejectUpdate = true;
                        }
                        if (mt2.tagCount == 0) {
                                doRejectUpdate = true;
                        }
                        if (!doRejectUpdate) {
                                seenMT = true;
                                // mine
                                // ometry.setVisionMeasurementStdDevs(VecBuilder.fill(2, 2, 5));
                                // limelights
                                ometry.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
                                ometry.addVisionMeasurement(
                                                mt2.pose,
                                                mt2.timestampSeconds);

                        }
                }
        }

        @Override
        public void periodic() {
                doMegatag(Constants.ReefLimelightName);
                doMegatag(Constants.UpperLimelightName);

                ometry.update(
                                getRotation(),
                                new SwerveModulePosition[] {
                                                fLSwerve.getPosition(),
                                                fRSwerve.getPosition(),
                                                bLSwerve.getPosition(),
                                                bRSwerve.getPosition()
                                }

                );

                field.setRobotPose(getPose());
        }

        public Pose2d getPose() {
                return ometry.getEstimatedPosition();
        }

        public void resetOmetry(Pose2d pose) {
                ometry.resetPosition(
                                getRotation(),
                                new SwerveModulePosition[] {
                                                fLSwerve.getPosition(),
                                                fRSwerve.getPosition(),
                                                bLSwerve.getPosition(),
                                                bRSwerve.getPosition()
                                },
                                pose);
        }

        private SwerveModuleState[] getModuleStates() {
                SwerveModuleState[] states = {
                                fLSwerve.getState(),
                                fRSwerve.getState(),
                                bLSwerve.getState(),
                                bRSwerve.getState(),
                };
                return states;
        }

        private ChassisSpeeds getSpeeds() {
                return DriveConstants.kinematics.toChassisSpeeds(getModuleStates());
        }

        public void driveStates(SwerveModuleState[] desiredStates) {
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates,
                                Constants.DriveConstants.MaxVelocityMetersPerSecond);

                SwerveModuleState[] optimizedSwerveModuleStates = {
                                fLSwerve.optimizeModuleState(desiredStates[0]),
                                fRSwerve.optimizeModuleState(desiredStates[1]),
                                bLSwerve.optimizeModuleState(desiredStates[2]),
                                bRSwerve.optimizeModuleState(desiredStates[3]),
                };

                fLSwerve.setDesiredState(optimizedSwerveModuleStates[0]);
                fRSwerve.setDesiredState(optimizedSwerveModuleStates[1]);
                bLSwerve.setDesiredState(optimizedSwerveModuleStates[2]);
                bRSwerve.setDesiredState(optimizedSwerveModuleStates[3]);
        }

        public SwerveSubsystem() {

                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        e.printStackTrace();
                        throw new RuntimeException(e);
                }
                // Configure AutoBuilder last
                AutoBuilder.configure(
                                this::getPose, // Robot pose supplier
                                this::resetOmetry, // Method to reset odometry (will be called if your auto has a
                                                   // starting pose)
                                this::getSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                (speeds, feedforwards) -> {
                                        var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                                                        ChassisSpeeds.discretize(speeds, .02));
                                        driveStates(swerveModuleStates);
                                }, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                                new PPHolonomicDriveController( // HolonomicPathFollowerConfig, this should likely live
                                                                // in your
                                                                // Constants class
                                                new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                                                new PIDConstants(3, 0.0, 0.0) // Rotation PID constants
                                // Max module speed, in m/s // Drive base radius in meters. Distance from robot
                                // center to
                                // furthest module.
                                // Default path replanning config. See the API
                                // for the options here
                                ),
                                config,
                                () -> {
                                        // Boolean supplier that controls when the path will be mirrored for the red
                                        // alliance
                                        // This will flip the path being followed to the red side of the field.
                                        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                                        var alliance = DriverStation.getAlliance();
                                        if (alliance.isPresent()) {
                                                return alliance.get() == DriverStation.Alliance.Red;
                                        }
                                        return false;
                                },
                                this // Reference to this subsystem to set requirements
                );
                Shuffleboard.getTab("Debug").addDouble("drive velocity unfiltered",
                                () -> fLSwerve.driveMotor.getEncoder().getVelocity());
                Shuffleboard.getTab("Debug").addDouble("robot angle from april tags",
                                () -> LimelightHelpers.getBotPose2d("limelight-back").getRotation().getDegrees());
                Shuffleboard.getTab("Debug").addDouble("robot angle from navx",
                                () -> gyro.getRotation2d().getDegrees());
                Shuffleboard.getTab("Debug").addDouble("yaw offset", () -> yawOffset.getDegrees());
                Shuffleboard.getTab("Debug").add(field);

                var sysIdRoutine = new SysIdRoutine(
                                new SysIdRoutine.Config(),
                                new SysIdRoutine.Mechanism(
                                                (voltage) -> this.runVolts(voltage.in(Volts)),
                                                null, // No log consumer, since data is recorded by URCL
                                                this));

                Shuffleboard.getTab("SysId").add("Quasi Forward Swerve",
                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
                Shuffleboard.getTab("SysId").add("Quasi Backward Swerve",
                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
                Shuffleboard.getTab("SysId").add("Dynamic Forward Swerve",
                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
                Shuffleboard.getTab("SysId").add("Dynamic Backaward Swerve",
                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));

        }

        private void runVolts(double in) {
                fLSwerve.runVolts(in);
                fRSwerve.runVolts(in);
                bLSwerve.runVolts(in);
                bRSwerve.runVolts(in);
        }
}