package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.io.File;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.NTDouble;
import swervelib.SwerveDrive;
import swervelib.motors.SparkMaxSwerve;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
        private final SlewRateLimiter xRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter yRateLimiter = new SlewRateLimiter(2);
        private final SlewRateLimiter rotRateLimiter = new SlewRateLimiter(2);

        private SwerveDrive swerveDrive;

        private Rotation2d driverRotationOffset = Rotation2d.kZero;

        public void zeroDriverRotation() {
                driverRotationOffset = getRotation();
        }

        public void driveHuman(double xPercent, double yPercent,
                        double rotPercent, boolean fieldRelative) {
                var xSpeed = xRateLimiter.calculate(xPercent) *
                                Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yRateLimiter.calculate(yPercent) *
                                Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rotation = rotRateLimiter.calculate(rotPercent)
                                * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                var translation = new Translation2d(xSpeed, ySpeed);

                // Creates a robot-relative ChassisSpeeds object, converting from field-relative
                // speeds if
                // necessary.
                ChassisSpeeds velocity = new ChassisSpeeds(translation.getX(),
                                translation.getY(), rotation);

                velocity = ChassisSpeeds.fromFieldRelativeSpeeds(
                                velocity, getRotation().minus(driverRotationOffset));
                swerveDrive.drive(velocity, false, new Translation2d());
        }

        public void drive(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
                var xSpeed = xRateLimiter.calculate(xPercent) *
                                Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yRateLimiter.calculate(yPercent) *
                                Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotRateLimiter.calculate(rotPercent)
                                * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rot, fieldRelative, false);
        }

        public void driveAuto(double xPercent, double yPercent, double rotPercent, boolean fieldRelative) {
                var xSpeed = xPercent * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var ySpeed = yPercent * Constants.DriveConstants.MaxVelocityMetersPerSecond;
                var rot = rotPercent * Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond;

                swerveDrive.drive(new Translation2d(xSpeed, ySpeed), rot, fieldRelative, false);
        }

        public void stop() {
                SwerveModuleState[] states = swerveDrive.getStates();
                for (var state : states) {
                        state.speedMetersPerSecond = 0;
                }
                driveStates(states);
        }

        public void xpattern() {
                SwerveModuleState[] states = swerveDrive.getStates();
                for (var i = 0; i < states.length; i++) {
                        states[i].angle = DriveConstants.kinematics.getModules()[i].getAngle();
                        states[i].speedMetersPerSecond = 0;
                }
                driveStates(states);
        }

        public Rotation2d getRotation() {
                return swerveDrive.getOdometryHeading();
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
                                // seenMT = true;
                                swerveDrive.addVisionMeasurement(
                                                mt1.pose,
                                                mt1.timestampSeconds,
                                                VecBuilder.fill(2, 2, 5));
                        }
                } else if (useMegaTag2 == true) {
                        LimelightHelpers.SetRobotOrientation(limelight,
                                        swerveDrive.getPose().getRotation().getDegrees(),
                                        0, 0, 0, 0, 0);
                        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers
                                        .getBotPoseEstimate_wpiBlue_MegaTag2(limelight);
                        // heeeeeeelloooooo
                        // if our angular velocity is greater than 720 degrees per second,
                        // ignore vision updates
                        if (Math.abs(swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond)) > 720) {
                                doRejectUpdate = true;
                        }
                        if (mt2.tagCount == 0) {
                                doRejectUpdate = true;
                        }
                        if (!doRejectUpdate) {
                                // seenMT = true;
                                swerveDrive.addVisionMeasurement(
                                                mt2.pose,
                                                mt2.timestampSeconds,
                                                VecBuilder.fill(.7, .7, 9999999));

                        }
                }
        }

        @Override
        public void periodic() {
                Logger.recordOutput("Pose", swerveDrive.getPose());
                swerveDrive.updateOdometry();
                doMegatag(Constants.ReefLimelightName);
                doMegatag(Constants.UpperLimelightName);
        }

        public Pose2d getPose() {
                return swerveDrive.getPose();
        }

        public void resetOmetry(Pose2d pose) {
                swerveDrive.resetOdometry(pose);
        }

        private ChassisSpeeds getRobotVelocity() {
                return swerveDrive.getRobotVelocity();
        }

        public void driveSpeeds(ChassisSpeeds chassisSpeeds) {
                var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                                ChassisSpeeds.discretize(chassisSpeeds, .02));
                driveStates(swerveModuleStates);

        }

        private void driveStates(SwerveModuleState[] desiredStates) {
                swerveDrive.setModuleStates(desiredStates, true);
        }

        public SwerveSubsystem() {
                double maximumSpeedMps = Constants.DriveConstants.MaxVelocityMetersPerSecond;
                File swerveJsonDirectory = new File(Filesystem.getDeployDirectory(), "swerve");
                SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
                try {
                        swerveDrive = new SwerveParser(swerveJsonDirectory).createSwerveDrive(maximumSpeedMps);
                } catch (Exception e) {
                        throw new RuntimeException(e);
                }
                // Heading correction should only be used while controlling the
                // robot via angle.
                // TODO: what is this?
                swerveDrive.setHeadingCorrection(true);
                // cc breaks sim
                swerveDrive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
                swerveDrive.setAngularVelocityCompensation(true,
                                true,
                                0.1);

                RobotConfig config;
                try {
                        config = RobotConfig.fromGUISettings();
                } catch (Exception e) {
                        e.printStackTrace();
                        throw new RuntimeException(e);
                }
                // Configure AutoBuilder last
                boolean enableFeedforward = true;
                AutoBuilder.configure(
                                this::getPose, // Robot pose supplier
                                this::resetOmetry, // Method to reset odometry (will be called if your auto has a
                                                   // starting pose)
                                this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                (speedsRobotRelative, moduleFeedForwards) -> {
                                        if (enableFeedforward) {
                                                swerveDrive.drive(
                                                                speedsRobotRelative,
                                                                swerveDrive.kinematics.toSwerveModuleStates(
                                                                                speedsRobotRelative),
                                                                moduleFeedForwards.linearForces());
                                        } else {
                                                swerveDrive.setChassisSpeeds(speedsRobotRelative);
                                        }

                                        // var swerveModuleStates = DriveConstants.kinematics.toSwerveModuleStates(
                                        // ChassisSpeeds.discretize(speeds, .02));
                                        // driveStates(swerveModuleStates);
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
                PathfindingCommand.warmupCommand().schedule();

                var sysIdRoutine = new SysIdRoutine(
                                new SysIdRoutine.Config(null, null, null,
                                                (state) -> Logger.recordOutput("SysIdTestState", state.toString())),
                                new SysIdRoutine.Mechanism(
                                                (voltage) -> this.runCharacterization(voltage.in(Volts)),
                                                null, // No log consumer, since data is recorded by URCL
                                                this));

                Shuffleboard.getTab("SysId").add("Quasi Forward Swerve",
                                Commands.run(() -> this.runCharacterization(0)).withTimeout(1).andThen(
                                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward)));
                Shuffleboard.getTab("SysId").add("Quasi Backward Swerve",
                                Commands.run(() -> this.runCharacterization(0)).withTimeout(1).andThen(
                                                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse)));
                Shuffleboard.getTab("SysId").add("Dynamic Forward Swerve",
                                Commands.run(() -> this.runCharacterization(0)).withTimeout(1).andThen(
                                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)));
                Shuffleboard.getTab("SysId").add("Dynamic Backaward Swerve",
                                Commands.run(() -> this.runCharacterization(0)).withTimeout(1).andThen(
                                                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse)));

                new NTDouble(0.01, "/Tune/Swerve/TurnP").subscribe((p) -> {
                        for (var m : swerveDrive.getModules()) {
                                System.out.println("set p " + p);
                                var conf = ((SparkMaxSwerve) m.getAngleMotor()).getConfig();
                                conf.closedLoop.p(p);
                                ((SparkMaxSwerve) m.getAngleMotor()).updateConfig(conf);
                        }
                });

                new NTDouble(0.0, "/Tune/Swerve/TurnI").subscribe((i) -> {
                        for (var m : swerveDrive.getModules()) {
                                var conf = ((SparkMaxSwerve) m.getAngleMotor()).getConfig();
                                conf.closedLoop.i(i);
                                ((SparkMaxSwerve) m.getAngleMotor()).updateConfig(conf);
                        }
                });

                new NTDouble(0.0, "/Tune/Swerve/TurnD").subscribe((d) -> {
                        for (var m : swerveDrive.getModules()) {
                                var conf = ((SparkMaxSwerve) m.getAngleMotor()).getConfig();
                                conf.closedLoop.d(d);
                                ((SparkMaxSwerve) m.getAngleMotor()).updateConfig(conf);
                        }
                });
        }

        private void runCharacterization(double in) {
                // fLSwerve.runCharacterization(in);
                // fRSwerve.runCharacterization(in);
                // bLSwerve.runCharacterization(in);
                // bRSwerve.runCharacterization(in);
        }

        // WIP
        public void postTrajectory(List<Pose2d> poses) {
                if (poses.isEmpty()) {
                        return;
                }
                // Create a Trajectory from the list of poses
                List<Trajectory.State> states = new java.util.ArrayList<>();
                double totalTime = 0.0;
                double velocity = DriveConstants.MaxVelocityMetersPerSecond * 0.5; // Use 50% of max velocity

                for (int i = 0; i < poses.size(); i++) {
                        Pose2d pose = poses.get(i);

                        // Calculate time based on distance traveled
                        if (i > 0) {
                                double distance = poses.get(i - 1).getTranslation()
                                                .getDistance(pose.getTranslation());
                                totalTime += distance / velocity;
                        }

                        // Create trajectory state
                        Trajectory.State state = new Trajectory.State(
                                        totalTime,
                                        velocity,
                                        0.0, // acceleration
                                        pose,
                                        0.0 // curvature
                        );
                        states.add(state);
                }

                // Create the trajectory from states
                Trajectory trajectory = new Trajectory(states);

                // Post trajectory for visualization
                swerveDrive.postTrajectory(trajectory);
        }
}