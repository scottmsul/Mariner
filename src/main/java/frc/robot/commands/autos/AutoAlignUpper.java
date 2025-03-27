package frc.robot.commands.autos;

import java.util.Optional;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers;
import frc.robot.NTDouble;
import frc.robot.NTDouble.NTD;
import frc.robot.Photon;
import frc.robot.subsystems.SwerveSubsystem;
import frc.souffle.Souffle;

public class AutoAlignUpper extends Command {

    private SwerveSubsystem swerveSub;
    private ProfiledPIDController strafePID;
    private ProfiledPIDController distancePID;
    private ProfiledPIDController rotationPID;
    private NTDouble strafeGoal = NTD.of(0.0);
    private NTDouble distanceGoal = NTD.of(0.5);
    private NTDouble rotationGoal;
    private boolean lowSpeed;
    private NTDouble strafeError;
    private NTDouble distanceError;
    // private static double rot;
    // private static double distanceSpeed;
    private boolean tune;
    private boolean firstRun = false;

    // static double getZontal() {
    // return (LimelightHelpers.getTX("limelight-back") / 27);
    // // return (x.getDouble(160)/160)-1;
    // // horizontal offset
    // }
    static final Optional<Pose3d> getTargetPoseLimelight() {
        if (LimelightHelpers.getTV(Constants.UpperLimelightName)) {
            var pose = LimelightHelpers.getTargetPose3d_RobotSpace(Constants.UpperLimelightName);
            return Optional.of(new Pose3d(pose.getZ(), pose.getX(), pose.getY(), pose.getRotation()));
        } else {
            return Optional.empty();
        }
    }

    public static final Optional<Pose3d> getTargetPosePhoton() {
        var results = Photon.getInstance().getLastResult();
        if (results.hasTargets()) {
            var cameraToTarget = results.getBestTarget().getBestCameraToTarget();
            var transform = Constants.robotToCamera.plus(cameraToTarget);
            return Optional.of(new Pose3d(transform.getTranslation(), transform.getRotation()));
        } else {
            return Optional.empty();
        }
    }

    public static boolean speakerAimReady() {
        return LimelightHelpers.getTV(Constants.UpperLimelightName);
    }

    public AutoAlignUpper(SwerveSubsystem swerveSub, NTDouble strafeGoal, NTDouble distanceGoal, NTDouble rotationGoal,
            NTDouble strafeError, NTDouble distanceError) {
        this(swerveSub, strafeGoal, distanceGoal, rotationGoal, strafeError, distanceError, false);
    }

    public AutoAlignUpper(SwerveSubsystem swerveSub, NTDouble strafeGoal, NTDouble distanceGoal, NTDouble rotationGoal,
            NTDouble strafeError, NTDouble distanceError, boolean tune) {
        addRequirements(swerveSub);
        this.tune = tune;
        this.swerveSub = swerveSub;
        this.strafeGoal = strafeGoal;
        this.distanceGoal = distanceGoal;
        this.rotationGoal = rotationGoal;
        this.strafeError = strafeError;
        this.distanceError = distanceError;

        strafePID = new ProfiledPIDController(3.65 * .9, .8 * .7, .8 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3.0, 3.0 / 1.5));
        distancePID = new ProfiledPIDController(3.65 * .9, .8 * .7, .8 * .1,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3.0, 1.4));
        rotationPID = new ProfiledPIDController(3.65 * .9, .8 * .7, .8 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.0,
                        3.0 / 1.5));

        distanceGoal.subscribe(goal -> distancePID.setGoal(goal));
        strafeGoal.subscribe(goal -> strafePID.setGoal(goal));
        rotationGoal.subscribe(goal -> rotationPID.setGoal(goal));
        distancePID.setIntegratorRange(-15, 15);
        strafePID.setIntegratorRange(-15, 15);

        if (tune) {
            Shuffleboard.getTab("Tune").add(distancePID);
            Shuffleboard.getTab("Tune").add(strafePID);
            Shuffleboard.getTab("Tune").add(rotationPID);
        }
    }

    @Override
    public void initialize() {
        firstRun = true;
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-back", new
        // int[]{4,7});
        // distancePID.reset(distanceGoal.get());
        // strafePID.reset(strafeGoal.get());
        // rotationPID.reset(rotationGoal.get());
    }

    public boolean isAligned() {
        // if (!LimelightHelpers.getTV("limelight-back")) {
        // return false;
        // }
        var target_opt = getTargetPosePhoton();
        if (target_opt.isEmpty()) {
            return false;
        }
        var target = target_opt.get();
        if ((Math.abs(distanceGoal.get() - target.getX()) < distanceError.get())
                && (Math.abs(strafeGoal.get() - target.getY()) < strafeError.get())
                && (Math.abs(rotationGoal.get() - target.getRotation().getX()) < 0.02)
        // && (Math.abs(target.getRotation().getAngle()) < 0.5)
        ) {
            return true;
        } else {
            return false;
        }
    }

    public boolean lowSpeed() {
        return lowSpeed;
    }

    @Override
    public void execute() {
        // if (LimelightHelpers.getTV("limelight-back")) {
        // var id = LimelightHelpers.getFiducialID("limelight-back");
        var target_opt = getTargetPosePhoton();
        if (target_opt.isEmpty()) {
            swerveSub.stop();
            return;
        }
        var target = target_opt.get();

        if (firstRun) {
            firstRun = false;

            distancePID.reset(target.getX());
            strafePID.reset(target.getY());
            rotationPID.reset(target.getRotation().getZ());
        }

        var nt = NetworkTableInstance.getDefault();

        double distanceSpeed = -distancePID.calculate(target.getX());
        distanceSpeed = MathUtil.clamp(distanceSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 3.5,
                DriveConstants.MaxVelocityMetersPerSecond / 3.5);

        double strafeSpeed = -strafePID.calculate(target.getY());
        strafeSpeed = MathUtil.clamp(strafeSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 5,
                DriveConstants.MaxVelocityMetersPerSecond / 5);

        double rot = rotationPID.calculate(target.getRotation().getZ());
        rot = MathUtil.clamp(rot, -DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5,
                DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5);
        rot = 0;

        Souffle.log("Poses/PVTuneTarget", target);

        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL Distance").setDouble(target.getX());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Out").setDouble(distanceSpeed);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Setpoint")
                .setDouble(distancePID.getSetpoint().position);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Goal").setDouble(distancePID.getGoal().position);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Error")
                .setDouble(distancePID.getSetpoint().position - target.getZ());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL Strafe").setDouble(target.getY());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Strafe Setpoint").setDouble(strafePID.getSetpoint().position);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Strafe Goal").setDouble(strafePID.getGoal().position);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Strafe Out").setDouble(strafeSpeed);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL rotation yaw").setDouble(target.getRotation().getZ());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID rotation out").setDouble(rot);

        swerveSub.drive(distanceSpeed / DriveConstants.MaxVelocityMetersPerSecond,
                strafeSpeed / DriveConstants.MaxVelocityMetersPerSecond,
                rot / DriveConstants.MaxAngularVelocityRadiansPerSecond, false);
        // wtf why is LimelightHelpers wrong
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-back", new int[]{});
        swerveSub.stop();
    }
}
