package frc.robot.commands.autos;

import java.util.Optional;
import java.util.stream.Stream;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.LimelightHelpers;
import frc.robot.NTDouble;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class AutoAlignReef extends Command {

    private SwerveSubsystem swerveSub;
    private ProfiledPIDController strafePID;
    private ProfiledPIDController distancePID;
    private ProfiledPIDController rotationPID;
    private NTDouble strafeGoal;
    private NTDouble distanceGoal;
    private NTDouble rotationGoal;
    private boolean lowSpeed;
    private NTDouble strafeError;
    private NTDouble distanceError;
    private String llName;
    // private static double rot;
    // private static double distanceSpeed;

    // static double getZontal() {
    // return (LimelightHelpers.getTX("limelight-back") / 27);
    // // return (x.getDouble(160)/160)-1;
    // // horizontal offset
    // }

    // Streamllresults.targets_Fiducials[0].getTargetPose_RobotSpace();
    // return (x.getDouble(160)/160)-1;
    // whatever the distance is
    // returns the specific distance value we want so we can pid it???
    // why is everything so

    public static boolean speakerAimReady() {
        return LimelightHelpers.getTV(Constants.ReefLimelightName);
    }

    public AutoAlignReef(SwerveSubsystem swerveSub, NTDouble strafeGoal, NTDouble distanceGoal, NTDouble rotationGoal,
            NTDouble strafeError, NTDouble distanceError) {
        this(swerveSub, strafeGoal, distanceGoal, rotationGoal, strafeError, distanceError, false);
    }

    public AutoAlignReef(SwerveSubsystem swerveSub, NTDouble strafeGoal, NTDouble distanceGoal, NTDouble rotationGoal,
            NTDouble strafeError, NTDouble distanceError, boolean slow) {
        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.strafeGoal = strafeGoal;
        this.distanceGoal = distanceGoal;
        this.rotationGoal = rotationGoal;
        this.strafeError = strafeError;
        this.distanceError = distanceError;

        double maxAccel = 3.0 / 1.5;
        if (slow) {
            maxAccel *= 0.2;
        }
        strafePID = new ProfiledPIDController(4.3 * .9, 0, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, maxAccel));
        distancePID = new ProfiledPIDController(4.3 * .9, 0, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, maxAccel));
        rotationPID = new ProfiledPIDController(3.95 * .9, 0, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond / 3,
                        maxAccel));

        distanceGoal.subscribe(goal -> distancePID.setGoal(goal));
        strafeGoal.subscribe(goal -> strafePID.setGoal(goal));
        rotationGoal.subscribe(goal -> rotationPID.setGoal(goal));
        distancePID.setIntegratorRange(-15, 15);
        strafePID.setIntegratorRange(-15, 15);

        // if (tune) {
        // Shuffleboard.getTab("Tune").add(distancePID);
        // Shuffleboard.getTab("Tune").add(strafePID);
        // Shuffleboard.getTab("Tune").add(rotationPID);
        // }
    }

    final Optional<Pose3d> getTargetPose() {
        if (LimelightHelpers.getTV(Constants.ReefLimelightName)) {
            System.out.println("using " + Constants.ReefLimelightName);
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.ReefLimelightName));
        } else if (LimelightHelpers.getTV(Constants.LeftReefLimelightName)) {
            System.out.println("using left limelight");
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.LeftReefLimelightName));
        } else {
            System.out.println("no tags detected");
            return Optional.empty();
        }
    }

    @Override
    public void initialize() {
        // LimelightHelpers.SetFiducialIDFiltersOverride("limelight-back", new
        // int[]{4,7});
        distancePID.reset(distanceGoal.get());
        strafePID.reset(strafeGoal.get());
        rotationPID.reset(rotationGoal.get());
    }

    public boolean isAligned() {
        // if (!LimelightHelpers.getTV("limelight-back")) {
        // return false;
        // }
        var target_opt = getTargetPose();
        if (target_opt.isEmpty()) {
            return false;
        }
        var target = target_opt.get();
        if ((Math.abs(distanceGoal.get() - target.getZ()) < distanceError.get())
                && (Math.abs(strafeGoal.get() - target.getX()) < strafeError.get())
                && (Math.abs(rotationGoal.get() - target.getRotation().getZ()) < 0.02)
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
        var target_opt = getTargetPose();
        if (target_opt.isEmpty()) {
            swerveSub.stop();
            return;
        }
        var target = target_opt.get();
        var nt = NetworkTableInstance.getDefault();

        double distanceSpeed = -distancePID.calculate(target.getZ());
        distanceSpeed = MathUtil.clamp(distanceSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 3.5,
                DriveConstants.MaxVelocityMetersPerSecond / 3.5);

        double strafeSpeed = strafePID.calculate(target.getX());
        strafeSpeed = MathUtil.clamp(strafeSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 5,
                DriveConstants.MaxVelocityMetersPerSecond / 5);

        double rot = rotationPID.calculate(target.getRotation().getZ());
        rot = MathUtil.clamp(rot, -DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5,
                DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5);

        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL
        // Distance").setDouble(target.getZ());
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance
        // Out").setDouble(distanceSpeed);
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL
        // Strafe").setDouble(target.getX());
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Strafe
        // Out").setDouble(strafeSpeed);
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL rotation
        // yaw").setDouble(target.getRotation().getZ());
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID rotation
        // out").setDouble(rot);

        DogLog.log("/Shuffleboard/Tune/AutoAlignTags/ReefAlign/LL Distance", target.getX());
        DogLog.log("AutoAlignTags/ReefAlign/PID Distance Out", distanceSpeed);
        DogLog.log("AutoAlignTags/ReefAlign/PID Distance Setpoint",
                distancePID.getSetpoint().position);
        DogLog.log("AutoAlignTags/ReefAlign/PID Distance Goal", distancePID.getGoal().position);
        DogLog.log("AutoAlignTags/ReefAlign/LL Strafe", target.getY());
        DogLog.log("AutoAlignTags/ReefAlign/PID Strafe Setpoint", strafePID.getSetpoint().position);
        DogLog.log("AutoAlignTags/ReefAlign/PID Strafe Goal", strafePID.getGoal().position);
        DogLog.log("AutoAlignTags/ReefAlign/PID Strafe Out", strafeSpeed);
        DogLog.log("AutoAlignTags/ReefAlign/LL rotation yaw", target.getRotation().getZ());
        DogLog.log("AutoAlignTags/ReefAlign/PID rotation out", rot);

        DogLog.log("AutoAlignTags/StrafeError", strafeGoal.get() - target.getY());
        DogLog.log("AutoAlignTags/DistanceError", distanceGoal.get() - target.getX());
        var rotationDelta = new Rotation2d(rotationGoal.get()).minus(target.getRotation().toRotation2d());
        DogLog.log("AutoAlignTags/RotationError", rotationDelta);
        // how do i set a different goal for the distance

        // System.out.println(getStance());

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
