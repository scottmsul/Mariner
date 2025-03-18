package frc.robot.commands.autos;

import java.util.Optional;
import java.util.stream.Stream;

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
    // private static double rot;
    // private static double distanceSpeed;

    private boolean tune;
    // static double getZontal() {
    // return (LimelightHelpers.getTX("limelight-back") / 27);
    // // return (x.getDouble(160)/160)-1;
    // // horizontal offset
    // }

    static final Optional<Pose3d> getTargetPose() {
        if (LimelightHelpers.getTV(Constants.ReefLimelightName)) {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.ReefLimelightName));
        } else {
            return Optional.empty();
        }

        // Streamllresults.targets_Fiducials[0].getTargetPose_RobotSpace();
        // return (x.getDouble(160)/160)-1;
        // whatever the distance is
        // returns the specific distance value we want so we can pid it???
        // why is everything so
    }

    public static boolean speakerAimReady() {
        return LimelightHelpers.getTV(Constants.ReefLimelightName);
    }

    public AutoAlignReef(SwerveSubsystem swerveSub, NTDouble strafeGoal, NTDouble distanceGoal, NTDouble rotationGoal, NTDouble strafeError, NTDouble distanceError) {
            this(swerveSub, strafeGoal, distanceGoal, rotationGoal, strafeError, distanceError, false);
    }

    public AutoAlignReef(SwerveSubsystem swerveSub, NTDouble strafeGoal, NTDouble distanceGoal, NTDouble rotationGoal, NTDouble strafeError, NTDouble distanceError, boolean tune) {
        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.strafeGoal = strafeGoal;
        this.distanceGoal = distanceGoal;
        this.rotationGoal = rotationGoal;
        this.strafeError = strafeError;
        this.distanceError = distanceError;
        this.tune = tune;

        strafePID = new ProfiledPIDController(3.85 * .9, .8 * .7, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));
        distancePID = new ProfiledPIDController(3.85 * .9, .8 * .7, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));
        rotationPID = new ProfiledPIDController(3.85 * .9, .8 * .7, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond / 3,
                        3 / 1.5));

        
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
                && (Math.abs(rotationGoal.get()- target.getRotation().getZ()) < 0.02)
        // && (Math.abs(target.getRotation().getAngle()) < 0.5)
        ) {
            return true;
        } else {
            return false;
        }
    }

    public boolean lowSpeed(){
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

        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL Distance").setDouble(target.getZ());
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Out").setDouble(distanceSpeed);
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL Strafe").setDouble(target.getX());
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Strafe Out").setDouble(strafeSpeed);
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL rotation yaw").setDouble(target.getRotation().getZ());
        // nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID rotation out").setDouble(rot);


        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL Distance").setDouble(target.getZ());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Out").setDouble(distanceSpeed);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Setpoint").setDouble(distancePID.getSetpoint().position);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Goal").setDouble(distancePID.getGoal().position);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Distance Error").setDouble(distancePID.getSetpoint().position - target.getZ());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL Strafe").setDouble(target.getX());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID Strafe Out").setDouble(strafeSpeed);
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/LL rotation yaw").setDouble(target.getRotation().getZ());
        nt.getEntry("/Shuffleboard/Tune/AutoAlignTags/PID rotation out").setDouble(rot);
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
