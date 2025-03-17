package frc.robot.commands.autos;

import edu.wpi.first.math.MathUtil;
import frc.robot.NTDouble;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;

public class AbsoluteGoto extends Command {
    private SwerveSubsystem swerveSub;
    private ProfiledPIDController strafePID;
    private ProfiledPIDController distancePID;
    private ProfiledPIDController rotationPID;
    private boolean lowSpeed;
    private NTDouble distanceError;
    private Pose2d goal;
    private double rotationError = 0.02;

    public AbsoluteGoto(SwerveSubsystem swerveSub, Pose2d goal, NTDouble strafeError, NTDouble distanceError) {
        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.distanceError = distanceError;

        this.goal = goal;

        strafePID = new ProfiledPIDController(3.3 * .6, .8 * .5, .8 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));
        distancePID = new ProfiledPIDController(3.3 * .6, .8 * .5, .8 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 3 / 1.5));
        rotationPID = new ProfiledPIDController(3.3 * .6, .8 * .5, .8 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond / 3,
                        3 / 1.5));

        distancePID.setIntegratorRange(-15, 15);
        strafePID.setIntegratorRange(-15, 15);
    }

    @Override
    public void initialize() {
    }

    public boolean isAligned() {
        if (goal.minus(swerveSub.getPose()).getTranslation().getNorm() < distanceError.get()
                && goal.getRotation().minus(swerveSub.getPose().getRotation()).getRadians() < rotationError) {
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
        var nt = NetworkTableInstance.getDefault();

        double distanceSpeed = -distancePID.calculate(swerveSub.getPose().getX());
        distanceSpeed = MathUtil.clamp(distanceSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 3.5,
                DriveConstants.MaxVelocityMetersPerSecond / 3.5);

        double strafeSpeed = strafePID.calculate(swerveSub.getPose().getY());
        strafeSpeed = MathUtil.clamp(strafeSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 5,
                DriveConstants.MaxVelocityMetersPerSecond / 5);

        double rot = rotationPID.calculate(swerveSub.getPose().getRotation().getRadians());
        rot = MathUtil.clamp(rot, -DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5,
                DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5);

        nt.getEntry("/Tune/AutoAlignAbs/X Goal").setDouble(distancePID.getGoal().position);
        nt.getEntry("/Tune/AutoAlignAbs/X Setpoint").setDouble(distancePID.getSetpoint().position);
        nt.getEntry("/Tune/AutoAlignAbs/X CurrentSwerve").setDouble(swerveSub.getPose().getX());
        nt.getEntry("/Tune/AutoAlignAbs/Y Goal").setDouble(strafePID.getGoal().position);
        nt.getEntry("/Tune/AutoAlignAbs/Y Setpoint").setDouble(strafePID.getSetpoint().position);
        nt.getEntry("/Tune/AutoAlignAbs/Y CurrentSwerve").setDouble(swerveSub.getPose().getY());
        nt.getEntry("/Tune/AutoAlignAbs/R Goal").setDouble(goal.getRotation().getRadians());
        nt.getEntry("/Tune/AutoAlignAbs/R Setpoint").setDouble(rotationPID.getSetpoint().position);
        nt.getEntry("/Tune/AutoAlignAbs/R CurrentSwerve").setDouble(swerveSub.getPose().getRotation().getRadians());

        swerveSub.drive(distanceSpeed / DriveConstants.MaxVelocityMetersPerSecond,
                strafeSpeed / DriveConstants.MaxVelocityMetersPerSecond,
                rot / DriveConstants.MaxAngularVelocityRadiansPerSecond, false);
    }

    @Override
    public boolean isFinished() {
        return isAligned();
    }

    @Override
    public void end(boolean interrupted) {
        swerveSub.stop();
    }
}
