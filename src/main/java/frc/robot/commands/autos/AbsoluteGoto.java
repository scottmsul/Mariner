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
    private ProfiledPIDController xPID;
    private ProfiledPIDController yPID;
    private ProfiledPIDController rotationPID;
    private NTDouble translationError;
    private Pose2d goal;
    private double rotationError = 0.02;

    public AbsoluteGoto(SwerveSubsystem swerveSub, Pose2d goal, NTDouble translationError) {
        addRequirements(swerveSub);
        this.swerveSub = swerveSub;
        this.translationError = translationError;

        this.goal = goal;

        xPID = new ProfiledPIDController(8, .0, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2));
        yPID = new ProfiledPIDController(8, 0, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2));
        rotationPID = new ProfiledPIDController(3.85 * .9, .8 * .7, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond / 3,
                        3 / 1.5));

        yPID.setGoal(goal.getY());
        xPID.setGoal(goal.getX());
        rotationPID.setGoal(goal.getRotation().getRadians());

        yPID.setIntegratorRange(-1, 1);
        xPID.setIntegratorRange(-1, 1);
    }

    @Override
    public void initialize() {
        yPID.reset(swerveSub.getPose().getY());
        xPID.reset(swerveSub.getPose().getX());
        rotationPID.reset(swerveSub.getPose().getRotation().getRadians());
    }

    public boolean isAligned() {
        if (goal.minus(swerveSub.getPose()).getTranslation().getNorm() < translationError.get()
                && goal.getRotation().minus(swerveSub.getPose().getRotation()).getRadians() < rotationError) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void execute() {
        var nt = NetworkTableInstance.getDefault();

        double ySpeed = yPID.calculate(swerveSub.getPose().getY());
        ySpeed = MathUtil.clamp(ySpeed, -DriveConstants.MaxVelocityMetersPerSecond / 3.5,
                DriveConstants.MaxVelocityMetersPerSecond / 3.5);

        double xSpeed = -xPID.calculate(swerveSub.getPose().getX());
        xSpeed = MathUtil.clamp(xSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 3.5,
                DriveConstants.MaxVelocityMetersPerSecond / 3.5);

        double rot = rotationPID.calculate(swerveSub.getPose().getRotation().getRadians());
        rot = MathUtil.clamp(rot, -DriveConstants.MaxAngularVelocityRadiansPerSecond
                / 3.5,
                DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5);

        nt.getEntry("/Tune/AutoAlignAbs/Y Goal").setDouble(yPID.getGoal().position);
        nt.getEntry("/Tune/AutoAlignAbs/Y Setpoint").setDouble(yPID.getSetpoint().position);
        nt.getEntry("/Tune/AutoAlignAbs/Y CurrentSwerve").setDouble(swerveSub.getPose().getY());
        nt.getEntry("/Tune/AutoAlignAbs/Y out").setDouble(ySpeed);
        nt.getEntry("/Tune/AutoAlignAbs/X Goal").setDouble(xPID.getGoal().position);
        nt.getEntry("/Tune/AutoAlignAbs/X Setpoint").setDouble(xPID.getSetpoint().position);
        nt.getEntry("/Tune/AutoAlignAbs/X CurrentSwerve").setDouble(swerveSub.getPose().getX());
        nt.getEntry("/Tune/AutoAlignAbs/X out").setDouble(xSpeed);
        nt.getEntry("/Tune/AutoAlignAbs/R Goal").setDouble(goal.getRotation().getRadians());
        nt.getEntry("/Tune/AutoAlignAbs/R Setpoint").setDouble(rotationPID.getSetpoint().position);
        nt.getEntry("/Tune/AutoAlignAbs/R CurrentSwerve").setDouble(swerveSub.getPose().getRotation().getRadians());

        swerveSub.driveAuto(ySpeed / DriveConstants.MaxVelocityMetersPerSecond,
                xSpeed / DriveConstants.MaxVelocityMetersPerSecond,
                rot / DriveConstants.MaxAngularVelocityRadiansPerSecond, true);
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
