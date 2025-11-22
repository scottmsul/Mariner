package frc.robot.commands.autos;

import org.littletonrobotics.junction.Logger;

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
import edu.wpi.first.math.kinematics.ChassisSpeeds;

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
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2.5));
        yPID = new ProfiledPIDController(8, 0, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxVelocityMetersPerSecond / 3, 2.5));
        rotationPID = new ProfiledPIDController(3.85 * .9, .8 * .7, .85 * .125,
                new TrapezoidProfile.Constraints(Constants.DriveConstants.MaxAngularVelocityRadiansPerSecond / 2,
                        (Math.PI * 2) / 2));

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

        Logger.recordOutput("/Debug/AutoAlignAbs/Goal", goal);
    }

    // TODO: Avg speed
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

        double xSpeed = xPID.calculate(swerveSub.getPose().getX());
        xSpeed = MathUtil.clamp(xSpeed, -DriveConstants.MaxVelocityMetersPerSecond / 3.5,
                DriveConstants.MaxVelocityMetersPerSecond / 3.5);

        double rot = rotationPID.calculate(swerveSub.getPose().getRotation().getRadians());
        rot = MathUtil.clamp(rot, -DriveConstants.MaxAngularVelocityRadiansPerSecond
                / 3.5,
                DriveConstants.MaxAngularVelocityRadiansPerSecond / 3.5);

        // Logger.recordOutput("/Debug/AutoAlignAbs/Y Goal", yPID.getGoal().position);
        // Logger.recordOutput("/Debug/AutoAlignAbs/Y Setpoint",
        // yPID.getSetpoint().position);
        // Logger.recordOutput("/Debug/AutoAlignAbs/Y CurrentSwerve",
        // swerveSub.getPose().getY());
        // Logger.recordOutput("/Debug/AutoAlignAbs/Y out", ySpeed);
        // Logger.recordOutput("/Debug/AutoAlignAbs/X Goal", xPID.getGoal().position);
        // Logger.recordOutput("/Debug/AutoAlignAbs/X Setpoint",
        // xPID.getSetpoint().position);
        // Logger.recordOutput("/Debug/AutoAlignAbs/X CurrentSwerve",
        // swerveSub.getPose().getX());
        // Logger.recordOutput("/Debug/AutoAlignAbs/X out", xSpeed);
        // Logger.recordOutput("/Debug/AutoAlignAbs/R Goal",
        // goal.getRotation().getRadians());
        // Logger.recordOutput("/Debug/AutoAlignAbs/R Setpoint",
        // rotationPID.getSetpoint().position);
        // Logger.recordOutput("/Debug/AutoAlignAbs/R CurrentSwerve",
        // swerveSub.getPose().getRotation().getRadians());

        var chas = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, swerveSub.getPose().getRotation());
        Logger.recordOutput("/Debug/AutoAlignAbs/Speeds", chas);
        swerveSub.driveSpeeds(chas);
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
