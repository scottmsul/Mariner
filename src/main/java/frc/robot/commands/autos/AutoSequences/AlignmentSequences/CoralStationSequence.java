package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.LLLeds;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignUpper;
import frc.robot.commands.autos.CoralAutos.AutoCoralIntake;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class CoralStationSequence extends SequentialCommandGroup {

    public CoralStationSequence(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSubsystem) {
                var config = new ConfigSystem(Constants.SetpointConstants.Options.coralStation, coralArm, elevator, algaeArm);
                var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator, algaeArm);
                var configureAlign = new AutoAlignUpper(swerveSubsystem, -0.1,0.7, 0, 0.04, 0.04);
                var intakeAlign = new AutoAlignUpper(swerveSubsystem, -0.1, 0.6, 0, 0.02, 0.02);
                var intakeCoral = new AutoCoralIntake(coralArm);
        addCommands(
            new ParallelCommandGroup(
                configureAlign.until(configureAlign::aligned),
                config
            ),
            intakeAlign.until(intakeAlign::aligned),
            Commands.parallel(
                intakeCoral.until(coralArm::hasCoral),
                LLLeds.shortBlink(Constants.UpperLimelightName)
            ),
            stow.until(stow::isConfigured)
        );
    }
}
