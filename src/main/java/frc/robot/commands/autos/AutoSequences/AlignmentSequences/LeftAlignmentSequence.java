package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignTags;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class LeftAlignmentSequence extends SequentialCommandGroup {

    public LeftAlignmentSequence(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSubsystem, ConfigOption configOption) {
                var config = new ConfigSystem(configOption, coralArm, elevator, algaeArm);
                var configureAlign = new AutoAlignTags(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.leftReef,Constants.SetpointConstants.DistanceOffsets.reefConfigure, 0);
                var scoreAlign = new AutoAlignTags(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.leftReef, Constants.SetpointConstants.DistanceOffsets.leftReefScore, 0);
        addCommands(
            new ParallelCommandGroup(
                configureAlign,
                config.until(config::isConfigured)
            ),
            scoreAlign.until(scoreAlign::aligned)
        );
    }
}
