package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.NTDouble.NTD;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignReef;
import frc.robot.commands.autos.AutoAlignUpper;
import frc.robot.commands.autos.AlgaeAutos.AutoAlgaeIntake;
import frc.robot.commands.autos.AlgaeAutos.AutoAlgaeScore;
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class ProcessorAlignmentSequence extends SequentialCommandGroup{

    public ProcessorAlignmentSequence(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSubsystem ) {
                var config = new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevator, algaeArm);
                var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator, algaeArm);
                var scoreAlign = new AutoAlignUpper(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.processor ,Constants.SetpointConstants.DistanceOffsets.processorScore, NTD.of(0), NTD.of(0.08), NTD.of(0.08));
                var configAlign = new AutoAlignUpper(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.processor ,Constants.SetpointConstants.DistanceOffsets.processorInitial, NTD.of(0), NTD.of(0.08), NTD.of(0.06));
                var configAlign2 = new AutoAlignUpper(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.processor ,Constants.SetpointConstants.DistanceOffsets.processorInitial, NTD.of(0), NTD.of(0.08), NTD.of(0.06));
                var scoreAlgae = new AutoAlgaeScore(algaeArm);
        addCommands(
            new ParallelCommandGroup(
                configAlign,
                config
            ),
            scoreAlign,
            scoreAlgae.withTimeout(1),
            configAlign2,
            stow
        );
    }
}
