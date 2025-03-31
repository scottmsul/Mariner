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
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RightAlignmentSequence extends SequentialCommandGroup {

        public RightAlignmentSequence(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator,
                        SwerveSubsystem swerveSubsystem, ConfigOption configOption) {
                var config = new ConfigSystem(configOption, coralArm, elevator, algaeArm);
                var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator,
                                algaeArm);
                var configureAlign = new AutoAlignReef(swerveSubsystem,
                                Constants.SetpointConstants.StrafeOffsets.rightReef,
                                Constants.SetpointConstants.DistanceOffsets.reefCoralConfigure, NTD.of(0), NTD.of(0.1),
                                NTD.of(0.1));
                var secondConfigureAlign = new AutoAlignReef(swerveSubsystem, NTD.of(0),
                                Constants.SetpointConstants.DistanceOffsets.reefCoralConfigure, NTD.of(0), NTD.of(0.1),
                                NTD.of(0.1));
                var scoreAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.rightReef,
                                Constants.SetpointConstants.DistanceOffsets.rightReefScore, NTD.of(0), NTD.of(0.02),
                                NTD.of(0.02));
                var scoreCoral = new AutoCoralScore(coralArm);
                addCommands(
                                new ParallelCommandGroup(
                                                configureAlign,
                                                config),
                                scoreAlign,
                                scoreCoral.withTimeout(0.5),
                                new ParallelRaceGroup(
                                                secondConfigureAlign,
                                                stow));
        }
}
