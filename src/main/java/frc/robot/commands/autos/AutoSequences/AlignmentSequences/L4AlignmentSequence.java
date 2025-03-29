package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.NTDouble;
import frc.robot.NTDouble.NTD;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignReef;
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class L4AlignmentSequence extends SequentialCommandGroup {
        // give it a strafeoffset for each side in the parameters, when constructing
        // just use the leftreef or rightReef setpoint strafeoffset constants
        // this is so we dont need two identical classes
        public L4AlignmentSequence(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator,
                        SwerveSubsystem swerveSubsystem,
                        NTDouble strafeOffset) {
                var config = new ConfigSystem(Constants.SetpointConstants.Options.l4, coralArm, elevator, algaeArm);
                var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator,
                                algaeArm);
                var configureAlign = new AutoAlignReef(swerveSubsystem, strafeOffset,
                                Constants.SetpointConstants.DistanceOffsets.reefCoralConfigure, NTD.of(0), NTD.of(0.1),
                                NTD.of(0.07));
                var secondConfigureAlign = new AutoAlignReef(swerveSubsystem, NTD.of(0),
                                Constants.SetpointConstants.DistanceOffsets.reefCoralConfigure, NTD.of(0), NTD.of(0.04),
                                NTD.of(0.04), true);
                var scoreAlign = new AutoAlignReef(swerveSubsystem, strafeOffset,
                                Constants.SetpointConstants.DistanceOffsets.leftReefScore, NTD.of(0), NTD.of(0.02),
                                NTD.of(0.02));
                var scoreCoral = new AutoCoralScore(coralArm);

                addCommands(
                                new ParallelCommandGroup(
                                                configureAlign,
                                                config),
                                scoreAlign,
                                scoreCoral.withTimeout(0.5),
                                new ParallelCommandGroup(
                                                secondConfigureAlign,
                                                stow));
        }
}
