package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import java.lang.annotation.ElementType;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.NTDouble.NTD;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignReef;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class AbortAbortReef extends SequentialCommandGroup {
    public AbortAbortReef(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSub) {
        var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator, algaeArm);
        var safetyAlign = new AutoAlignReef(swerveSub, NTD.of(0.0),
                Constants.SetpointConstants.DistanceOffsets.reefAlgaeStow, NTD.of(0.0), NTD.of(0.05), NTD.of(0.05));
        addCommands(
                Commands.print("ABORT ABORT ABORT Started"),
                new ParallelCommandGroup(
                        stow,
                        safetyAlign),
                Commands.print("ABORT ABORT ABORT done"));
    }
}
