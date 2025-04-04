
package frc.robot.commands.autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoTo;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class CenterScoreOnceRight extends SequentialCommandGroup {
    final CoralArm coralArm;
    final Elevator elevator;
    final AlgaeArm algaeArm;
    final SwerveSubsystem swerveSubsystem;

    public CenterScoreOnceRight(CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm,
            SwerveSubsystem swerveSubsystem) {
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.algaeArm = algaeArm;
        this.swerveSubsystem = swerveSubsystem;

        addCommands(
                GoTo.reefN(),
                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerveSubsystem,
                        Constants.SetpointConstants.StrafeOffsets.rightL4,
                        Constants.SetpointConstants.DistanceOffsets.L4Right));
    }
}
