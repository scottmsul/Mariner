package frc.robot.commands.autos.AutoSequences;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.GoTo;
import frc.robot.commands.autos.AutoDrive;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class LeftScoreOnce extends SequentialCommandGroup {
    final CoralArm coralArm;
    final Elevator elevator;
    final AlgaeArm algaeArm;
    final SwerveSubsystem swerveSubsystem;

    public LeftScoreOnce(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSubsystem,
            Optional<Alliance> alliance) {
        this.coralArm = coralArm;
        this.algaeArm = algaeArm;
        this.elevator = elevator;
        this.swerveSubsystem = swerveSubsystem;

        addCommands(
                new AutoDrive(swerveSubsystem, 1, 0.5),
                new GoTo(alliance).reefNW(),
                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerveSubsystem, null));

    }
}
