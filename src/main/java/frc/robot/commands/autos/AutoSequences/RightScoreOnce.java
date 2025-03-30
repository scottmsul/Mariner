package frc.robot.commands.autos.AutoSequences;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.GoTo;
import frc.robot.commands.autos.AutoDrive;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.CoralStationSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RightScoreOnce extends SequentialCommandGroup {
    final CoralArm coralArm;
    final Elevator elevator;
    final SwerveSubsystem swerveSub;
    final AlgaeArm algaeArm;

    public RightScoreOnce(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerveSubsystem) {
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.swerveSub = swerveSubsystem;
        this.algaeArm = algaeArm;

        addCommands(
                new AutoDrive(swerveSubsystem, 1, 0.5),
                GoTo.reefNE(),
                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerveSubsystem,
                        Constants.SetpointConstants.StrafeOffsets.leftL4),
                new WaitCommand(5),
                GoTo.coralStationRight(),
                new CoralStationSequence(coralArm, algaeArm, elevator, swerveSubsystem));
    }
}
