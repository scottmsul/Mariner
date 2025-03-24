package frc.robot.commands.autos.AutoSequences;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoTo;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.CoralStationSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.commands.autos.CoralAutos.AutoCoralIntake;
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class LeftAuto extends SequentialCommandGroup {
        final SwerveSubsystem swerveSubsystem;
        final AlgaeArm algaeArm;
        final CoralArm coralArm;
        final Elevator elevator;

        public LeftAuto(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerve) {
                this.coralArm = coralArm;
                this.algaeArm = algaeArm;
                this.elevator = elevator;
                this.swerveSubsystem = swerve;

                addCommands(
                                new GoTo().reefNW(),
                                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerve,
                                                Constants.SetpointConstants.StrafeOffsets.leftL4),
                                new GoTo().coralStationLeft(),
                                new CoralStationSequence(coralArm, algaeArm, elevator, swerve),
                                new GoTo().reefSW(),
                                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerve,
                                                Constants.SetpointConstants.StrafeOffsets.leftL4),
                                new GoTo().coralStationLeft(),
                                new CoralStationSequence(coralArm, algaeArm, elevator, swerve));
        }
}
