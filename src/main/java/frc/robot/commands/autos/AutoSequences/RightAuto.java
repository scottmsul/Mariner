package frc.robot.commands.autos.AutoSequences;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.CoralStationSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RightAuto extends SequentialCommandGroup {
        final SwerveSubsystem swerveSubsystem;
        final CoralArm coralArm;
        final AlgaeArm algaeArm;
        final Elevator elevator;

        public RightAuto(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerve) {
                this.coralArm = coralArm;
                this.algaeArm = algaeArm;
                this.elevator = elevator;
                this.swerveSubsystem = swerve;

                addCommands(
                                AutoBuilder.pathfindToPose(Constants.WaypointConstants.ReefNE,
                                                Constants.AutoConstants.constantConstraints),
                                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerve,
                                                Constants.SetpointConstants.StrafeOffsets.rightL4),
                                AutoBuilder.pathfindToPose(Constants.WaypointConstants.CoralStationRight,
                                                Constants.AutoConstants.constantConstraints),
                                new CoralStationSequence(coralArm, algaeArm, elevator, swerve),
                                AutoBuilder.pathfindToPose(Constants.WaypointConstants.ReefSE,
                                                Constants.AutoConstants.constantConstraints),
                                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerve,
                                                Constants.SetpointConstants.StrafeOffsets.leftL4),
                                AutoBuilder.pathfindToPose(Constants.WaypointConstants.CoralStationRight,
                                                Constants.AutoConstants.constantConstraints),
                                new CoralStationSequence(coralArm, algaeArm, elevator, swerve));

        }

}
