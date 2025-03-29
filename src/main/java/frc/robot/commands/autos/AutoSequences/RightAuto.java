package frc.robot.commands.autos.AutoSequences;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.GoTo;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoDrive;
import frc.robot.commands.autos.AutoRotate;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.CoralStationSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.commands.autos.CoralAutos.AutoCoralIntake;
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RightAuto extends SequentialCommandGroup {
        final CoralArm coralArm;
        final AlgaeArm algaeArm;
        final Elevator elevator;
        final SwerveSubsystem swerveSubsystem;

        public RightAuto(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator, SwerveSubsystem swerve) {
                this.coralArm = coralArm;
                this.algaeArm = algaeArm;
                this.elevator = elevator;
                this.swerveSubsystem = swerve;

                addCommands(
                                // new AutoRotate(swerve, 90, 0.5),
                                GoTo.reefNE(),
                                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerve,
                                                Constants.SetpointConstants.StrafeOffsets.leftL4),
                                GoTo.coralStationRight(),
                                new CoralStationSequence(coralArm, algaeArm, elevator, swerve),
                                GoTo.reefSE(),
                                new L4AlignmentSequence(coralArm, algaeArm, elevator, swerve,
                                                Constants.SetpointConstants.StrafeOffsets.leftL4),
                                GoTo.coralStationRight(),
                                new CoralStationSequence(coralArm, algaeArm, elevator, swerve),
                                GoTo.reefSE()

                // new ParallelCommandGroup(new AutoNav(), ConfigSystem)
                // aim andl score corale
                // new AutoNav(0) //go to coral station
                // aim and intake coral
                // run to reef spot again
                // aim and score coral

                );
        }
}