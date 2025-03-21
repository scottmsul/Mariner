package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.NTDouble.NTD;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignReef;
import frc.robot.commands.autos.AutoDrive;
import frc.robot.commands.autos.AlgaeAutos.AutoAlgaeIntake;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class AlgaeIntakeAlignmentSequence extends SequentialCommandGroup {

    public AlgaeIntakeAlignmentSequence(CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm,
            SwerveSubsystem swerveSubsystem, ConfigOption configOption) {
        var config = new ConfigSystem(configOption, coralArm, elevator, algaeArm);
        var stow = new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevator, algaeArm);
        var configureAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.centerReef,
                Constants.SetpointConstants.DistanceOffsets.reefAlgaeConfigure, NTD.of(0), NTD.of(0.04), NTD.of(0.04));
        var secondConfigureAlign = new AutoAlignReef(swerveSubsystem, NTD.of(0),
                Constants.SetpointConstants.DistanceOffsets.reefAlgaeConfigure, NTD.of(0), NTD.of(0.04), NTD.of(0.04));
        var stowAlgaeAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.centerReef,
                Constants.SetpointConstants.DistanceOffsets.reefAlgaeStow, NTD.of(0), NTD.of(0.04), NTD.of(0.04));
        var intakeAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.centerReef,
                Constants.SetpointConstants.DistanceOffsets.algaeReefGrab, NTD.of(0), NTD.of(0.02), NTD.of(0.02));
        var intakeAlgae = new AutoAlgaeIntake(algaeArm);
        var autoDriveBack = new AutoDrive(swerveSubsystem, 0.15, -0.1);
        addCommands(
                // new ParallelCommandGroup(
                Commands.print("Start AlgaeIntakeAlighnSeque"),
                configureAlign.andThen(Commands.print("aligned")),
                config.andThen(Commands.print("configed")),
                new ParallelRaceGroup(
                        intakeAlign,
                        intakeAlgae.until(algaeArm::hasAlgae)).andThen(Commands.print("algaeIntaked")),
                autoDriveBack,
                stow
        // intakeAlign.andThen(Commands.print("intaked"))
        // ),
        // intakeAlign,
        // intakeAlgae,
        // secondConfigureAlign,
        // stow
        );
    }
}
