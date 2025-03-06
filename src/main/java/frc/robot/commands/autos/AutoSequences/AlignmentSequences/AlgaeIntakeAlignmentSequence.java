package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoAlignReef;
import frc.robot.commands.autos.AlgaeAutos.AutoAlgaeIntake;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class AlgaeIntakeAlignmentSequence extends SequentialCommandGroup{

    public AlgaeIntakeAlignmentSequence(CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm, SwerveSubsystem swerveSubsystem, ConfigOption configOption) {
                var config = new ConfigSystem(configOption, coralArm, elevator, algaeArm);
                var stow = new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevator, algaeArm);
                var configureAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.centerReef,Constants.SetpointConstants.DistanceOffsets.reefAlgaeConfigure, 0, 0.04, 0.04);
                var secondConfigureAlign = new AutoAlignReef(swerveSubsystem, 0 ,Constants.SetpointConstants.DistanceOffsets.reefAlgaeConfigure, 0, 0.04, 0.04);
                var stowAlgaeAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.centerReef,Constants.SetpointConstants.DistanceOffsets.reefAlgaeStow, 0, 0.04, 0.04);
                var intakeAlign = new AutoAlignReef(swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.centerReef, Constants.SetpointConstants.DistanceOffsets.algaeReefGrab, 0, 0.02, 0.02);
                var intakeAlgae = new AutoAlgaeIntake(algaeArm);
        addCommands(
            // new ParallelCommandGroup(
                configureAlign.andThen(Commands.print("aligned")),
                config.andThen(Commands.print("configed")),
                new ParallelRaceGroup(
                    intakeAlign,
                    intakeAlgae.until(algaeArm::hasAlgae)
                ).andThen(Commands.print("algaeIntaked")),
                secondConfigureAlign.andThen(Commands.print("second aligned")),
                stow
                //intakeAlign.andThen(Commands.print("intaked"))
            // ),
            // intakeAlign,
            // intakeAlgae,
            // secondConfigureAlign,
            // stow
        );
    }
}
