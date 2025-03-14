package frc.robot.commands.autos.AutoSequences.AlignmentSequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AlgaeAutos.AutoAlgaeIntake;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoIntakeAlgae extends SequentialCommandGroup {
    public AutoIntakeAlgae(CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm, SwerveSubsystem swerveSubsystem) {
        var config = new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevator, algaeArm);
        var intakeAlgae = new AutoAlgaeIntake(algaeArm);
        addCommands(
                Commands.print("AutoIntakeAlgaeStart"),
                intakeAlgae.until(algaeArm::hasAlgae),
                Commands.print("AutoIntakeAlgae Done Intaking"),
                config,
                Commands.print("AutoIntakeAlgae Done Config"));
    }
}
