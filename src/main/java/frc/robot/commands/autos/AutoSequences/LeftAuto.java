package frc.robot.commands.autos.AutoSequences;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoNav;
import frc.robot.commands.autos.CoralAutos.AutoCoralIntake;
import frc.robot.commands.autos.CoralAutos.AutoCoralScore;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class LeftAuto extends SequentialCommandGroup {
    public LeftAuto(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator) {
        addCommands(
        new AutoNav(0), // Northwest
        new ConfigSystem(Constants.SetpointConstants.Options.l3, coralArm, elevator, algaeArm),
        new AutoCoralScore(coralArm),
        new AutoNav(0), //move away
        new ParallelCommandGroup(
            new SequentialCommandGroup(
                Commands.waitSeconds(1),
                new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevator, algaeArm)), 
            new AutoNav(0)), //coral station
        //new ConfigSystem(Constants.SetpointConstants.Options.testConfig, coralArm, elevator, algaeArm),
        //new AutoNav(0), //CoralStation
        new ConfigSystem(Constants.SetpointConstants.Options.coralStation, coralArm, elevator, algaeArm),
        new AutoCoralIntake(coralArm),
        new AutoNav(0),//Southwest
        new ConfigSystem(Constants.SetpointConstants.Options.l3, coralArm, elevator, algaeArm),
        new AutoCoralScore(coralArm)

        );
    }
}
