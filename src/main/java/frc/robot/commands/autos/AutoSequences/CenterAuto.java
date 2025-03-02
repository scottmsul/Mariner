package frc.robot.commands.autos.AutoSequences;

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

public class CenterAuto extends SequentialCommandGroup {
    final CoralArm coralArm;
    final AlgaeArm algaeArm;
    final Elevator elevator ;

    public CenterAuto(CoralArm coralArm, AlgaeArm algaeArm, Elevator elevator) {
        this.coralArm = coralArm;
        this.algaeArm = algaeArm;
        this.elevator = elevator;

        addCommands(
        new AutoNav(0), //go to reef side
        new ConfigSystem(Constants.SetpointConstants.Options.l3, coralArm, elevator, algaeArm),
        new AutoCoralScore(coralArm),
        new AutoNav(0), //coral station //maybe have it run autonav and stow config in parallel
        new ConfigSystem(Constants.SetpointConstants.Options.coralStation, coralArm, elevator, algaeArm),
        new AutoCoralIntake(coralArm)
        //new ParallelCommandGroup(new AutoNav(), ConfigSystem)
        // aim andl score corale
        // new AutoNav(0) //go to coral station
        // aim and intake coral
        // run to reef spot again
        // aim and score coral

        );
    }
}