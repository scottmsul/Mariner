package frc.robot.commands.autos.AlgaeAutos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class AutoAlgaeIntake extends Command {
    AlgaeArm algaeSub;

    public AutoAlgaeIntake(AlgaeArm algaeSub) {
        // addRequirements(algaeSub);
        this.algaeSub = algaeSub;
    }

    @Override
    public void execute() {
        // System.out.println("grab grab grab");
        algaeSub.grab();
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            System.out.println("Algae Interrupted");
        else
            System.out.println("Algae Done");
        algaeSub.stop();
    }

    // @Override
    // public boolean isFinished() {
    // return algaeSub.hasAlgae();
    // }
}
