package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeSub;

public class AlgaeAutos {
    public class AutoIntakeAlgae extends Command {
        AlgaeSub algaeSub = new AlgaeSub();

        AutoIntakeAlgae(AlgaeSub algaeSub) {
            addRequirements(algaeSub);
            this.algaeSub = algaeSub;
        }

        // @Override
        // public void execute() {
        //     algaeSub.grab();
        // }

        // @Override
        // public void end(boolean interrupted) {
        //     algaeSub.stop();
        // }

        // @Override
        // public boolean isFinished() {
        //     return algaeSub.hasAlgae();
        // }
    }

    public class AutoScoreAlgae extends Command {
        AlgaeSub algaeSub = new AlgaeSub();

        AutoScoreAlgae(AlgaeSub algaeSub) {
            addRequirements(algaeSub);
            this.algaeSub = algaeSub;
        }

        @Override
        public void execute() {
            // algaeSub.release();
        }

        @Override
        public void end(boolean interrupted) {
            // algaeSub.stop();
        }

        // @Override
        // public boolean isFinished() {
        //     // return !algaeSub.hasAlgae();
        // }
    }
}
