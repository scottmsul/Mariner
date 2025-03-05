package frc.robot.commands.autos.AlgaeAutos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class AutoAlgaeScore extends Command{
        AlgaeArm algaeArm;

        public AutoAlgaeScore(AlgaeArm algaeArm) {
            addRequirements(algaeArm);
            this.algaeArm = algaeArm;
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
