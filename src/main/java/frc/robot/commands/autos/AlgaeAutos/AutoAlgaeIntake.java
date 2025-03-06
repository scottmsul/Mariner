package frc.robot.commands.autos.AlgaeAutos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AlgaeArm;

public class AutoAlgaeIntake extends Command {
        AlgaeArm algaeSub;

        public AutoAlgaeIntake(AlgaeArm algaeSub) {
            addRequirements(algaeSub);
            this.algaeSub = algaeSub;
        }

        @Override
        public void execute() {
            algaeSub.grab();
        }

        @Override
        public void end(boolean interrupted) {
            algaeSub.stop();
        }

        // @Override
        // public boolean isFinished() {
        //     return algaeSub.hasAlgae();
        // }
    }

