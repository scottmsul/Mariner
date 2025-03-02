package frc.robot.commands.autos.CoralAutos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

    public class AutoCoralIntake extends Command {
        private CoralArm coralArm;

        public AutoCoralIntake(CoralArm coralArm) {
            addRequirements(coralArm);
            this.coralArm = coralArm;
        }

        @Override
        public void execute() {
            coralArm.intakeCoral();
        }

        @Override
        public void end(boolean interrupted) {
            coralArm.stopCoralRoller();
        }

        // @Override
        // public boolean isFinished() {
        //     // return coralArm.hasCoral();
        // }
    }


