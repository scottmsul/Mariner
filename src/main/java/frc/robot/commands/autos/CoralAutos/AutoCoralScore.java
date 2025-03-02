package frc.robot.commands.autos.CoralAutos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class AutoCoralScore extends Command{
     private CoralArm coralArm;

        public AutoCoralScore(CoralArm coralArm) {
            addRequirements(coralArm);
            this.coralArm = coralArm;
        }

        @Override
        public void execute() {
            coralArm.releaseCoral();
        }

        @Override
        public void end(boolean interrupted) {
            coralArm.stopCoralRoller();
        }

        // @Override
        // public boolean isFinished() {
        //     return !coralArm.hasCoral();
        // }
}
