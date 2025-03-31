package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class FinalAuto extends SequentialCommandGroup {
    public FinalAuto(Command command1, Command command2) {
        addCommands(command1, command2);
    }
}
