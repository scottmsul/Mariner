package frc.robot.commands.Configuration;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants.Options;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class ConfigSystem extends Command{

    int choice;

    CoralArm coralArm;
    Elevator elevator;
    AlgaeArm algaeArm;
    Options options;
    ConfigOption configOption;

    static int i = 0;

    public ConfigSystem(ConfigOption configOption, CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm) {

        addRequirements(coralArm);
        addRequirements(elevator);
        addRequirements(algaeArm);

        
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.algaeArm = algaeArm;
        this.configOption = configOption;

        // Shuffleboard.getTab("Debug").addBoolean("configured" + i++, () -> isConfigured());

    }

    public boolean isConfigured(){
        return coralArm.isReady() && algaeArm.isReady() && elevator.isReady();
    }

    @Override
    public void initialize() {
        coralArm.setCoralWristSetpoint(configOption.coralAngle.get());
        elevator.setPosition(configOption.elevatorSetpoint.get());
        algaeArm.setAlgaeSetpoint(configOption.algaeAngle.get());
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        var done = isConfigured();
        return done;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted)
            System.out.println("Config Interrupted");
        else
            System.out.println("Config Done");
    }

    // public Command configureCommand(int choice){
    //     return run(() -> configure(choice));
    // }


}
