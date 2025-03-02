package frc.robot.commands.Configuration;

import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants.Options;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;

public class ConfigSystem extends InstantCommand{

    int choice;

    CoralArm coralArm;
    Elevator elevator;
    AlgaeArm algaeArm;
    Options options;
    ConfigOption configOption;

    public ConfigSystem(ConfigOption configOption, CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm) {

        addRequirements(coralArm);
        addRequirements(elevator);
        addRequirements(algaeArm);

        
        this.coralArm = coralArm;
        this.elevator = elevator;
        this.algaeArm = algaeArm;
        this.configOption = configOption;

    }

    public boolean isConfigured(){
        return coralArm.isReady() && algaeArm.isReady() && elevator.isReady();
    }

    @Override
    public void execute() {
        coralArm.setCoralWristSetpoint(configOption.coralAngle);
        elevator.setPosition(configOption.elevatorSetpoint);
        algaeArm.setAlgaeSetpoint(configOption.algaeAngle);
    }

    // public Command configureCommand(int choice){
    //     return run(() -> configure(choice));
    // }


}
