package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralArm;

public class DefaultCoral extends Command{
    private final Joystick primaryJoystick;
    private final XboxController secondController;
    CoralArm coralArm;

    public DefaultCoral(CoralArm coralArm, Joystick primaryJoystick, XboxController secondController){
        addRequirements(coralArm);
        this.primaryJoystick = primaryJoystick;
        this.coralArm = coralArm;
        this.secondController = secondController;
    }

    //what is the meaning of this

    // public void intakeCoral(){
    //     while(!coralArm.hasCoral()){
    //         coralArm.intakeCoral();
    //     }
    //     coralArm.stopCoralRoller();
    // }

    // public void releaseCoral(){
    //     while(coralArm.hasCoral()){
    //         coralArm.releaseCoral();
    //     }
    //     coralArm.stopCoralRoller();
    // }


    //methods
        //when get some button
            //intake coral
        //when get other button
            //release coral
        //when get some joystick
            //have it pivot the arm
        
}
