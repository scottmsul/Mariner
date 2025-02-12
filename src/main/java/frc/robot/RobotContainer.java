// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DefaultSwerve;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeSub;

public class RobotContainer {

  Joystick primaryJoy = new Joystick(0);
  XboxController secondaryController = new XboxController(1);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  Elevator elevatorSub = new Elevator();
  //CoralArm coralArm = new CoralArm();
  AlgaeSub algaeSub = new AlgaeSub();
  //ClimbSub climbSub = new ClimbSub();

  DefaultSwerve swerve = new DefaultSwerve(primaryJoy, swerveSubsystem);

  // AutoNav autoNav = new AutoNav();

  // Elevator elevator = new Elevator();

  public RobotContainer() {
    //URCL.start();    

    swerveSubsystem.setDefaultCommand(swerve);
    configureBindings();

    // dependent on the alliance color
    // have to rotate to face april tag at each pos
    // have to align with april tag after
    // left coral station (1.5, 6.5)
    // right coral station (2, 1.7)
    // processor (6.3, 1.55)
    // }
  }

  private void configureBindings() {
    // new JoystickButton(primaryJoy, 0).onTrue(AutoNav.goTo(1));

    new JoystickButton(secondaryController, XboxController.Button.kY.value)
        .onTrue(elevatorSub.stow());
    new JoystickButton(secondaryController, XboxController.Button.kA.value)
        .onTrue(elevatorSub.l1());
    //new JoystickButton(secondaryController, XboxController.Button.kB.value)
        //.onTrue(elevatorSub.l2());
    //new JoystickButton(secondaryController, XboxController.Button.kX.value)
        //.onTrue(elevatorSub.l3());
    new JoystickButton(secondaryController, XboxController.Button.kStart.value)
        .onTrue(elevatorSub.l4());
    //new JoystickButton(secondaryController, XboxController.Button.kB.value)
        //.onTrue(coralArm.lmid());
    new JoystickButton(secondaryController, XboxController.Button.kX.value)
      .onTrue(algaeSub.algaeArmDown());
    new JoystickButton(secondaryController, XboxController.Button.kB.value)
      .onTrue(algaeSub.algaeArmDown());
    // new JoystickButton(secondaryController, XboxController.Button.kB.value)
    //   .onTrue(algaeSub.algaeArmStop());
    // secondaryController.getPOV()
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
