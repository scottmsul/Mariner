// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.autos.AutoNav;
import frc.robot.subsystems.Elevator;

public class RobotContainer {

  Joystick primaryJoy = new Joystick(0);
  XboxController secondaryController = new XboxController(1);

  AutoNav autoNav = new AutoNav();

  Elevator elevator = new Elevator();

  public RobotContainer() {
    // configureBindings(){

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
        .onTrue(Commands.run(elevator::stow));
    new JoystickButton(secondaryController, XboxController.Button.kA.value)
        .onTrue(Commands.run(elevator::l1));
    // secondaryController.getPOV()

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
