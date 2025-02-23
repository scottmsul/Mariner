// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.print.attribute.standard.JobHoldUntil;

import edu.wpi.first.cameraserver.CameraServer;

//import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SetpointConstants;
import frc.robot.Constants.SetpointConstants.Options;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ClimbSub;

public class RobotContainer {

  Joystick primaryJoy = new Joystick(0);
  XboxController secondaryController = new XboxController(1);
  GenericHID keyboard = new GenericHID(2);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  Elevator elevatorSub = new Elevator();
  // CoralArm coralArm = new CoralArm();
  AlgaeArm algaeArm = new AlgaeArm();
  // ClimbSub climbSub = new ClimbSub();
  CoralArm coralArm = new CoralArm();

  ClimbSub climbSub = new ClimbSub();
  
  DefaultSwerve swerve = new DefaultSwerve(primaryJoy, swerveSubsystem);


  // PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  //Constants constants = new Constants()
  //SetpointConstants setpointConstants = new SetpointConstants();
  //Options options = new Options();

  //ConfigSystem configSystem = new ConfigSystem(Constants.SetpointConstants.OptionArrays.positionList, coralArm, elevatorSub, algaeSub)

  // AutoNav autoNav = new AutoNav();

  // Elevator elevator = new Elevator();

  public RobotContainer() {
    //URCL.start();    

    swerveSubsystem.setDefaultCommand(swerve);
    configureBindings();
    CameraServer.startAutomaticCapture();

    // Shuffleboard.getTab("Debug").add(pdh);

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

    // new JoystickButton(secondaryController, XboxController.Button.kY.value)
    //     .onTrue(Commands.parallel(elevatorSub.stow(), algaeSub.algaeArmUp(), coralArm.up()));
    // new JoystickButton(secondaryController, XboxController.Button.kA.value)
    //     .onTrue(elevatorSub.l1());
    // new JoystickButton(primaryJoy, 3)
    //     .onTrue(elevatorSub.l2());
    // new JoystickButton(primaryJoy, 4)
    //     .onTrue(elevatorSub.l3());
    // new JoystickButton(secondaryController, XboxController.Button.kStart.value)
    //     .onTrue(elevatorSub.l4());
    // var button3 = new JoystickButton(primaryJoy, 3);
    // var button4 = new JoystickButton(primaryJoy, 4);

    // button3.and(button4).onTrue(elevatorSub.algaeLow());


    //new JoystickButton(secondaryController, XboxController.Button.kB.value)
        //.onTrue(coralArm.lmid());
    // new JoystickButton(secondaryController, XboxController.Button.kX.value)
    //   .onTrue(algaeSub.algaeArmDown());
    // new JoystickButton(secondaryController, XboxController.Button.kB.value)
    //   .onTrue(algaeSub.algaeArmUp());
    // new JoystickButton(secondaryController, XboxController.Button.kBack.value)
    //   .onTrue(coralArm.coralStation());

    new JoystickButton(secondaryController, XboxController.Button.kA.value)
      .onTrue(algaeArm.algaeSpinIn())
      .onFalse(algaeArm.algaeSpinStop());
    new JoystickButton(secondaryController, XboxController.Button.kB.value)
      .onTrue(algaeArm.algaeSpinOut())
      .onFalse(algaeArm.algaeSpinStop());

    new JoystickButton(secondaryController, XboxController.Button.kX.value)
      .onTrue(coralArm.intakeCoralCommand())
      .onFalse(coralArm.stopCoralSpin());
    new JoystickButton(secondaryController, XboxController.Button.kY.value)
      .onTrue(coralArm.outtakeCoralCommand())
      .onFalse(coralArm.stopCoralSpin());

    new JoystickButton(primaryJoy, 7)
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l4Left, coralArm, elevatorSub, algaeArm));
    new JoystickButton(primaryJoy, 9)
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l3Left, coralArm, elevatorSub, algaeArm));
    new JoystickButton(primaryJoy, 11)
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l2Left, coralArm, elevatorSub, algaeArm));
    //new JoystickButton(primaryJoy, 4)
    //  .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l1, coralArm, elevatorSub, algaeArm));

    new JoystickButton(primaryJoy, 10)
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.algaeLow, coralArm, elevatorSub, algaeArm));
    new JoystickButton(primaryJoy, 8)
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.algaeHigh, coralArm, elevatorSub, algaeArm));
    new JoystickButton(primaryJoy, 3)
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.coralStation, coralArm, elevatorSub, algaeArm));
    
    //new JoystickButton(keyboard, 0);
    // var button7 = new JoystickButton(primaryJoy, 7);
    // var button8 = new JoystickButton(primaryJoy, 8);
    // button7.and(button8).onTrue(coralArm.coralStation());
    //new JoystickButton(primaryJoy, 6)
     // .onTrue(new ConfigSystem(Constants.SetpointConstants.OptionArrays.positionList, 0, coralArm, elevatorSub, algaeSub));

    new JoystickButton(primaryJoy, 5)
      .onTrue(climbSub.climb());
    new JoystickButton(primaryJoy, 4)
      .onTrue(climbSub.climbStopCommand());
    new JoystickButton(primaryJoy, 6).and(algaeArm::hasAlgae)
     .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevatorSub, algaeArm));
    new JoystickButton(primaryJoy, 6).and(() -> !algaeArm.hasAlgae())
      .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub, algaeArm));
    // new JoystickButton(secondaryController, XboxController.Button.kB.value)
    //   .onTrue(algaeSub.algaeArmStop());
    // secondaryController.getPOV()
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
