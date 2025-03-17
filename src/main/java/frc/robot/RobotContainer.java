// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import org.littletonrobotics.urcl.URCL;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.SetpointConstants.ConfigOption;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.GoTo;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.AbortAbortReef;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.AbortAbortUpper;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.AlgaeIntakeAlignmentSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.CoralStationSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L1AlignmentSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.L4AlignmentSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.LeftAlignmentSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.ProcessorAlignmentSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.RightAlignmentSequence;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.ClimbSub;
import frc.robot.subsystems.CoralArm;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {

  private boolean isOffset;
  CommandJoystick primaryJoy = new CommandJoystick(0);
  XboxController secondaryController = new XboxController(1);
  //GenericHID keyboard = new GenericHID(1);

  SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  Elevator elevatorSub = new Elevator();
  // CoralArm coralArm = new CoralArm();
  AlgaeArm algaeArm = new AlgaeArm();
  // ClimbSub climbSub = new ClimbSub();
  CoralArm coralArm = new CoralArm(elevatorSub);

  ClimbSub climbSub = new ClimbSub();
  
  DefaultSwerve swerve = new DefaultSwerve(primaryJoy.getHID(), swerveSubsystem, elevatorSub);


  PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  GoTo goTo = new GoTo();

  private void isOffset(boolean isOffset){
    this.isOffset = isOffset;
  }

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

    if (false) {
      DataLogManager.start();
      var sparks = new HashMap<Integer, String>();
      sparks.put(10, "SwerveTurnBR");
      sparks.put(11, "SwerveDriveBR");
      sparks.put(12, "SwerveTurnFR");
      sparks.put(13, "SwerveDriveFR");
      sparks.put(14, "SwerveTurnFL");
      sparks.put(15, "SwerveDriveFL");
      sparks.put(16, "SwerveTurnBL");
      sparks.put(17, "SwerveDriveBL");

      sparks.put(59, "Elevator1");
      sparks.put(60, "Elevator2");
      URCL.start(sparks);
    }
    // CameraServer.startAutomaticCapture();

    // Shuffleboard.getTab("Debug").add(pdh);

    // dependent on the alliance color
    // have to rotate to face april tag at each pos
    // have to align with april tag after
    // left coral station (1.5, 6.5)
    // right coral station (2, 1.7)
    // processor (6.3, 1.55)
    // }

    climbSub.setDefaultCommand(Commands.run(() -> {
      climbSub.climbWithSpeed(secondaryController.getRightTriggerAxis());
    }, climbSub));
  }  

  
  private boolean getUpperTag(){
    return LimelightHelpers.getTV(Constants.UpperLimelightName);
  }
  private boolean getLowerTag(){
    return LimelightHelpers.getTV(Constants.ReefLimelightName);
  }
  private boolean getNoTag(){
    return !getLowerTag() && !getUpperTag();
  }

  private Pose2d inFrontOfTag(int id){
    Transform2d rot180 = new Transform2d(Translation2d.kZero, Rotation2d.k180deg);
    var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    var tag8 = field.getTagPose(id).get().toPose2d();
    var offset = new Transform2d(1, 0, new Rotation2d());
    Pose2d infrontOfTag = tag8.plus(offset).transformBy(rot180);
    return infrontOfTag;
  }

  private void configureBindings() {
  //   // new JoystickButton(primaryJoy, 0).onTrue(AutoNav.goTo(1));

  //   // new JoystickButton(secondaryController, XboxController.Button.kY.value)
  //   //     .onTrue(Commands.parallel(elevatorSub.stow(), algaeSub.algaeArmUp(), coralArm.up()));
  //   // new JoystickButton(secondaryController, XboxController.Button.kA.value)
  //   //     .onTrue(elevatorSub.l1());
  //   // new JoystickButton(primaryJoy, 3)
  //   //     .onTrue(elevatorSub.l2());
  //   // new JoystickButton(primaryJoy, 4)
  //   //     .onTrue(elevatorSub.l3());
  //   // new JoystickButton(secondaryController, XboxController.Button.kStart.value)
  //   //     .onTrue(elevatorSub.l4());
  //   // var button3 = new JoystickButton(primaryJoy, 3);
  //   // var button4 = new JoystickButton(primaryJoy, 4);

  //   // button3.and(button4).onTrue(elevatorSub.algaeLow());


  //   //new JoystickButton(secondaryController, XboxController.Button.kB.value)
  //       //.onTrue(coralArm.lmid());
  //   // new JoystickButton(secondaryController, XboxController.Button.kX.value)
  //   //   .onTrue(algaeSub.algaeArmDown());
  //   // new JoystickButton(secondaryController, XboxController.Button.kB.value)
  //   //   .onTrue(algaeSub.algaeArmUp());
  //   // new JoystickButton(secondaryController, XboxController.Button.kBack.value)
  //   //   .onTrue(coralArm.coralStation());

  //   // new JoystickButton(secondaryController, XboxController.Button.kA.value)
  //   //   .onTrue(algaeArm.algaeSpinIn())
  //   //   .onFalse(algaeArm.algaeSpinStop());
  //   // new JoystickButton(secondaryController, XboxController.Button.kB.value)
  //   //   .onTrue(algaeArm.algaeSpinOut())
  //   //   .onFalse(algaeArm.algaeSpinStop());

  //   // new JoystickButton(secondaryController, XboxController.Button.kX.value)
  //   //   .onTrue(coralArm.intakeCoralCommand())
  //   //   .onFalse(coralArm.stopCoralSpin());
  //   // new JoystickButton(secondaryController, XboxController.Button.kY.value)
  //   //   .onTrue(coralArm.outtakeCoralCommand())
  //   //   .onFalse(coralArm.stopCoralSpin());

  //   // new JoystickButton(primaryJoy, 7)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l4Left, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(primaryJoy, 9)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l3Left, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(primaryJoy, 11)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l2Left, coralArm, elevatorSub, algaeArm));
  //   // //new JoystickButton(primaryJoy, 4)
  //   // //  .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l1, coralArm, elevatorSub, algaeArm));

  //   // new JoystickButton(primaryJoy, 10)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.algaeLow, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(primaryJoy, 8)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.algaeHigh, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(primaryJoy, 3)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.coralStation, coralArm, elevatorSub, algaeArm));
    
    primaryJoy.button(3)
      .onTrue(coralArm.intakeCoralCommand())
      .onFalse(coralArm.stopCoralSpin());
    primaryJoy.button(5)
      .onTrue(coralArm.outtakeCoralCommand())
      .onFalse(coralArm.stopCoralSpin());
    primaryJoy.button(4)
    .onTrue(algaeArm.algaeSpinIn())
    .onFalse(algaeArm.algaeSpinStop());
    primaryJoy.button(6)
      .onTrue(algaeArm.algaeSpinOut())
      .onFalse(algaeArm.algaeSpinStop());
  //   primaryJoy.button(10)
  //     .whileTrue(new AutoAlignReef(swerveSubsystem, NTD.of(0), NTD.of(0.75), NTD.of(0), NTD.of(0.02), NTD.of(0.02)).alongWith(Commands.print("aligning to center")));
  //   // new JoystickButton(primaryJoy, 8)
  //   //   .whileTrue(new AutoAlignTags(swerveSubsystem, 0.11, 0.5, 0));
  //   // new JoystickButton(primaryJoy, 9)
  //   //   .whileTrue(new AutoAlignTags(swerveSubsystem, -0.21, 0.5, 0));
  primaryJoy.button(11).and(primaryJoy.button(7))
      .onTrue(climbSub.climbSlow().withTimeout(0.5).andThen(climbSub.climb()))
      .onFalse(climbSub.climbStopManual());
  // primaryJoy.button(10).and(primaryJoy.button(7))
  //     .onTrue(climbSub.climb())
  //     .onFalse(climbSub.climbStopManual());
  //   //new JoystickButton(keyboard, 0);
  //   // var button7 = new JoystickButton(primaryJoy, 7);
  //   // var button8 = new JoystickButton(primaryJoy, 8);
  //   // button7.and(button8).onTrue(coralArm.coralStation());
  //   //new JoystickButton(primaryJoy, 6)
  //    // .onTrue(new ConfigSystem(Constants.SetpointConstants.OptionArrays.positionList, 0, coralArm, elevatorSub, algaeSub));

  //   // new JoystickButton(keyboard,8)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.algaeHigh, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard,5)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.algaeLow, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard,2)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l1, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard, 9)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l4, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard,6)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l3, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard, 3)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.l2, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard, 7)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.coralStation, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard, 4)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevatorSub, algaeArm));
  //   // //new JoystickButton(keyboard, 10)
  //   // //  .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub, algaeArm));
  //   // //new JoystickButton(primaryJoy, 4)
  //   // //  .onTrue(climbSub.climbStopCommand());
  //   // new JoystickButton(keyboard, 10).and(algaeArm::hasAlgae)
  //   //  .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(keyboard, 10).and(() -> !algaeArm.hasAlgae())
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub, algaeArm));
  //   // new JoystickButton(primaryJoy, 9)
  //   //   .onTrue(new ConfigSystem(Constants.SetpointConstants.Options.barge, coralArm, elevatorSub, algaeArm));

    var leftBumper = new JoystickButton(secondaryController, XboxController.Button.kLeftBumper.value);
    var rightBumper = new JoystickButton(secondaryController, XboxController.Button.kRightBumper.value);
    var xboxA = new JoystickButton(secondaryController, XboxController.Button.kA.value);
    var xboxB = new JoystickButton(secondaryController, XboxController.Button.kB.value);
    var xboxX = new JoystickButton(secondaryController, XboxController.Button.kX.value);
    var xboxY = new JoystickButton(secondaryController, XboxController.Button.kY.value);
    var leftStick = new JoystickButton(secondaryController, XboxController.Button.kLeftStick.value);
    var rightStick = new JoystickButton(secondaryController, XboxController.Button.kRightStick.value);
    var xboxStart = new JoystickButton(secondaryController, XboxController.Button.kStart.value);
    var xboxBack = new JoystickButton(secondaryController, XboxController.Button.kBack.value);
  //   //var leftPOV = new JoystickButton(secondaryController, XboxController.Button.)

  //   var stow = new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub, algaeArm);



    // leftBumper.onTrue(new AutoAlignTags(swerveSubsystem, 0.13, 0.6, 0).alongWith(Commands.print("aligning to x goal 0.11")));
    // rightBumper.onTrue(new AutoAlignTags(swerveSubsystem, -0.2, 0.63, 0).alongWith(Commands.print("aligning to -0.21")));

    var noBumper = leftBumper.or(rightBumper).negate();
    var hasAlgae = new Trigger(algaeArm::hasAlgae);

    var hasUpperTarget = new Trigger(this::getUpperTag);
    var hasLowerTarget = new Trigger(this::getLowerTag);
    var hasNoTarget = new Trigger(this::getNoTag);

  //   //xboxA.and(leftBumper).onTrue();
  //   //xboxA.and(noBumper).onTrue(new PrintCommand("no bumper and button pressed"));

    xboxA.and(leftBumper).onTrue(
      new L1AlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.l1Left, Constants.SetpointConstants.DistanceOffsets.l1, Constants.SetpointConstants.RotOffsets.l1Left));
    xboxB.and(leftBumper).onTrue(
      new LeftAlignmentSequence(coralArm,algaeArm,elevatorSub,swerveSubsystem,Constants.SetpointConstants.Options.l2));
    xboxX.and(leftBumper).onTrue(
      new LeftAlignmentSequence(coralArm,algaeArm,elevatorSub,swerveSubsystem,Constants.SetpointConstants.Options.l3));
    xboxY.and(leftBumper).onTrue(
      new L4AlignmentSequence(coralArm, algaeArm, elevatorSub,swerveSubsystem,Constants.SetpointConstants.StrafeOffsets.leftL4));
      // new LeftAlignmentSequence(coralArm,algaeArm,elevatorSub,swerveSubsystem,Constants.SetpointConstants.Options.l4));
    xboxA.and(rightBumper).onTrue(
      new L1AlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem, Constants.SetpointConstants.StrafeOffsets.l1Right, Constants.SetpointConstants.DistanceOffsets.l1, Constants.SetpointConstants.RotOffsets.l1Right));
    xboxB.and(rightBumper).onTrue(
      new RightAlignmentSequence(coralArm,algaeArm,elevatorSub,swerveSubsystem,Constants.SetpointConstants.Options.l2));
    xboxX.and(rightBumper).onTrue(
      new RightAlignmentSequence(coralArm,algaeArm,elevatorSub,swerveSubsystem,Constants.SetpointConstants.Options.l3));
    xboxY.and(rightBumper).onTrue(
      new L4AlignmentSequence(coralArm, algaeArm, elevatorSub,swerveSubsystem,Constants.SetpointConstants.StrafeOffsets.rightL4));
    xboxA.and(noBumper).and(hasAlgae).and(hasUpperTarget).onTrue(
      new ProcessorAlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem));
    xboxA.and(noBumper).and(hasAlgae.negate()).and(hasLowerTarget).onTrue(
      new AbortAbortReef(coralArm, algaeArm, elevatorSub, swerveSubsystem));
    xboxA.and(noBumper).and(hasAlgae.negate()).and(hasUpperTarget).onTrue(
      new AbortAbortUpper(coralArm, algaeArm, elevatorSub, swerveSubsystem));
    xboxA.and(noBumper).and(hasAlgae).and(hasUpperTarget.negate()).onTrue(
      new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevatorSub, algaeArm));
    xboxA.and(noBumper).and(hasAlgae.negate()).and(hasNoTarget).onTrue(
      new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub, algaeArm));
    xboxB.and(noBumper).onTrue(
      new CoralStationSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem));
    xboxX.and(noBumper).onTrue(
      new AlgaeIntakeAlignmentSequence(coralArm, elevatorSub, algaeArm, swerveSubsystem, Constants.SetpointConstants.Options.algaeLow));
    xboxY.and(noBumper).onTrue(
      new AlgaeIntakeAlignmentSequence(coralArm, elevatorSub, algaeArm, swerveSubsystem, Constants.SetpointConstants.Options.algaeHigh));
    xboxStart.and(noBumper).onTrue(
        new ConfigSystem(Constants.SetpointConstants.Options.algaeGround, coralArm, elevatorSub, algaeArm));

//         CommandScheduler.getInstance().onCommandInitialize((c) -> System.out.println("initalizing: "+c.getName()));
//         CommandScheduler.getInstance().onCommandExecute((c) ->{
//  if (c.getName().equals("DefaultSwerve")) return;
//  System.out.println("executing: "+c.getName());
//         });
//         CommandScheduler.getInstance().onCommandFinish((c) -> System.out.println("finishing: "+c.getName()));
//         CommandScheduler.getInstance().onCommandInterrupt((c,b) -> System.out.println("finishing: "+c.getName() + " " + b));

    // primaryJoy
    //     .button(4)
    //     // AutoIntakeAlgae(coralArm,elevatorSub,algaeArm,swerveSubsystem));
    //     // .whileTrue(algaeArm.new AlgaeSpinCommandTest(algaeArm));
    //     // .whileTrue(Commands.sequence(
    //     //     new ConfigSystem(Constants.SetpointConstants.Options.algaeGround, coralArm, elevatorSub, algaeArm),
    //     //     Commands.print("Begin Spin"),
    //     //     algaeArm.new AlgaeSpinCommandTest(algaeArm),
    //     //     Commands.print("End Spin"),
    //     //     new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm,
    //     //         elevatorSub, algaeArm)));
    //     .whileTrue(new algaeArm);

    xboxBack.and(xboxA).onTrue(
        new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub, algaeArm));
    xboxBack.and(xboxX).onTrue(
        new ConfigSystem(Constants.SetpointConstants.Options.l3, coralArm, elevatorSub, algaeArm));
    xboxBack.and(xboxY).onTrue(
        new ConfigSystem(Constants.SetpointConstants.Options.l4, coralArm, elevatorSub, algaeArm));

    var constraints = new PathConstraints(1.75, 2, 360, 360);
    // Transform2d rot180 = new Transform2d(Translation2d.kZero, Rotation2d.k180deg);
    // var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    // var tag8 = field.getTagPose(8).get().toPose2d();
    // var offset = new Transform2d(1, 0, new Rotation2d());
    // Pose2d infrontOfTag8 = tag8.plus(offset).transformBy(rot180);
    Pose2d infrontOfTag8 = inFrontOfTag(8);

    Shuffleboard.getTab("PathPlanner").add(
        "Goto Before 8",
        AutoBuilder.pathfindToPose(infrontOfTag8, constraints));
    Shuffleboard.getTab("PathPlanner").add(
        "class go to before 8",
        goTo.testTag8());
  }

  public class MyCommandShouldHaveAName extends SequentialCommandGroup {
    MyCommandShouldHaveAName(CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm) {
      addRequirements(algaeArm);
      addRequirements(coralArm);
      addRequirements(elevator);

      addCommands(
            new ConfigSystem(Constants.SetpointConstants.Options.algaeGround, coralArm, elevatorSub, algaeArm),
            Commands.print("Begin Spin"),
            algaeArm.new AlgaeSpinCommandTest(algaeArm),
            Commands.print("End Spin"),
            new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm,
                elevatorSub, algaeArm));
      
    }


  }

  public Command simple() {
    var pose = swerveSubsystem.getPose();
    // pose.plus(new Transform2d(.5, 0, new Rotation2d()));
    return AutoBuilder.pathfindToPose(
        pose,
        new PathConstraints(1, 1, 1, 1),
        0);
  }

public void help() {
  var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
  var pose = field.getTagPose(1).get();
  pose.plus(new Transform3d(0, 1, 0, new Rotation3d()));
  AutoBuilder.pathfindToPose(
      pose.toPose2d(),
      new PathConstraints(1, 1, 1, 1),
      0);
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
