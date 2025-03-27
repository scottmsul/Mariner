// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.lang.StackWalker.Option;
import java.util.HashMap;
import java.util.Optional;

import org.littletonrobotics.urcl.URCL;

import com.ctre.phoenix6.swerve.jni.SwerveJNI.DriveState;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DefaultSwerve;
import frc.robot.commands.GoTo;
import frc.robot.commands.Configuration.ConfigSystem;
import frc.robot.commands.autos.AutoSequences.CenterAutoLeft;
import frc.robot.commands.autos.AutoSequences.CenterAutoRight;
import frc.robot.commands.autos.AutoSequences.CenterScoreOnce;
import frc.robot.commands.autos.AutoSequences.Forward;
import frc.robot.commands.autos.AutoSequences.LeftAuto;
import frc.robot.commands.autos.AutoSequences.LeftScoreOnce;
import frc.robot.commands.autos.AutoSequences.RightAuto;
import frc.robot.commands.autos.AutoSequences.RightScoreOnce;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.AlgaeIntakeAlignmentSequence;
import frc.robot.commands.autos.AutoSequences.AlignmentSequences.CoralStationSequence;
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

        // private boolean isOffset;
        CommandJoystick primaryJoy = new CommandJoystick(0);
        XboxController secondaryController = new XboxController(1);
        // GenericHID keyboard = new GenericHID(1);
        Servo servo = new Servo(0);

        SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
        Elevator elevatorSub = new Elevator();
        // CoralArm coralArm = new CoralArm();
        AlgaeArm algaeArm = new AlgaeArm();
        // ClimbSub climbSub = new ClimbSub();
        CoralArm coralArm = new CoralArm(elevatorSub);

        ClimbSub climbSub = new ClimbSub();

        DefaultSwerve swerve = new DefaultSwerve(primaryJoy.getHID(), swerveSubsystem, elevatorSub);

        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

        // private void isOffset(boolean isOffset) {
        // this.isOffset = isOffset;
        // }

        // Constants constants = new Constants()
        // SetpointConstants setpointConstants = new SetpointConstants();
        // Options options = new Options();

        // ConfigSystem configSystem = new
        // ConfigSystem(Constants.SetpointConstants.OptionArrays.positionList, coralArm,
        // elevatorSub, algaeSub)

        // AutoNav autoNav = new AutoNav();

        // Elevator elevator = new Elevator();
        enum StartingPlace {
                Left, Right, Center
        }

        SendableChooser<Command> autoChooser = AutoBuilder.buildAutoChooser();
        SendableChooser<StartingPlace> poseChooser = new SendableChooser<>();
        // GoTo goTo = new GoTo(sideChooser);

        public RobotContainer() {
                // if start lowercase, its object
                // NOT USED
                LiveWindow.enableTelemetry(CommandScheduler.getInstance());
                // URCL.start();
                autoChooser.addOption("left", new LeftAuto(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                autoChooser.addOption("center left cs",
                                new CenterAutoLeft(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                autoChooser.addOption("center right cs",
                                new CenterAutoRight(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                autoChooser.addOption("right", new RightAuto(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                autoChooser.addOption("goforward", new Forward(swerveSubsystem));
                autoChooser.setDefaultOption("Do Nothing", Commands.none());
                autoChooser.addOption("score once right",
                                new RightScoreOnce(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                autoChooser.addOption("score once right",
                                new LeftScoreOnce(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                autoChooser.addOption("score once center",
                                new CenterScoreOnce(coralArm, algaeArm, elevatorSub, swerveSubsystem));

                Shuffleboard.getTab("auto").add(autoChooser);

                poseChooser.addOption("Left", StartingPlace.Left);
                poseChooser.addOption("Center", StartingPlace.Center);
                poseChooser.addOption("Right", StartingPlace.Right);

                // sideChooser.addOption("Red", GoTo.side.Red);
                // sideChooser.addOption("Blue", GoTo.side.Blue);
                Shuffleboard.getTab("pose").add(poseChooser);
                // Shuffleboard.getTab("side").add(sideChooser);

                swerveSubsystem.setDefaultCommand(swerve);
                configureBindings();

                if (true) {
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

                        sparks.put(55, "CoralWrist");

                        sparks.put(56, "AlgaeWrist");
                        sparks.put(57, "AlgaeIntake");

                        sparks.put(58, "ClimbByBattery");
                        sparks.put(57, "ClimbByPDH");

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

                var scheduler = CommandScheduler.getInstance();
                Shuffleboard.getTab("Drive").addBoolean("Is Auto", () -> !scheduler.isScheduled(swerve));
        }

        private boolean getUpperTag() {
                return LimelightHelpers.getTV(Constants.UpperLimelightName);
        }

        private boolean getLowerTag() {
                return LimelightHelpers.getTV(Constants.ReefLimelightName);
        }

        private boolean getNoTag() {
                return !getLowerTag() && !getUpperTag();
        }

        private void configureBindings() {

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
                primaryJoy.button(9)
                                .onTrue(Commands.runOnce(() -> servo.set(0)))
                                .onFalse(Commands.runOnce(() -> servo.set(1)));
                primaryJoy.button(11).and(primaryJoy.button(7))
                                .onTrue(climbSub.climbSlow().withTimeout(0.5).andThen(climbSub.climb()))
                                .onFalse(climbSub.climbStopManual());
                primaryJoy.button(10).onTrue(Commands.runOnce(() -> CommandScheduler.getInstance().cancelAll()));

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
                var noBumper = leftBumper.or(rightBumper).negate();
                var hasAlgae = new Trigger(algaeArm::hasAlgae);

                var hasUpperTarget = new Trigger(this::getUpperTag);
                var hasLowerTarget = new Trigger(this::getLowerTag);
                var hasNoTarget = new Trigger(this::getNoTag);

                xboxA.and(leftBumper).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.l1, coralArm, elevatorSub,
                                                algaeArm)
                                                .alongWith(Commands.print("a button and left bumper pressed")));
                xboxB.and(leftBumper).onTrue(
                                new LeftAlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem,
                                                Constants.SetpointConstants.Options.l2));
                xboxX.and(leftBumper).onTrue(
                                new LeftAlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem,
                                                Constants.SetpointConstants.Options.l3));
                xboxY.and(leftBumper).onTrue(
                                new L4AlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem,
                                                Constants.SetpointConstants.StrafeOffsets.leftL4));
                xboxA.and(rightBumper).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.l1, coralArm, elevatorSub,
                                                algaeArm));
                xboxB.and(rightBumper).onTrue(
                                new RightAlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem,
                                                Constants.SetpointConstants.Options.l2));
                xboxX.and(rightBumper).onTrue(
                                new RightAlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem,
                                                Constants.SetpointConstants.Options.l3));
                xboxY.and(rightBumper).onTrue(
                                new L4AlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem,
                                                Constants.SetpointConstants.StrafeOffsets.rightL4));
                xboxA.and(noBumper).and(hasAlgae).and(hasUpperTarget).onTrue(
                                new ProcessorAlignmentSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                xboxA.and(noBumper).and(hasAlgae).and(hasUpperTarget.negate()).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.processor, coralArm, elevatorSub,
                                                algaeArm));
                xboxA.and(noBumper).and(hasAlgae.negate()).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub,
                                                algaeArm));
                xboxB.and(noBumper).and(xboxStart.negate()).onTrue(
                                new CoralStationSequence(coralArm, algaeArm, elevatorSub, swerveSubsystem));
                xboxB.and(noBumper).and(xboxStart).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.coralStationManual, coralArm,
                                                elevatorSub, algaeArm));
                xboxX.and(noBumper).onTrue(
                                new AlgaeIntakeAlignmentSequence(coralArm, elevatorSub, algaeArm, swerveSubsystem,
                                                Constants.SetpointConstants.Options.algaeLow));
                xboxY.and(noBumper).onTrue(
                                new AlgaeIntakeAlignmentSequence(coralArm, elevatorSub, algaeArm, swerveSubsystem,
                                                Constants.SetpointConstants.Options.algaeHigh));
                xboxStart.and(noBumper).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.algaeGround, coralArm, elevatorSub,
                                                algaeArm));

                // CommandScheduler.getInstance().onCommandInitialize((c) ->
                // System.out.println("initalizing: "+c.getName()));
                // CommandScheduler.getInstance().onCommandExecute((c) ->{
                // if (c.getName().equals("DefaultSwerve")) return;
                // System.out.println("executing: "+c.getName());
                // });
                // CommandScheduler.getInstance().onCommandFinish((c) ->
                // System.out.println("finishing: "+c.getName()));
                // CommandScheduler.getInstance().onCommandInterrupt((c,b) ->
                // System.out.println("finishing: "+c.getName() + " " + b));

                xboxBack.and(xboxA).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.driveConfig, coralArm, elevatorSub,
                                                algaeArm));
                xboxBack.and(xboxX).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.l3, coralArm, elevatorSub,
                                                algaeArm));
                xboxBack.and(xboxY).onTrue(
                                new ConfigSystem(Constants.SetpointConstants.Options.l4, coralArm, elevatorSub,
                                                algaeArm));
        }

        public class MyCommandShouldHaveAName extends SequentialCommandGroup {
                MyCommandShouldHaveAName(CoralArm coralArm, Elevator elevator, AlgaeArm algaeArm) {
                        addRequirements(algaeArm);
                        addRequirements(coralArm);
                        addRequirements(elevator);

                        addCommands(
                                        new ConfigSystem(Constants.SetpointConstants.Options.algaeGround, coralArm,
                                                        elevatorSub, algaeArm),
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

        public Command getAutonomousCommand() {
                // return Commands.print("No autonomous command configured");
                var command = autoChooser.getSelected();
                if (command != null) {
                        return command;
                } else {
                        return Commands.none();
                }
        }

        public void setStartingPose() {
                var poseChooserState = poseChooser.getSelected();
                var selectedStartingPose = poseChooserState == null ? StartingPlace.Center : poseChooserState;
                if (GoTo.getAlliance() == Alliance.Red) {
                        switch (selectedStartingPose) {
                                case Left:
                                        swerveSubsystem.resetOmetry(new Pose2d(9.5, 0.7, new Rotation2d(0.58)));
                                        break;
                                case Center:
                                        swerveSubsystem.resetOmetry(new Pose2d(9.5, 4, new Rotation2d()));
                                        break;
                                case Right:
                                        swerveSubsystem.resetOmetry(new Pose2d(9.5, 7.1, new Rotation2d(-0.58)));
                                        break;
                        }
                } else {
                        switch (selectedStartingPose) {
                                case Left:
                                        swerveSubsystem.resetOmetry(new Pose2d(8.1, 7.3, new Rotation2d(0.58)));
                                        break;
                                case Center:
                                        swerveSubsystem.resetOmetry(new Pose2d(8, 4, Rotation2d.k180deg));
                                        break;
                                case Right:
                                        swerveSubsystem.resetOmetry(new Pose2d(8, 0.7, new Rotation2d(-0.58)));
                                        break;
                        }
                }
        }
}
