package frc.robot.commands;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class GoTo {

    private int redReefNTagID = 10;
    private int redReefNETagID = 9;
    private int redReefNWTagID = 11;
    private int redReefSTagID = 7;
    private int redReefSETagID = 8;
    private int redReefSWTagID = 6;
    private int redCsLeftTagID = 1;
    private int redCsRightTagID = 2;
    private int redProcessorTagID = 3;
    private int blueReefNTagID = 21;
    private int blueReefNETagID = 22;
    private int blueReefNWTagID = 20;
    private int blueReefSTagID = 18;
    private int blueReefSETagID = 17;
    private int blueReefSWTagID = 19;
    private int blueCsLeftTagID = 13;
    private int blueCsRightTagID = 12;
    private int blueProcessorTagID = 16;

    public boolean isRed() {
        return getAlliance() == Alliance.Red;
    }

    public static Alliance getAlliance() {
        System.out.println("returning alliance: " + DriverStation.getAlliance());
        return DriverStation.getAlliance().orElse(Alliance.Blue);
    }

    public GoTo(Optional<Alliance> alliance) {
        // alliance not used, delete
    }

    // public void setAllianceIDs() {
    // if (true) {
    // if (getAlliance() == Alliance.Red) {
    // reefNTagID = 10;
    // reefNETagID = 9;
    // reefNWTagID = 11;
    // reefSTagID = 7;
    // reefSETagID = 8;
    // reefSWTagID = 6;
    // csLeftTagID = 1;
    // csRightTagID = 2;
    // processorTagID = 3;
    // } else if (getAlliance() == Alliance.Blue) {
    // reefNTagID = 21;
    // reefNETagID = 22;
    // reefNWTagID = 20;
    // reefSTagID = 18;
    // reefSETagID = 17;
    // reefSWTagID = 19;
    // csLeftTagID = 13;
    // csRightTagID = 12;
    // processorTagID = 16;
    // }
    // } else {
    // reefNTagID = 21;
    // reefNETagID = 22;
    // reefNWTagID = 20;
    // reefSTagID = 18;
    // reefSETagID = 17;
    // reefSWTagID = 19;
    // csLeftTagID = 13;
    // csRightTagID = 12;
    // processorTagID = 16;
    // }
    // switch (side.getSelected()) {
    // case Red:
    // reefNTagID = 10;
    // reefNETagID = 9;
    // reefNWTagID = 11;
    // reefSTagID = 7;
    // reefSETagID = 8;
    // reefSWTagID = 6;
    // csLeftTagID = 1;
    // csRightTagID = 2;
    // processorTagID = 3;
    // break;

    // case Blue:
    // reefNTagID = 21;
    // reefNETagID = 22;
    // reefNWTagID = 20;
    // reefSTagID = 18;
    // reefSETagID = 17;
    // reefSWTagID = 19;
    // csLeftTagID = 13;
    // csRightTagID = 12;
    // processorTagID = 16;
    // break;

    // default:
    // reefNTagID = 21;
    // reefNETagID = 22;
    // reefNWTagID = 20;
    // reefSTagID = 18;
    // reefSETagID = 17;
    // reefSWTagID = 19;
    // csLeftTagID = 13;
    // csRightTagID = 12;
    // processorTagID = 16;
    // break;
    // }
    // }

    public PathConstraints constraints = new PathConstraints(1.75, 2, 360, 360);

    private Pose2d inFrontOfTag(int id) {
        Transform2d rot180 = new Transform2d(Translation2d.kZero, Rotation2d.k180deg);
        var field = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        var tag = field.getTagPose(id).get().toPose2d();
        var offset = new Transform2d(1, 0, new Rotation2d());
        Pose2d infrontOfTag = tag.plus(offset).transformBy(rot180);
        return infrontOfTag;
    }

    public Command reefN() {
        // setAllianceIDs();
        // return AutoBuilder.pathfindToPose(inFrontOfTag(reefNTagID), constraints);
        // var redReefNTagID = 1;
        // var blueReefNTagID = 2;
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redReefNTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefNTagID), constraints),
                this::isRed);
    }

    public Command reefNE() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redReefNETagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefNETagID), constraints),
                this::isRed);
    }

    public Command reefNW() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redReefNWTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefNWTagID), constraints),
                this::isRed);
    }

    public Command reefS() {
        // setAllianceIDs();
        // return AutoBuilder.pathfindToPose(inFrontOfTag(reefSTagID), constraints);
        // var redReefSTagID = 1;
        // var blueReefSTagID = 2;
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redReefSTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefSTagID), constraints),
                this::isRed);
    }

    public Command reefSE() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redReefSETagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefSETagID), constraints),
                this::isRed);
    }

    public Command reefSW() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redReefSWTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefSWTagID), constraints),
                this::isRed);
    }

    public Command coralStationLeft() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redCsLeftTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueCsLeftTagID), constraints),
                this::isRed);
    }

    public Command coralStationRight() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redCsRightTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueCsRightTagID), constraints),
                this::isRed);
    }

    public Command processor() {
        return Commands.either(
                AutoBuilder.pathfindToPose(inFrontOfTag(redProcessorTagID), constraints),
                AutoBuilder.pathfindToPose(inFrontOfTag(blueProcessorTagID), constraints),
                this::isRed);
    }

    // public Command testTag8() {
    // setAllianceIDs();
    // return AutoBuilder.pathfindToPose(inFrontOfTag(8), constraints);
    // }

}
