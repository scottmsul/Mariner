package frc.robot.commands;

import java.util.Optional;

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
import edu.wpi.first.wpilibj2.command.Command;

public class GoTo {

    private int reefNTagID;
    private int reefNETagID;
    private int reefNWTagID;
    private int reefSTagID;
    private int reefSETagID;
    private int reefSWTagID;
    private int csLeftTagID;
    private int csRightTagID;
    private int processorTagID;

    Optional<Alliance> alliance = DriverStation.getAlliance();

    public GoTo() {
    }

    public void setAllianceIDs() {
        if (alliance.isPresent()) {
            if (alliance.get() == Alliance.Red) {
                reefNTagID = 10;
                reefNETagID = 9;
                reefNWTagID = 11;
                reefSTagID = 7;
                reefSETagID = 8;
                reefSWTagID = 6;
                csLeftTagID = 1;
                csRightTagID = 2;
                processorTagID = 3;
            } else if (alliance.get() == Alliance.Blue) {
                reefNTagID = 21;
                reefNETagID = 22;
                reefNWTagID = 20;
                reefSTagID = 18;
                reefSETagID = 17;
                reefSWTagID = 19;
                csLeftTagID = 13;
                csRightTagID = 12;
                processorTagID = 16;
            }
        } else {
            reefNTagID = 21;
            reefNETagID = 22;
            reefNWTagID = 20;
            reefSTagID = 18;
            reefSETagID = 17;
            reefSWTagID = 19;
            csLeftTagID = 13;
            csRightTagID = 12;
            processorTagID = 16;
        }
    }

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
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(reefNTagID), constraints);
    }

    public Command reefNE() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(reefNETagID), constraints);
    }

    public Command reefNW() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(reefNWTagID), constraints);
    }

    public Command reefS() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(reefSTagID), constraints);
    }

    public Command reefSE() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(reefSETagID), constraints);
    }

    public Command reefSW() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(reefSWTagID), constraints);
    }

    public Command coralStationLeft() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(csLeftTagID), constraints);
    }

    public Command coralStationRight() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(csRightTagID), constraints);
    }

    public Command processor() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(processorTagID), constraints);
    }

    public Command testTag8() {
        setAllianceIDs();
        return AutoBuilder.pathfindToPose(inFrontOfTag(8), constraints);
    }

}
