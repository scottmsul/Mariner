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
import frc.robot.Constants;

public class GoTo {

        private final static int redReefNTagID = 10;
        private final static int redReefNETagID = 9;
        private final static int redReefNWTagID = 11;
        private final static int redReefSTagID = 7;
        private final static int redReefSETagID = 8;
        private final static int redReefSWTagID = 6;
        private final static int redCsLeftTagID = 1;
        private final static int redCsRightTagID = 2;
        private final static int redProcessorTagID = 3;
        private final static int blueReefNTagID = 21;
        private final static int blueReefNETagID = 22;
        private final static int blueReefNWTagID = 20;
        private final static int blueReefSTagID = 18;
        private final static int blueReefSETagID = 17;
        private final static int blueReefSWTagID = 19;
        private final static int blueCsLeftTagID = 13;
        private final static int blueCsRightTagID = 12;
        private final static int blueProcessorTagID = 16;

        public static boolean isRed() {
                return getAlliance() == Alliance.Red;
        }

        public static Alliance getAlliance() {
                System.out.println("returning alliance: " + DriverStation.getAlliance());
                return DriverStation.getAlliance().orElse(Alliance.Blue);
        }

        public static PathConstraints constraints = new PathConstraints(3.5, 3, 360 * 1.5, 360);

        private static Pose2d inFrontOfTag(int id) {
                Transform2d rot180 = new Transform2d(Translation2d.kZero, Rotation2d.k180deg);
                var tag = Constants.kField.getTagPose(id).get().toPose2d();
                var offset = new Transform2d(1.5, 0, new Rotation2d());
                Pose2d infrontOfTag = tag.plus(offset).transformBy(rot180);
                return infrontOfTag;
        }

        public static Command reefN() {
                // setAllianceIDs();
                // return AutoBuilder.pathfindToPose(inFrontOfTag(reefNTagID), constraints);
                // var redReefNTagID = 1;
                // var blueReefNTagID = 2;
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redReefNTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID" + redReefNTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefNTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID" + blueReefNTagID)),
                                GoTo::isRed);
        }

        public static Command reefNE() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redReefNETagID), constraints)
                                                .alongWith(Commands.print("going to tag ID" + redReefNETagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefNETagID), constraints)
                                                .alongWith(Commands.print("going to tag ID" + blueReefNETagID)),
                                GoTo::isRed);
        }

        public static Command reefNW() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redReefNWTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID" + redReefNWTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefNWTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID" + blueReefNWTagID)),
                                GoTo::isRed);
        }

        public static Command reefS() {
                // setAllianceIDs();
                // return AutoBuilder.pathfindToPose(inFrontOfTag(reefSTagID), constraints);
                // var redReefSTagID = 1;
                // var blueReefSTagID = 2;
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redReefSTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + redReefSTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefSTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + blueReefSTagID)),
                                GoTo::isRed);
        }

        public static Command reefSE() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redReefSETagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + redReefSETagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefSETagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + blueReefSETagID)),
                                GoTo::isRed);
        }

        public static Command reefSW() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redReefSWTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + redReefSWTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueReefSWTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + blueReefSWTagID)),
                                GoTo::isRed);
        }

        public static Command coralStationLeft() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redCsLeftTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + redCsLeftTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueCsLeftTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + blueCsLeftTagID)),
                                GoTo::isRed);
        }

        public static Command coralStationRight() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redCsRightTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + redCsRightTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueCsRightTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + blueCsRightTagID)),
                                GoTo::isRed);
        }

        public static Command processor() {
                return Commands.either(
                                AutoBuilder.pathfindToPose(inFrontOfTag(redProcessorTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + redProcessorTagID)),
                                AutoBuilder.pathfindToPose(inFrontOfTag(blueProcessorTagID), constraints)
                                                .alongWith(Commands.print("going to tag ID " + blueProcessorTagID)),
                                GoTo::isRed);
        }

        public Command testTag8() {
                return AutoBuilder.pathfindToPose(inFrontOfTag(8), constraints);
        }

}
