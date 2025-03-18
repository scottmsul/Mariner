package frc.robot.commands.autos;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class CoralVision {

    public CoralVision() {

    }

    // aprilTag = limelightFront get target
    // tagi = aprilTag pose get x
    // tagj = aprilTag pose get y
    // branch translations
    // leftL4 = (i, j)
    // rightL4 = (i, j)
    // leftL3 = (i, j)
    // rightL3 = (i, j)
    // leftL2 = (i, j)
    // rightL2 = (i, j)

    static final Optional<Pose3d> getAprilTagPose() {
        if (LimelightHelpers.getTV(Constants.ReefLimelightName)) {
            return Optional.of(LimelightHelpers.getTargetPose3d_RobotSpace(Constants.ReefLimelightName));
        } else {
            return Optional.empty();
        }
    }

    private boolean checkPoseThreshold(Pose3d branchPose, Pose3d targetPose) {
        if ((Math.abs(targetPose.getX() - branchPose.getX()) < 0.2)
                && (Math.abs(targetPose.getY() - branchPose.getY()) < 0.2)) {
            return true;
        } else {
            return false;
        }
    }

    public double[] GetAvailabilities() {

        double[] availabilities = new double[6];
        var tag = getAprilTagPose();
        if (tag.isEmpty()) {
            return availabilities;
        }
        var tagPose = tag.get();

        var llResults = LimelightHelpers.getLatestResults("").targets_Detector;

        Transform3d l4LeftTranslation = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        Transform3d l4RightTranslation = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        Transform3d l3LeftTranslation = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        Transform3d l3RightTranslation = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        Transform3d l2LeftTranslation = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());
        Transform3d l2RightTranslation = new Transform3d(0.0, 0.0, 0.0, new Rotation3d());

        Pose3d l4LeftPose = tagPose.transformBy(l4LeftTranslation);
        Pose3d l4RightPose = tagPose.transformBy(l4RightTranslation);
        Pose3d l3LeftPose = tagPose.transformBy(l3LeftTranslation);
        Pose3d l3RightPose = tagPose.transformBy(l3RightTranslation);
        Pose3d l2LeftPose = tagPose.transformBy(l2LeftTranslation);
        Pose3d l2RightPose = tagPose.transformBy(l2RightTranslation);

        // need to get each pose in camera pixel place
        // then compare the two in the threshold instead of the poses :(((((((((((

        // for(int index = 0; index < llResults.length; index++){
        // var targetPose = llResults[index];
        // targetPose.
        // }
        // if (checkPoseThreshold(l4LeftPose, targetPose)){
        // availabilities[0] = 1.0;
        // }
        // else if (checkPoseThreshold(l4RightPose, targetPose){
        // availabilities[1] = 1.0;
        // }
        // else if (checkPoseThreshold(l4RightPose, targetPose){
        // availabilities[2] = 1.0;
        // }
        // else if (checkPoseThreshold(l4RightPose, targetPose){
        // availabilities[3] = 1.0;
        // }
        // else if (checkPoseThreshold(l4RightPose, targetPose){
        // availabilities[4] = 1.0;
        // }
        // else if (checkPoseThreshold(l4RightPose, targetPose){
        // availabilities[5] = 1.0;
        // }

        return availabilities;

    }

}
