package frc.robot.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;

public class TrajectoryHelpers {

    public static Pose2d correctedPose = new Pose2d();

    /**
     * This method will adjust the endPose based on the angle.
     * For now it will siply turn it to the specified angle using the startPose as origin.
     * This will likely be sufficient for small trajectory deviations during auto-driving to pickup notes.
     * Note that Rotation2d of the final pose will change.
     * In other words, this routine willl turn endPose by angle using startPose as origin
     */
    public static Pose2d correctEndingPoseBasedOnNoteLocation(Pose2d startPose, Pose2d endPose, double angle) {
        Pose2d relativePoseTurned = endPose.relativeTo(startPose).rotateBy(Rotation2d.fromDegrees(angle));  //NEW POSE BASED ON ANGLE RELATIVE 2 STARTING POSE
        Transform2d t = relativePoseTurned.minus(new Pose2d()); // converts relative pose to a transformation
        return startPose.transformBy(t);
        
    }

    // get corrected X of the pose when turning the endpose using startpose as origin
    // Used to get part of the Transform
    public static double getCorrectedX(Pose2d startPose, Pose2d endPose, double angle) {
        return correctEndingPoseBasedOnNoteLocation(startPose, endPose, angle).getX();
    }

    // get corrected X of the pose when turning the endpose using startpose as origin
    // Used to get part of the Transform
    public static double getCorrectedY(Pose2d startPose, Pose2d endPose, double angle) {
        return correctEndingPoseBasedOnNoteLocation(startPose, endPose, angle).getY();
    }


    public static void setCorrectedPose(Pose2d startPose, Pose2d endPose, double angle) {
        correctedPose = correctEndingPoseBasedOnNoteLocation(startPose, endPose, angle);
    }

    public static Pose2d getCorrectedPose() {
        return correctedPose;
    }

    // How much do I need to rotate from current pose to point to the second pose
    public static Rotation2d rotateToPointToSecondPose(Pose2d currentPose, Pose2d secondPose) {
        return secondPose.relativeTo(currentPose).getTranslation().getAngle();
    }

    public static boolean isValueBetween(double v, double min, double max) {
        return v >= min && v <= max;
    }

}
