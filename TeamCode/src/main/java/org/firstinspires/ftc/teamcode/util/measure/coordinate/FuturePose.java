package org.firstinspires.ftc.teamcode.util.measure.coordinate;

import org.firstinspires.ftc.teamcode.util.kinematics.PoseVelocityTracker;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;
import org.firstinspires.ftc.teamcode.util.measure.geometry.Vector2d;

import java.util.function.Function;

public class FuturePose
{
        /**
         * Data class containing the results of the predictive solver.
         */
        public static class FuturePoseResult
        {
                /**
                 * The predicted field-absolute pose of the robot at the moment of intercept.
                 * Use this for aiming the turret.
                 */
                public final Pose2d pose;

                /**
                 * The total estimated time (seconds) from the current moment until intercept.
                 * This includes both constant system latency and distance-dependent flight
                 * time.
                 */
                public final double totalTime;

                /**
                 * The predicted straight-line distance between the robot and target at
                 * intercept.
                 * Use this to look up flywheel RPMs or vertical outtake angles.
                 */
                public final Distance distance;

                /**
                 * The predicted velocity of the robot at the moment of launch (now + latency).
                 * Use this to counteract the robot's motion on the projectile's trajectory.
                 */
                public final Vector2d predictedVelocity;

                public FuturePoseResult(Pose2d pose, double totalTime, Distance distance, Vector2d predictedVelocity)
                {
                        this.pose = pose;
                        this.totalTime = totalTime;
                        this.distance = distance;
                        this.predictedVelocity = predictedVelocity;
                }
        }

        /**
         * Solves for the robot's pose at the moment a projectile would hit the target.
         * 
         * @param currentPose
         *                The robot's current pose.
         * @param targetPose
         *                The field-absolute target pose.
         * @param tracker
         *                The kinematic tracker providing velocity and acceleration.
         * @param flightTimeLUT
         *                A function that returns projectile flight time for a
         *                given distance.
         * @param systemLatency
         *                Constant overhead (seconds) for processing/hardware
         *                delay.
         * @return A FuturePoseResult containing the predicted intercept state.
         */
        public static FuturePoseResult solve(
                        Pose2d currentPose,
                        Pose2d targetPose,
                        PoseVelocityTracker tracker,
                        Function<Distance, Double> flightTimeLUT,
                        double systemLatency)
        {
                Distance currentDistance = currentPose.distanceTo(targetPose);
                double estimatedTime = systemLatency + flightTimeLUT.apply(currentDistance);

                Pose2d futurePose = currentPose;
                Distance futureDistance = currentDistance;

                // 3 iterations with damping as requested
                for (int i = 0; i < 3; i++)
                {
                        Pose2d newFuturePose = tracker.getFuturePose(estimatedTime, currentPose);

                        // Apply damping: average the previous guess and the new guess
                        // futurePose = (futurePose + newFuturePose) / 2
                        futurePose = averagePoses(futurePose, newFuturePose);

                        futureDistance = futurePose.distanceTo(targetPose);
                        estimatedTime = systemLatency + flightTimeLUT.apply(futureDistance);
                }

                return new FuturePoseResult(futurePose, estimatedTime, futureDistance, tracker.getFutureVelocity(systemLatency));
        }

        private static Pose2d averagePoses(Pose2d a, Pose2d b)
        {
                // Ensure we are working in the same coordinate system
                Pose2d bConverted = b.toCoordinateSystem(a.coord.coordSys);

                // Average coordinates
                double avgX = (a.coord.x.magnitude + bConverted.coord.x.magnitude) / 2.0;
                double avgY = (a.coord.y.magnitude + bConverted.coord.y.magnitude) / 2.0;

                // Average heading (simple average is sufficient for prediction convergence
                // where differences are small)
                // For more robustness across wrap-around, we could use vector averaging, but
                // likely unnecessary here.
                double avgH = (a.heading.angle.measure + bConverted.heading.angle.measure) / 2.0;

                return new Pose2d(
                                new FieldCoordinate(new Distance(avgX, a.coord.x.unit), new Distance(avgY, a.coord.y.unit), a.coord.coordSys),
                                new FieldHeading(new Angle(avgH, a.heading.angle.unit), a.coord.coordSys));
        }
}
