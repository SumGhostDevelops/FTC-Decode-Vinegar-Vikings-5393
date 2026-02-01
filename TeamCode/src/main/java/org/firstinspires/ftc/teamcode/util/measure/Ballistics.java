package org.firstinspires.ftc.teamcode.util.measure;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.math.BetterInterpLUT;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FuturePose;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FuturePose.FuturePoseResult;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

public class Ballistics
{
    private static final BetterInterpLUT flightTimeLUT;

    static
    {
        BetterInterpLUT.Builder builder = BetterInterpLUT.builder();
        double[] dists = RobotConstants.Ballistics.DISTANCES;
        double[] times = RobotConstants.Ballistics.FLIGHT_TIMES;

        for (int i = 0; i < dists.length && i < times.length; i++)
        {
            builder.add(dists[i], times[i]);
        }
        flightTimeLUT = builder.build();
    }

    /**
     * Calculates the future pose result based on the current robot state and
     * target.
     */
    public static FuturePoseResult calculate(FieldCoordinate target, Odometry odometry)
    {
        return FuturePose.solve(
                odometry.getPose(),
                new Pose2d(target, odometry.getPose().heading), // Target heading doesn't matter for distance calc
                odometry.getVelocityTracker(),
                (dist) -> flightTimeLUT.get(dist.getInch()),
                RobotConstants.Ballistics.SYSTEM_LATENCY);
    }
}
