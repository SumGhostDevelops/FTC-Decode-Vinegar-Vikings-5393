package org.firstinspires.ftc.teamcode.util.measure.coordinate;

public class FuturePose
{
        /*
         * futurePose = currentPose // vector (x, y)
         * repeat 3 times:
         * distance = ||targetPos - futurePose|| // Euclidean distance
         * flightTime = LUT(distance) // or physics formula
         * newFuturePose = currentPose + robotVelocity * flightTime
         * futurePose = (futurePose + newFuturePose) / 2 // optional averaging
         * 
         * //all of the following will probably be outside the function, just wanted to
         * put it somewhere
         * distance = ||targetPos - futurePose|| // final distance
         * turretAngle = atan2(target_y - future_y, target_x - future_x)
         * flywheelRPM = LUT(distance)
         */

}
