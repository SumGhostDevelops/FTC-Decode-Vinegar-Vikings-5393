package org.firstinspires.ftc.teamcode.util.measure.coordinate;

public class FuturePose {
    /*
    futurePose = currentPose   // vector (x, y)
    repeat 3 times:
    distance = ||targetPos - futurePose||         // Euclidean distance
    flightTime = LUT(distance)                     // or physics formula
    newFuturePose = currentPose + robotVelocity * flightTime
            futurePose = (futurePose + newFuturePose) / 2 // optional averaging

    distance = ||targetPos - futurePose||             // final distance
    turretAngle = atan2(target_y - future_y, target_x - future_x)
    flywheelRPM = LUT(distance)
     */

}
