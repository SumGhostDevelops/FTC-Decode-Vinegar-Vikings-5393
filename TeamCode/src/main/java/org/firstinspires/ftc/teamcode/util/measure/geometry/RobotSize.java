package org.firstinspires.ftc.teamcode.util.measure.geometry;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;
import org.firstinspires.ftc.teamcode.util.measure.distance.Distance;

/**
 * Useful class for getting centered positions on the field based on a robot's size.
 */
public class RobotSize
{
    public final Distance forwardLength;
    public final Distance sidewaysWidth;

    // Pre-calculated half dimensions to prevent repetitive division
    public final Distance halfLength;
    public final Distance halfWidth;

    public enum Corner
    {
        TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT
    }

    public enum Side
    {
        LEFT, RIGHT, FRONT, BACK
    }

    public RobotSize(double forwardLengthInches, double sidewaysWidthInches)
    {
        this(
                new Distance(forwardLengthInches, DistanceUnit.INCH),
                new Distance(sidewaysWidthInches, DistanceUnit.INCH)
        );
    }

    public RobotSize(Distance forwardLength, Distance sidewaysWidth)
    {
        this.forwardLength = forwardLength;
        this.sidewaysWidth = sidewaysWidth;
        this.halfLength = forwardLength.divide(2.0);
        this.halfWidth = sidewaysWidth.divide(2.0);
    }

    /**
     * Prepares to put the corner on the field coordinate.
     * @param corner The part of the robot to place at the coordinate.
     * @param fieldCoordinate The target coordinate.
     * @return A builder to finalize the orientation.
     */
    public CoordinateInProgress put(Corner corner, FieldCoordinate fieldCoordinate)
    {
        Distance x = Distance.ZERO;
        Distance y = Distance.ZERO;

        // Assuming Top is Forward (+X) and Left is Positive Strafe (+Y)
        switch (corner) {
            case TOP_LEFT:
                x = halfLength; y = halfWidth; break;
            case TOP_RIGHT:
                x = halfLength; y = halfWidth.negative(); break;
            case BOTTOM_LEFT:
                x = halfLength.negative(); y = halfWidth; break;
            case BOTTOM_RIGHT:
                x = halfLength.negative(); y = halfWidth.negative(); break;
        }
        return new CoordinateInProgress(x, y, fieldCoordinate);
    }

    /**
     * Prepares to put the center of this side on the field coordinate.
     * @param side The part of the robot to place at the coordinate.
     * @param fieldCoordinate The target coordinate.
     * @return A builder to finalize the orientation.
     */
    public CoordinateInProgress put(Side side, FieldCoordinate fieldCoordinate)
    {
        Distance x = Distance.ZERO;
        Distance y = Distance.ZERO;

        switch (side) {
            case FRONT:
                x = halfLength; break;
            case BACK:
                x = halfLength.negative(); break;
            case LEFT:
                y = halfWidth; break;
            case RIGHT:
                y = halfWidth.negative(); break;
        }
        return new CoordinateInProgress(x, y, fieldCoordinate);
    }

    /**
     * Calculates the radius of the circle that perfectly circumscribes the robot.
     * Useful for collision avoidance logic and pathing constraints.
     * @return The diagonal radius of the robot.
     */
    public Distance getCircumscribedRadius()
    {
        double l = halfLength.getInch();
        double w = halfWidth.getInch();
        return new Distance(Math.sqrt(l * l + w * w), DistanceUnit.INCH);
    }

    public class CoordinateInProgress
    {
        private final Distance localOffsetX;
        private final Distance localOffsetY;
        private final FieldCoordinate targetPoint;

        public CoordinateInProgress(Distance localOffsetX, Distance localOffsetY, FieldCoordinate targetPoint)
        {
            this.localOffsetX = localOffsetX;
            this.localOffsetY = localOffsetY;
            this.targetPoint = targetPoint;
        }

        /**
         * Alters the field coordinate for the robot to facing the field heading.
         * @param fieldHeading The heading the robot will be facing.
         * @return The final centered Pose2d of the robot.
         */
        public Pose2d facing(FieldHeading fieldHeading)
        {
            Pose2d edgePose = new Pose2d(targetPoint, fieldHeading);

            // Shift the center of the robot backward from the targeted edge/corner
            // by translating it by the negative offsets.
            return edgePose.transform(localOffsetX.negative(), localOffsetY.negative());
        }

        /**
         * Orients the robot to the default heading (90 degrees PedroPath).
         * @return The final centered Pose2d of the robot.
         */
        public Pose2d defaultFacing()
        {
            return facing(new FieldHeading(
                    new Angle(90, AngleUnit.DEGREES),
                    CoordinateSystem.DECODE_PEDROPATH
            ));
        }

        /**
         * Pushes the robot's placement away from the target edge by an explicit clearance distance.
         */
        public CoordinateInProgress addClearance(Distance forwardClearance, Distance strafeClearance)
        {
            return new CoordinateInProgress(
                    localOffsetX.plus(forwardClearance),
                    localOffsetY.plus(strafeClearance),
                    targetPoint
            );
        }

        /**
         * Pushes the robot away from the target by a multiplier of its own total dimensions.
         * e.g., scaleClearance(1.0, 0.0) leaves an entire robot length between the corner/side and the target.
         */
        public CoordinateInProgress scaleClearance(double lengthMultiplier, double widthMultiplier)
        {
            return new CoordinateInProgress(
                    localOffsetX.plus(forwardLength.multiply(lengthMultiplier)),
                    localOffsetY.plus(sidewaysWidth.multiply(widthMultiplier)),
                    targetPoint
            );
        }
    }
}