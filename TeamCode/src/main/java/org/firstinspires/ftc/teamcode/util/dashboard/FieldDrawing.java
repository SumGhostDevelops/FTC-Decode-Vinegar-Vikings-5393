package org.firstinspires.ftc.teamcode.util.dashboard;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

/**
 * Utility class for drawing the robot and field elements on Panels Dashboard's Field panel.
 * Designed for use with the Panels (bylazar) plugin.
 *
 * @author Vinegar Vikings - 5393
 * @version 2.0
 */
public class FieldDrawing
{
    // Robot radius in inches
    public static final double ROBOT_RADIUS = 9.0;

    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    // Styles - Using stroke width for visibility
    private static final Style robotStyle = new Style(
            "#3F51B5", "#3F51B5", 2.0  // Material Blue, filled with stroke
    );
    private static final Style directionStyle = new Style(
            "#FF5722", "#FF5722", 3.0  // Material Deep Orange, thicker stroke
    );
    private static final Style turretStyle = new Style(
            "#4CAF50", "#4CAF50", 3.0  // Material Green, thicker stroke
    );
    private static final Style targetStyle = new Style(
            "#F44336", "#F44336", 2.0  // Material Red
    );
    private static final Style targetMarkerStyle = new Style(
            "#F44336", "#F44336", 2.0  // Material Red for target marker
    );

    /**
     * Prepares Panels Field for using FTC Standard coordinate offsets.
     * Call this in your OpMode init.
     */
    public static void init()
    {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * Draws the robot on the field at the given pose.
     * Uses FTC Standard coordinate system (center of field is origin).
     *
     * @param robotPose The robot's current pose
     */
    public static void drawRobot(Pose2d robotPose)
    {
        drawRobot(robotPose, null, null);
    }

    /**
     * Draws the robot on the field with turret direction.
     *
     * @param robotPose             The robot's current pose
     * @param turretAbsoluteHeading The turret's absolute heading on the field (or null to skip)
     */
    public static void drawRobot(Pose2d robotPose, Double turretAbsoluteHeading)
    {
        drawRobot(robotPose, turretAbsoluteHeading, null);
    }

    /**
     * Draws the robot on the field with turret direction and target point.
     *
     * @param robotPose             The robot's current pose
     * @param turretAbsoluteHeading The turret's absolute heading on the field in degrees (or null to skip)
     * @param targetCoord           The target coordinate the turret is aiming at (or null to skip)
     */
    public static void drawRobot(Pose2d robotPose, Double turretAbsoluteHeading, FieldCoordinate targetCoord)
    {
        if (robotPose == null)
        {
            return;
        }

        // Convert pose to RIGHT_HAND coordinates (corner origin) in inches
        // This matches the PEDRO_PATHING preset which expects (0,0) at corner
        Pose2d convertedPose = robotPose
                .toCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND)
                .toDistanceUnit(DistanceUnit.INCH);

        double x = convertedPose.coord.x.magnitude;
        double y = convertedPose.coord.y.magnitude;
        double headingRad = convertedPose.heading.getAngle(AngleUnit.RADIANS);

        if (Double.isNaN(x) || Double.isNaN(y) || Double.isNaN(headingRad))
        {
            return;
        }

        // Draw robot body (filled circle)
        panelsField.setStyle(robotStyle);
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS);

        // Draw a smaller inner circle to make it more visible
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS * 0.6);

        // Draw direction indicator (line from center extending past the robot edge)
        double dirX1 = x;
        double dirY1 = y;
        double dirX2 = x + (ROBOT_RADIUS * 1.5) * Math.cos(headingRad);
        double dirY2 = y + (ROBOT_RADIUS * 1.5) * Math.sin(headingRad);

        panelsField.setStyle(directionStyle);
        panelsField.moveCursor(dirX1, dirY1);
        panelsField.line(dirX2, dirY2);

        // Draw turret direction if provided (extends past the robot edge)
        if (turretAbsoluteHeading != null)
        {
            double turretRad = Math.toRadians(turretAbsoluteHeading);
            double turretX1 = x;
            double turretY1 = y;
            double turretX2 = x + (ROBOT_RADIUS * 1.8) * Math.cos(turretRad);
            double turretY2 = y + (ROBOT_RADIUS * 1.8) * Math.sin(turretRad);

            panelsField.setStyle(turretStyle);
            panelsField.moveCursor(turretX1, turretY1);
            panelsField.line(turretX2, turretY2);

            // Draw a small circle at the turret end for visibility
            panelsField.moveCursor(turretX2, turretY2);
            panelsField.circle(2);
        }

        // Draw target point if provided
        if (targetCoord != null)
        {
            FieldCoordinate convertedTarget = targetCoord
                    .toCoordinateSystem(FieldCoordinate.CoordinateSystem.RIGHT_HAND)
                    .toDistanceUnit(DistanceUnit.INCH);

            double targetX = convertedTarget.x.magnitude;
            double targetY = convertedTarget.y.magnitude;

            // Draw target marker (larger circle for visibility)
            panelsField.setStyle(targetMarkerStyle);
            panelsField.moveCursor(targetX, targetY);
            panelsField.circle(6);

            // Draw an X through the target
            panelsField.setStyle(targetStyle);
            double markerSize = 4;
            panelsField.moveCursor(targetX - markerSize, targetY - markerSize);
            panelsField.line(targetX + markerSize, targetY + markerSize);
            panelsField.moveCursor(targetX - markerSize, targetY + markerSize);
            panelsField.line(targetX + markerSize, targetY - markerSize);
        }
    }

    /**
     * Sends the current packet to Panels Dashboard.
     * Call this after drawing all elements for the current frame.
     */
    public static void sendPacket()
    {
        panelsField.update();
    }
}

