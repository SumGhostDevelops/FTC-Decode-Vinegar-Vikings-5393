package org.firstinspires.ftc.teamcode.util.dashboard;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

/**
 * Utility class for drawing the robot and field elements on FTC Dashboard's Field panel.
 * Designed for use with the Panels (bylazar) plugin.
 */
public class FieldDrawing
{
    private final FtcDashboard dashboard;

    // Robot dimensions in inches (adjust as needed)
    private static final double ROBOT_WIDTH = 18.0;
    private static final double ROBOT_LENGTH = 18.0;

    // Colors
    private static final String ROBOT_COLOR = "#3F51B5"; // Material Blue
    private static final String ROBOT_DIRECTION_COLOR = "#FF5722"; // Material Deep Orange
    private static final String TURRET_COLOR = "#4CAF50"; // Material Green
    private static final String TARGET_COLOR = "#F44336"; // Material Red

    public FieldDrawing()
    {
        this.dashboard = FtcDashboard.getInstance();
    }

    public FieldDrawing(FtcDashboard dashboard)
    {
        this.dashboard = dashboard;
    }

    /**
     * Draws the robot on the field at the given pose.
     * Uses FTC Standard coordinate system (center of field is origin).
     *
     * @param robotPose The robot's current pose
     */
    public void drawRobot(Pose2d robotPose)
    {
        drawRobot(robotPose, null, null);
    }

    /**
     * Draws the robot on the field with turret direction.
     *
     * @param robotPose The robot's current pose
     * @param turretAbsoluteHeading The turret's absolute heading on the field (or null to skip)
     */
    public void drawRobot(Pose2d robotPose, Double turretAbsoluteHeading)
    {
        drawRobot(robotPose, turretAbsoluteHeading, null);
    }

    /**
     * Draws the robot on the field with turret direction and target point.
     *
     * @param robotPose The robot's current pose
     * @param turretAbsoluteHeading The turret's absolute heading on the field in degrees (or null to skip)
     * @param targetCoord The target coordinate the turret is aiming at (or null to skip)
     */
    public void drawRobot(Pose2d robotPose, Double turretAbsoluteHeading, FieldCoordinate targetCoord)
    {
        TelemetryPacket packet = new TelemetryPacket();
        Canvas canvas = packet.fieldOverlay();

        // Convert pose to FTC Standard coordinates (center origin) in inches
        Pose2d ftcPose = robotPose
                .toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD)
                .toDistanceUnit(DistanceUnit.INCH);

        double x = ftcPose.coord.x.magnitude;
        double y = ftcPose.coord.y.magnitude;
        double headingRad = ftcPose.heading.getAngle(AngleUnit.RADIANS);

        // Draw robot body (rotated rectangle)
        canvas.setStroke(ROBOT_COLOR);
        canvas.setStrokeWidth(2);
        drawRotatedRectangle(canvas, x, y, ROBOT_WIDTH, ROBOT_LENGTH, headingRad);

        // Draw direction indicator (line from center pointing forward)
        double directionLength = ROBOT_LENGTH * 0.6;
        double dirX = x + directionLength * Math.cos(headingRad);
        double dirY = y + directionLength * Math.sin(headingRad);
        canvas.setStroke(ROBOT_DIRECTION_COLOR);
        canvas.setStrokeWidth(3);
        canvas.strokeLine(x, y, dirX, dirY);

        // Draw turret direction if provided
        if (turretAbsoluteHeading != null)
        {
            double turretRad = Math.toRadians(turretAbsoluteHeading);
            double turretLength = ROBOT_LENGTH * 0.8;
            double turretX = x + turretLength * Math.cos(turretRad);
            double turretY = y + turretLength * Math.sin(turretRad);
            canvas.setStroke(TURRET_COLOR);
            canvas.setStrokeWidth(2);
            canvas.strokeLine(x, y, turretX, turretY);

            // Draw small circle at turret end
            canvas.setFill(TURRET_COLOR);
            canvas.fillCircle(turretX, turretY, 2);
        }

        // Draw target point if provided
        if (targetCoord != null)
        {
            FieldCoordinate ftcTarget = targetCoord
                    .toCoordinateSystem(FieldCoordinate.CoordinateSystem.FTC_STD)
                    .toDistanceUnit(DistanceUnit.INCH);

            double targetX = ftcTarget.x.magnitude;
            double targetY = ftcTarget.y.magnitude;

            // Draw target marker (X shape)
            canvas.setStroke(TARGET_COLOR);
            canvas.setStrokeWidth(2);
            double markerSize = 4;
            canvas.strokeLine(targetX - markerSize, targetY - markerSize, targetX + markerSize, targetY + markerSize);
            canvas.strokeLine(targetX - markerSize, targetY + markerSize, targetX + markerSize, targetY - markerSize);

            // Draw line from robot to target
            canvas.setStroke(TARGET_COLOR);
            canvas.setStrokeWidth(1);
            canvas.strokeLine(x, y, targetX, targetY);
        }

        // Add telemetry data to packet
        packet.put("Robot X (in)", String.format("%.2f", x));
        packet.put("Robot Y (in)", String.format("%.2f", y));
        packet.put("Robot Heading (deg)", String.format("%.1f", Math.toDegrees(headingRad)));
        if (turretAbsoluteHeading != null)
        {
            packet.put("Turret Heading (deg)", String.format("%.1f", turretAbsoluteHeading));
        }

        dashboard.sendTelemetryPacket(packet);
    }

    /**
     * Draws a rotated rectangle on the canvas.
     */
    private void drawRotatedRectangle(Canvas canvas, double cx, double cy, double width, double height, double headingRad)
    {
        // Calculate corner offsets
        double halfW = width / 2;
        double halfH = height / 2;

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // Calculate the four corners
        double[] xCorners = new double[4];
        double[] yCorners = new double[4];

        // Front-left
        xCorners[0] = cx + halfH * cos - halfW * sin;
        yCorners[0] = cy + halfH * sin + halfW * cos;
        // Front-right
        xCorners[1] = cx + halfH * cos + halfW * sin;
        yCorners[1] = cy + halfH * sin - halfW * cos;
        // Back-right
        xCorners[2] = cx - halfH * cos + halfW * sin;
        yCorners[2] = cy - halfH * sin - halfW * cos;
        // Back-left
        xCorners[3] = cx - halfH * cos - halfW * sin;
        yCorners[3] = cy - halfH * sin + halfW * cos;

        // Draw the rectangle
        canvas.strokePolygon(xCorners, yCorners);
    }
}

