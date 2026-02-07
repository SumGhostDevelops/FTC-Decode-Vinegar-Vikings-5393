package org.firstinspires.ftc.teamcode.util.dashboard;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.measure.angle.field.FieldHeading;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.CoordinateSystem;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.FieldCoordinate;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

import java.util.function.Supplier;

/**
 * Utility class for drawing the robot and field elements on Panels Dashboard's Field panel.
 * Designed for use with the Panels (bylazar) plugin.
 *
 * @author Vinegar Vikings - 5393
 * @version 2.1 - Updated for FieldHeading
 */
public class FieldDrawing
{
    public static final double ROBOT_RADIUS = 9.0;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    // Styles - Using stroke width for visibility
    private static final Style robotStyle = new Style(
            "#3F51B5", "#3F51B5", 2.0  // Material Blue, filled with stroke
    );
    private static final Style futurePoseStyle = new Style(
            "2DE6AE", "2DE6AE", 2.0
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

    /**
     * Prepares Panels Field for using FTC Standard coordinate offsets.
     * Call this in your OpMode init.
     */
    public static void init()
    {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    public static void draw(Supplier<Pose2d> pose, Supplier<Pose2d> futurePose, Supplier<FieldHeading> turretHeading, Supplier<FieldCoordinate> targetCoord)
    {
        draw(pose.get(), futurePose.get(), turretHeading.get(), targetCoord.get());
    }

    /**
     * Draws a robot, heading, optional turret, and optional target using explicit styles.
     *
     * @param pose           The robot pose
     * @param futurePose     The target/future robot pose (ghost), or null
     * @param turretHeading  Turret absolute heading, or null to skip
     * @param targetCoord    Target field coordinate (or null to skip)
     */
    public static void draw(Pose2d pose, Pose2d futurePose, FieldHeading turretHeading, FieldCoordinate targetCoord)
    {
        if (pose == null) return;
        drawRobotBody(pose, robotStyle);
        drawRobotHeading(pose, directionStyle);

        if (futurePose != null)
        {
            drawRobotBody(futurePose, futurePoseStyle); // Fixed style usage
            drawRobotHeading(futurePose, directionStyle);
        }

        if (turretHeading != null)
        {
            // Convert everything to Pedro pathing system for drawing consistency
            Pose2d robotPedro = pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH);
            FieldHeading turretPedro = turretHeading.toSystem(CoordinateSystem.DECODE_PEDROPATH);

            double turretRad = turretPedro.angle.getRadians();

            // Draw line representing turret barrel
            drawAngle(pose, turretPedro, ROBOT_RADIUS * 1.8, turretStyle);

            // Draw a small circle at the turret end (muzzle)
            double x = robotPedro.coord.x.magnitude;
            double y = robotPedro.coord.y.magnitude;
            double x2 = x + (ROBOT_RADIUS * 1.8) * Math.cos(turretRad);
            double y2 = y + (ROBOT_RADIUS * 1.8) * Math.sin(turretRad);

            panelsField.setStyle(turretStyle);
            panelsField.moveCursor(x2, y2);
            panelsField.circle(2);
        }

        if (targetCoord != null)
        {
            drawFieldCoordinate(targetCoord, targetStyle, 6);
            // Draw an X through the target
            FieldCoordinate convertedTarget = targetCoord.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH);
            double targetX = convertedTarget.x.magnitude;
            double targetY = convertedTarget.y.magnitude;
            double markerSize = 4;

            panelsField.setStyle(targetStyle);
            panelsField.moveCursor(targetX - markerSize, targetY - markerSize);
            panelsField.line(targetX + markerSize, targetY + markerSize);
            panelsField.moveCursor(targetX - markerSize, targetY + markerSize);
            panelsField.line(targetX + markerSize, targetY - markerSize);
        }
    }

    /**
     * Draws the robot body (circle) at the given pose.
     */
    public static void drawRobotBody(Pose2d pose, Style style)
    {
        if (pose == null || style == null) return;
        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH);
        double x = converted.coord.x.magnitude;
        double y = converted.coord.y.magnitude;
        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS);
        panelsField.moveCursor(x, y);
        panelsField.circle(ROBOT_RADIUS * 0.6);
    }

    /**
     * Draws the robot heading (line) at the given pose.
     */
    public static void drawRobotHeading(Pose2d pose, Style style)
    {
        if (pose == null || style == null) return;
        Pose2d converted = pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH);

        double x = converted.coord.x.magnitude;
        double y = converted.coord.y.magnitude;
        double headingRad = converted.heading.angle.getRadians();

        double x2 = x + (ROBOT_RADIUS * 1.5) * Math.cos(headingRad);
        double y2 = y + (ROBOT_RADIUS * 1.5) * Math.sin(headingRad);

        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.line(x2, y2);
    }

    /**
     * Draws a field coordinate as a circle.
     */
    public static void drawFieldCoordinate(FieldCoordinate coord, Style style, double radius)
    {
        if (coord == null || style == null) return;
        FieldCoordinate converted = coord.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH);
        double x = converted.x.magnitude;
        double y = converted.y.magnitude;
        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.circle(radius);
    }

    /**
     * Draws an angle (e.g., turret or custom) from the robot's position.
     *
     * @param pose     The robot pose (origin for the angle)
     * @param heading  The heading to draw (automatically converted to match dashboard system)
     * @param length   The length of the line
     * @param style    The style to use
     */
    public static void drawAngle(Pose2d pose, FieldHeading heading, double length, Style style)
    {
        if (pose == null || style == null || heading == null) return;

        // Ensure both robot pose and draw angle are in the Dashboard's native system (Pedro)
        Pose2d convertedPose = pose.toCoordinateSystem(CoordinateSystem.DECODE_PEDROPATH).toDistanceUnit(DistanceUnit.INCH);
        FieldHeading convertedHeading = heading.toSystem(CoordinateSystem.DECODE_PEDROPATH);

        double x = convertedPose.coord.x.magnitude;
        double y = convertedPose.coord.y.magnitude;
        double angleRad = convertedHeading.angle.getRadians();

        double x2 = x + length * Math.cos(angleRad);
        double y2 = y + length * Math.sin(angleRad);

        panelsField.setStyle(style);
        panelsField.moveCursor(x, y);
        panelsField.line(x2, y2);
    }

    /**
     * Sends the current packet to Panels Dashboard.
     * Call this after drawing all elements for the current frame.
     */
    public static void update()
    {
        panelsField.update();
    }
}