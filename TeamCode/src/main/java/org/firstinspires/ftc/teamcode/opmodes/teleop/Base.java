package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.controls.Gamepads;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;

/**
 * This class is for only using verified, stable features (like driving)
 */
public abstract class Base extends LinearOpMode
{
    protected Team team;
    protected RobotContext robot;
    protected InputHandler input;

    protected void initSystems()
    {
        input = new InputHandler();
        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);
        //robot = new RobotContext(team, hardwareMap, telemetry, gamepads);
        bindKeys();

        telemetry.setAutoClear(RobotConstants.TELEMETRY_SET_AUTOCLEAR);
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();
    }

    /**
     * For customizing what shows up in the telemetry
     */
    protected void displayTelemetry()
    {
        telemetry.addData("Team", team);
        telemetry.addData("Heading", robot.odometry.getAngle().getUnsignedAngle(AngleUnit.DEGREES));
    }

    /**
     * For updating important systems, like the Telemetry or Localization
     */
    protected void update()
    {
        telemetry.update();
        input.update();
    }

    /**
     * For controlling what things need to be properly ended
     */
    protected void close()
    {
        robot.odometry.close();
    }

    protected void bindKeys()
    {
        // Handle B
        input.bind
                (
                        () -> robot.gamepads.driver.wasJustPressed(GamepadKeys.Button.B),
                        () -> robot.odometry.resetAngle()
                );

        input.bind
                (
                        () -> robot.gamepads.driver.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () ->
                        {
                            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER - RobotConstants.DRIVE_SPEED_CHANGE;

                            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.max(RobotConstants.DRIVE_SPEED_MINIMUM, newMultiplier);
                            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
                        }
                );

        input.bind
                (
                        () -> robot.gamepads.driver.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () ->
                        {
                            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER + RobotConstants.DRIVE_SPEED_CHANGE;

                            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.min(RobotConstants.DRIVE_SPEED_MAXIMUM, newMultiplier);
                            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
                        }
                );
    }

    /**
     * Runs initSystems(), displayTelemetry(), update(), run(), and eventually close()
     * @throws InterruptedException
     */
    @Override
    public void runOpMode() throws InterruptedException
    {
        initSystems();

        waitForStart();
        while (opModeIsActive())
        {
            displayTelemetry();
            update();
            run();
        }

        close();
    }

    protected void run() throws InterruptedException
    {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        //robot.drive.drive(axial, lateral, yaw, robot.odometry.getDriverHeading());
    }
}