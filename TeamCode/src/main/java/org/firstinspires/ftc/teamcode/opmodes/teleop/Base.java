package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.modules.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.modules.odometry.modules.EncompassingPose;
import org.firstinspires.ftc.teamcode.definitions.RobotContext;

public abstract class Base extends LinearOpMode
{
    protected Team team;

    protected RobotContext robot;

    protected InputHandler input;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initSystems();

        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();

            robot.localization.update();
            robot.intake.update();
            robot.transfer.update();
            robot.outtake.update();

            telemetry.update();
        }

        robot.localization.close();
    }

    protected void initSystems()
    {
        Gamepads gamepads = new Gamepads(gamepad1, gamepad2);
        RobotContext robot = new RobotContext(team, hardwareMap, telemetry, gamepads);
        bindKeys();

        telemetry.setAutoClear(RobotConstants.TELEMETRY_SET_AUTOCLEAR);
        telemetry.addData("Status", "Initialized for " + team);
        telemetry.update();
    }

    protected void run() throws InterruptedException
    {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        // Drive - only send manual commands if no blocking macro is running
        robot.drive.drive(axial, lateral, yaw);

        //telemetry.addData("Drive Mode", drive.getMode());
        telemetry.addData("Team", team);
        telemetry.addLine("\n-----Velocity-----");
        telemetry.addData("Speed", RobotConstants.DRIVE_SPEED_MULTIPLIER);
        telemetry.addData("Heading", robot.localization.getHeading(AngleUnit.DEGREES, EncompassingPose.AngleType.UNSIGNED));
        telemetry.addLine("\n-----Outtake-----");
        telemetry.addData("Toggled", robot.outtake.isToggled());
        telemetry.addData("F Offset", RobotConstants.OUTTAKE_F_OFFSET);
        telemetry.addData("Target RPM", robot.outtake.getTargetRPM());
        telemetry.addData("RPM", robot.outtake.getRPM());
        telemetry.addData("RPM Acceleration", robot.outtake.getRPMAcceleration());
    }

    protected void bindKeys()
    {
        // Handle B
        input.bind
                (
                        () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.B),
                        () -> robot.localization.resetHeading()
                );

        input.bind
                (
                        () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.X),
                        () -> robot.outtake.toggleRPM()
                );

        input.bind
                (
                        () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER),
                        () ->
                        {
                            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER - RobotConstants.DRIVE_SPEED_CHANGE;

                            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.max(RobotConstants.DRIVE_SPEED_MINIMUM, newMultiplier);
                            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
                        }
                );

        input.bind
                (
                        () -> robot.gamepads.gamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER),
                        () ->
                        {
                            double newMultiplier = RobotConstants.DRIVE_SPEED_MULTIPLIER + RobotConstants.DRIVE_SPEED_CHANGE;

                            RobotConstants.DRIVE_SPEED_MULTIPLIER = Math.min(RobotConstants.DRIVE_SPEED_MAXIMUM, newMultiplier);
                            telemetry.log().add("New Speed: " + RobotConstants.DRIVE_SPEED_MULTIPLIER);
                        }
                );

        input.bind
                (
                        () -> gamepad1.left_trigger > 0.25 && robot.outtake.isReadyToLaunch(),
                        () -> robot.transfer.setPower(1)
                );

        input.bind
                (
                        () -> gamepad1.left_trigger <= 0.25 || !robot.outtake.isReadyToLaunch(),
                        () -> robot.transfer.stop()
                );

        input.bind
                (
                        () -> gamepad1.right_trigger > 0.25,
                        () -> robot.outtake.setRPM()
                );

        input.bind
                (
                        () -> gamepad1.right_trigger <= 0.25,
                        () -> robot.outtake.stop()
                );
    }
}