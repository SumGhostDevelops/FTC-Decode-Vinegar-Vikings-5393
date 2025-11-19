package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Team;

public abstract class VikingsTeleOp extends LinearOpMode
{
    protected Team team;

    @Override
    public void runOpMode() throws InterruptedException
    {
        waitForStart();
        while (opModeIsActive())
        {
            motorAction(gamepad1);
        }
    }

    public void motorAction(Gamepad gamepad) throws InterruptedException
    {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;
        double botHeading = robot.hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);

        robot.wheels.leftFront = (rotY + rotX + rx) / denominator;
        robot.wheels.leftBack = (rotY - rotX + rx) / denominator;
        robot.wheels.rightFront = (rotY - rotX - rx) / denominator;
        robot.wheels.rightBack = (rotY + rotX - rx) / denominator;
    }
}