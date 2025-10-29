package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.ControlHub;
import org.firstinspires.ftc.teamcode.exceptions.TooManyTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.teamcode.robot.AprilTagWebcam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;


// TODO: Move all code to FieldCentric

@Disabled
@TeleOp(name="OldVikingsTeleOp")
public class OldVikingsTeleOp extends LinearOpMode {
    private final double upperMultiplierLimit = 0.6;
    private final double lowerMultiplierLimit = 0.05;
    private double powerMultiplier = 0.4; // initial power reduction value

    private boolean leftBumperPressed = false;
    private boolean rightBumperPressed = false;
    private boolean yButtonPressed = false;


    ControlHub hub = new ControlHub();
    AprilTagWebcam aprilTagWebcam;
    WebcamName camera = hub.camera;



    @Override
    public void runOpMode() throws InterruptedException {
        hub.init(hardwareMap, new Pose2d(10,10,Math.toRadians(Math.PI/2)));

        aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);

        waitForStart();
        while (opModeIsActive())
        {
            motorAction(gamepad1);
        }

        aprilTagWebcam.close();
    }

    public void motorAction(Gamepad gamepad) throws InterruptedException
    {
        double y = gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        // TODO: Add keybind system for different drivers

        if (gamepad.x) // Panic button; kills all power TODO: Remove later
        {
            frontLeftPower = 0;
            frontRightPower = 0;
            backLeftPower = 0;
            backRightPower = 0;
        }

        if (gamepad.y) // Auto aim to AprilTag
        {
            if (!yButtonPressed)
            {
                AprilTagDetection tag;
                aprilTagWebcam.updateDetections();

                // Getting an AprilTag is a dangerous method, so simply restart the iteration if there is an error
                try
                {
                    tag = aprilTagWebcam.getSingleDetection(); // TODO: Add code to only aim if the AprilTag ID is ours
                }
                catch (NoTagsDetectedException | TooManyTagsDetectedException e)
                {
                    telemetry.addLine(String.valueOf(e));
                    telemetry.update();
                    return; // Restart motorAction() if there is an error; should have negligible effect on driving
                }

                // Resolve the Yaw and time it takes to turn
                double yaw = tag.ftcPose.yaw;

                double time = RobotMath.Calculate(yaw) * 2.5;

                if (yaw > 0)
                {
                    hub.leftFront.setPower(-powerMultiplier);
                    hub.leftBack.setPower(-powerMultiplier);
                    hub.rightFront.setPower(powerMultiplier);
                    hub.rightBack.setPower(powerMultiplier);
                }
                else
                {
                    hub.leftFront.setPower(powerMultiplier);
                    hub.leftBack.setPower(powerMultiplier);
                    hub.rightFront.setPower(-powerMultiplier);
                    hub.rightBack.setPower(-powerMultiplier);
                }

                telemetry.addData("Vision Yaw", yaw);
                telemetry.addData("Calculated Turn Time (s)", time);
                telemetry.update();

                sleep((long) (time * 1000));

                // Stop motors after turning
                hub.leftFront.setPower(0);
                hub.leftBack.setPower(0);
                hub.rightFront.setPower(0);
                hub.rightBack.setPower(0);

                telemetry.addLine("Done turning!");
                telemetry.update();
            }
        }
        else
        {
            yButtonPressed = false;
        }

        if (gamepad.a)
        {

        }

        if (gamepad.b)
        {

        }

        if (gamepad.dpad_up)
        {

        }

        if (gamepad.dpad_down)
        {

        }

        if (gamepad.dpad_left)
        {

        }

        if (gamepad.dpad_right)
        {

        }

        if (gamepad.left_bumper && (powerMultiplier > lowerMultiplierLimit)) // Lower speed
        {
            if (!leftBumperPressed)
            {
                powerMultiplier -= 0.05;
                leftBumperPressed = true;
                telemetry.addData("Power Multiplier: ", powerMultiplier);
                telemetry.update();
            }
        }
        else
        {
            leftBumperPressed = false;
        }

        if (gamepad.right_bumper && (powerMultiplier < upperMultiplierLimit)) // Increase speed
        {
            if (!rightBumperPressed)
            {
                powerMultiplier += 0.05;
                rightBumperPressed = true;
                telemetry.addData("Power Multiplier: ", powerMultiplier);
                telemetry.update();
            }
        }
        else
        {
            rightBumperPressed = false;
        }

        if (gamepad.right_trigger > 0.25) // Shoot
        {

        }

        if (gamepad.left_trigger > 0.25)
        {

        }

        // Handle movement inputs
        hub.leftFront.setPower(frontLeftPower * powerMultiplier);
        hub.leftBack.setPower(backLeftPower * powerMultiplier);
        hub.rightFront.setPower(frontRightPower * powerMultiplier);
        hub.rightBack.setPower(backRightPower * powerMultiplier);

        telemetry.update();
    }
}