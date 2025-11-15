package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.robot.Actions;
import org.firstinspires.ftc.teamcode.robot.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.robot.ControlHub;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.RobotContext;
import org.firstinspires.ftc.teamcode.robot.Wheels;

import org.firstinspires.ftc.teamcode.exceptions.NoTagsDetectedException;
import org.firstinspires.ftc.teamcode.exceptions.TagNotFoundException;
import org.firstinspires.ftc.teamcode.util.RobotMath;

@Autonomous(name="VikingsAuton")
public class VikingsAuton extends LinearOpMode
{
    RobotContext robot;
    Actions actions;

    @Override
    public void runOpMode() throws InterruptedException
    {
        ControlHub hub = new ControlHub();
        hub.init(hardwareMap, new Pose2d(10, 10, Math.toRadians(Math.PI / 2)));
        AprilTagWebcam aprilTagWebcam = new AprilTagWebcam(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, true);
        robot = new RobotContext(hub, aprilTagWebcam, telemetry, gamepad1, this::opModeIsActive, new Robot(), new Wheels());
        actions = new Actions(robot);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

                // depending on what direction the logo is facing on the control hub would determine what orientation is.
                //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

        robot.hub.imu.initialize(parameters);
        robot.hub.imu.resetYaw();

        telemetry.setAutoClear(false);

        waitForStart();

        actions.sleep(2, "Waiting for camera to have a clear picture.");

        robot.webcam.updateDetections();

        try
        {
            int tagId = robot.webcam.getAnyGoalId();
            robot.telemetry.log().add("detected goal tag id (" + tagId + ")");
            actions.aimToAprilTag(tagId);
            robot.telemetry.log().add("aimed to april tag");
            actions.updateLauncherSpeed(tagId);
            robot.telemetry.log().add("adjusted launcher speed");
        }
        catch (NoTagsDetectedException | TagNotFoundException e)
        {
            robot.telemetry.log().add("Could not find the AprilTag, so we will not automatically aim: " + e.getMessage());
        }

        robot.hub.launcher.setPower(robot.self.getLauncherSpeed());

        actions.sleep(3);
        for (int i = 0; i < 4; i++)
        {
            robot.hub.loader.setPower(1);
            actions.sleep(0.4, "Letting the loader move the ball.");
            robot.hub.loader.setPower(0);
            if (i == 3)
            {
                actions.sleep(0.5);
                break;
            }
            actions.sleep(2, "Letting the launcher launch.");
        }

        robot.hub.launcher.setPower(0);
        robot.wheels.setAllPower(1);
        actions.move();
        actions.sleep(0.4, "Letting the robot move.");
        robot.wheels.setAllPower(0);
        actions.move();

        robot.webcam.close();
    }
}