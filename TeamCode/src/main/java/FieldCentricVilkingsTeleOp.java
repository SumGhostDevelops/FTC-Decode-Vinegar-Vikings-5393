import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp(name="FieldCentricVilkingsTeleOp")
public class FieldCentricVilkingsTeleOp extends LinearOpMode {
    private final double upperMultiplierLimit = 0.6;
    private final double lowerMultiplierLimit = 0.05;
    private double powerMultiplier = 0.4; // initial power reduction value


    ControlHub hub = new ControlHub();
    VisionHelper visionHelper;
    WebcamName camera = hub.camera;







    @Override
    public void runOpMode() throws InterruptedException {
        hub.init(hardwareMap, new Pose2d(10,10,Math.toRadians(Math.PI/2)));


        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(

 // depending on what direction the logo is facing on the control hub would determine what orientation is.
 //change depending on what it actually is lmao

                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));

        hub.imu.initialize(parameters);

        visionHelper = new VisionHelper(new double[]{1424.38, 1424.38, 637.325, 256.774}, hub.camera, 2);

        waitForStart();
        while (opModeIsActive())
        {
            motorAction(gamepad1);
        }

        visionHelper.close();
    }

    public void motorAction(Gamepad gamepad) throws InterruptedException
    {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad.left_stick_x;
        double rx = gamepad.right_stick_x;
        double heading = hub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);


        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        //pick anything lmao
            if (gamepad1.b) {
                hub.imu.resetYaw();
            }

        if (gamepad.x) // if you press x it kills all power
        {
            frontLeftPower = 0;
            frontRightPower = 0;
            backLeftPower = 0;
            backRightPower = 0;
        }

        if (gamepad.left_bumper && powerMultiplier > lowerMultiplierLimit)
        {
            powerMultiplier -= 0.05;
        }
        else if (gamepad.right_bumper && powerMultiplier < upperMultiplierLimit)
        {
            powerMultiplier += 0.05;
        }

        if (!gamepad.x) // if power is not called to be killed
        {
            hub.leftFront.setPower(frontLeftPower*powerMultiplier);
            hub.leftBack.setPower(backLeftPower*powerMultiplier);
            hub.rightFront.setPower(frontRightPower*powerMultiplier);
            hub.rightBack.setPower(backRightPower*powerMultiplier);
        }

        if (gamepad.y)
        {
            double yaw = visionHelper.get("yaw");
            double time = TurningMath.Calculate(yaw) * 2.5;

            if (yaw > 0)
            {
                hub.leftFront.setPower(frontLeftPower*powerMultiplier);
                hub.leftBack.setPower(backLeftPower*powerMultiplier);
                hub.rightFront.setPower(-frontRightPower*powerMultiplier);
                hub.rightBack.setPower(-backRightPower*powerMultiplier);
            }
            else
            {
                hub.leftFront.setPower(-frontLeftPower*powerMultiplier);
                hub.leftBack.setPower(-backLeftPower*powerMultiplier);
                hub.rightFront.setPower(frontRightPower*powerMultiplier);
                hub.rightBack.setPower(backRightPower*powerMultiplier);
            }

            telemetry.addData("Time to turn:", time);
            sleep((long) (time*1000));
            telemetry.addLine("Done turning!");

            hub.leftFront.setPower(0);
            hub.leftBack.setPower(0);
            hub.rightFront.setPower(0);
            hub.rightBack.setPower(0);

        }
    }
}


