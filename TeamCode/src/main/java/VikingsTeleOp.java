import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="VikingsTeleOp")
public class VikingsTeleOp extends LinearOpMode {
    private final double upperMultiplierLimit = 0.6;
    private final double lowerMultiplierLimit = 0.05;
    private double powerMultiplier = 0.4; // initial power reduction value


    ControlHub hub = new ControlHub();
    VisionHelper visionHelper;
    WebcamName camera = hub.camera;



    @Override
    public void runOpMode() throws InterruptedException {
        hub.init(hardwareMap, new Pose2d(10,10,Math.toRadians(Math.PI/2)));

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
            // TODO: Add code to only aim if the AprilTag ID is ours

            // Resolve the Yaw and time it takes to turn
            double yaw = visionHelper.getYaw();
            double time = TurningMath.Calculate(yaw) * 2.5;

            if (yaw > 0)
            {
                hub.leftFront.setPower(frontLeftPower * powerMultiplier);
                hub.leftBack.setPower(backLeftPower * powerMultiplier);
                hub.rightFront.setPower(-frontRightPower * powerMultiplier);
                hub.rightBack.setPower(-backRightPower * powerMultiplier);
            }
            else
            {
                hub.leftFront.setPower(-frontLeftPower * powerMultiplier);
                hub.leftBack.setPower(-backLeftPower * powerMultiplier);
                hub.rightFront.setPower(frontRightPower * powerMultiplier);
                hub.rightBack.setPower(backRightPower * powerMultiplier);
            }

            telemetry.addData("Time to turn:", time);
            sleep((long) (time * 1000));
            telemetry.addLine("Done turning!");

            hub.leftFront.setPower(0);
            hub.leftBack.setPower(0);
            hub.rightFront.setPower(0);
            hub.rightBack.setPower(0);
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

        if (gamepad.left_bumper && powerMultiplier > lowerMultiplierLimit) // Lower speed
        {
            powerMultiplier -= 0.05;
        }
        else if (gamepad.right_bumper && powerMultiplier < upperMultiplierLimit) // Increase speed
        {
            powerMultiplier += 0.05;
        }

        if (gamepad.right_trigger > 0.25) // TODO: Shoot
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
    }
}