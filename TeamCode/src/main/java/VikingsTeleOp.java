import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
@TeleOp(name="VikingsTeleOp")
public class VikingsTeleOp extends LinearOpMode {
    ControlHub hub;
    @Override
    public void runOpMode() throws InterruptedException {
        hub = new ControlHub();
        hub.init(hardwareMap,new Pose2d(10,10,1));

        waitForStart();
        while (opModeIsActive())
        {
            motorAction(gamepad1);
        }
    }

    public void motorAction(Gamepad gamepad)
    {
        double y = -gamepad.left_stick_y; // Remember, Y stick value is reversed
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
        if (gamepad.x) // if you press x it kills all power
        {
            frontLeftPower=0;
            frontRightPower=0;
            backLeftPower=0;
            backRightPower=0;
        }

        if (!gamepad.x) // if power is not called to be killed
        {
            hub.leftFront.setPower(frontLeftPower/2);
            hub.leftBack.setPower(backLeftPower/2);
            hub.rightFront.setPower(frontRightPower/2);
            hub.rightBack.setPower(backRightPower/2);
        }
    }
}