package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "EasyAutonLeave")
public class EasyAutonLeave extends LinearOpMode
{
    // Created by team #5393, the Vinegar Vikings
    // Make sure to configure this section!

    // --------------------------------------------------

    // The names of your wheels, as configured into your Driver Hub and their directions
    private final String frontLeftWheel = "frontLeft";
    private final DcMotorSimple.Direction frontLeftDirection = DcMotorSimple.Direction.FORWARD;

    private final String frontRightWheel = "frontRight";
    private final DcMotorSimple.Direction frontRightDirection = DcMotorSimple.Direction.FORWARD;

    private final String backLeftWheel = "backLeft";
    private final DcMotorSimple.Direction backLeftDirection = DcMotorSimple.Direction.FORWARD;

    private final String backRightWheel = "backRight";
    private final DcMotorSimple.Direction backRightDirection = DcMotorSimple.Direction.FORWARD;

    // --------------------------------------------------
    // Make sure that these two numbers (waitTime and driveTime) are less than 30 seconds (the length of the autonomous period).

    private final double waitTime = 25; // How long to wait before driving
    private final double driveTime = 2; // How long to drive for

    private final double drivePower = 1; // The power to send to the drive wheels (from 0-1

    // --------------------------------------------------

    // You probably don't need to change anything under this line.
    @Override
    public void runOpMode() throws InterruptedException
    {
        // Get all of the motors
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, frontLeftWheel);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, frontRightWheel);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, backLeftWheel);
        DcMotor backRight = hardwareMap.get(DcMotor.class, backRightWheel);

        // Set the motor directions
        frontLeft.setDirection(frontLeftDirection);
        frontRight.setDirection(frontRightDirection);
        backLeft.setDirection(backLeftDirection);
        backRight.setDirection(backRightDirection);

        // Set the motors to brake mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for waitTime seconds
        telemetry.setAutoClear(false);
        telemetry.addLine("Waiting for " + waitTime + " seconds before driving.");
        telemetry.update();

        // Drive for driveTime seconds
        Thread.sleep((long) (waitTime * 1000));
        telemetry.addLine("Driving now (for " + driveTime + " seconds)!");
        telemetry.update();

        // Set the power of the drive wheels
        frontLeft.setPower(drivePower);
        frontRight.setPower(drivePower);
        backLeft.setPower(drivePower);
        backRight.setPower(drivePower);

        Thread.sleep((long) (driveTime * 1000));

        // Stop driving
        telemetry.addLine("Stopping now!");
        telemetry.update();

        // Set the power of the driver wheels to 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        Thread.sleep(1000); // To prevent the OpMode from instantly ending and allow braking to happen.
        telemetry.addLine("Done!");
        telemetry.update();
    }
}