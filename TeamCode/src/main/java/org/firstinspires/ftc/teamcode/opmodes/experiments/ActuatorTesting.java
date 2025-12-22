package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Actuator Testing", group = "Experiments")
public class ActuatorTesting extends LinearOpMode
{
    protected InputHandler input;
    DcMotorEx backRight;
    @Override
    public void runOpMode() throws InterruptedException
    {

        initSystems();

        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();

            telemetry.update();
        }
    }

    protected void initSystems()
    {
        input = new InputHandler();
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.BACK_RIGHT);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    protected void bindKeys()
    {
        input.bind(
                () -> gamepad1.right_trigger > 0.25,
                () -> backRight.setPower(0.25)
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25,
                () -> backRight.setPower(-0.25)
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25 && gamepad1.right_trigger > 0.25 || gamepad1.left_trigger < 0.25 && gamepad1.right_trigger < 0.25,
                () -> backRight.setPower(0)
        );
    }

    protected void run() throws InterruptedException
    {
        telemetry.addData("Motor Power", backRight.getPower());
    }
}
