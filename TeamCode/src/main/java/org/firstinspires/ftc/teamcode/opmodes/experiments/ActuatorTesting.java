package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Actuator Testing", group = "Experiments")
public class ActuatorTesting extends Base
{
    DcMotorEx backRight;
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;

        initSystems();

        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();

            telemetry.update();
        }
    }

    @Override
    protected void initSystems()
    {
        backRight = hardwareMap.get(DcMotorEx.class, RobotConstants.Drive.BACK_RIGHT);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    protected void bindKeys()
    {
        input.bind(
                () -> robot.gamepads.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25,
                () -> backRight.setPower(0.25)
        );

        input.bind(
                () -> robot.gamepads.gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25,
                () -> backRight.setPower(-0.25)
        );

        input.bind(
                () -> (robot.gamepads.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.25 && robot.gamepads.gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.25) || (robot.gamepads.gamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) < 0.25 && robot.gamepads.gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.25),
                () -> backRight.setPower(0)
        );
    }

    @Override
    protected void run() throws InterruptedException
    {
        telemetry.addData("Motor Power", backRight.getPower());
    }
}
