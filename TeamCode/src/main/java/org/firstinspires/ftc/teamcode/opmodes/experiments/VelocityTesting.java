package org.firstinspires.ftc.teamcode.opmodes.experiments;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.util.RobotMath;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;

@TeleOp(name = "Velocity Testing", group = "Experimental")
@Disabled
public class VelocityTesting extends LinearOpMode
{
    MotorEx motor;
    DcMotorEx dcMotorEx;
    int ticks = 1000;

    @Override
    public void runOpMode() throws InterruptedException
    {
        motor = new MotorEx(hardwareMap, "leftOuttake", Motor.GoBILDA.BARE);
        dcMotorEx = motor.motorEx;

        telemetry();
        waitForStart();

        while (opModeIsActive())
        {
            run();
        }
    }

    public void telemetry()
    {
        telemetry.addData("Ticks", ticks);
        //telemetry.addData("Achievable Max RPM", motor.getMaxRPM());
        telemetry.addData("Motor Velocity", motor.getCorrectedVelocity());
        telemetry.addData("Motor RPM", RobotMath.Motor.tpsToRpm(motor.getCorrectedVelocity(), 28));
        telemetry.addData("Acceleration", motor.getAcceleration());
        telemetry.update();
    }

    public void run()
    {
        telemetry();

        if (gamepad1.aWasPressed())
        {
            dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //motor.setRunMode(Motor.RunMode.VelocityControl);
            //motor.setVelocity(ticks);
            dcMotorEx.setVelocity(ticks);
        }
        else if (gamepad1.aWasReleased())
        {
            dcMotorEx.setPower(0);
        }

        if (gamepad1.dpadUpWasPressed())
        {
            ticks += 100;
        }
        if (gamepad1.dpadDownWasPressed())
        {
            ticks -= 100;
        }
    }

}
