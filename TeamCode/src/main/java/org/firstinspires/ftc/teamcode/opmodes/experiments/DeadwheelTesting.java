package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;
import org.firstinspires.ftc.teamcode.definitions.Team;

@TeleOp(name = "Deadwheel Testing", group = "Experiments")
public class DeadwheelTesting extends Base
{
    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        super.runOpMode();
    }

    @Override
    public void run()
    {
        telemetry.addData("Deadwheel Position", hw.xEncoder.getCurrentPosition());
        telemetry.addData("Front Left Motor Position", hw.frontLeft.getCurrentPosition());
        telemetry.addData("Front Right Motor Position", hw.frontRight.getCurrentPosition());
        telemetry.addData("Back Left Motor Position", hw.backLeft.getCurrentPosition());
        telemetry.addData("Back Right Motor Position", hw.backRight.getCurrentPosition());
    }
}
