package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

@TeleOp(name = "Flywheel PIDF Tuning", group = "Experiments")
public class FlywheelPIDFTuning extends Base
{
    private enum Coefficient {P, I, D, F}

    private Coefficient coefficient = Coefficient.P;
    private double p, i, d, f;
    private double coefficientChange = 1;

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        //RobotConstants.TELEMETRY_SET_AUTOCLEAR = false;
        super.runOpMode();
    }

    @Override
    protected void initSystems()
    {
        super.initSystems();
        PIDFCoefficients cfs = robot.hw.outtakeLeftMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        p = cfs.p;
        i = cfs.i;
        d = cfs.d;
        f = cfs.f;
    }

    @Override
    protected void run() throws InterruptedException
    {
        super.run();
        telemetry.addLine("\n-----Flywheel PIDF Tuning-----");

        telemetry.addData("Modifying Coefficient", coefficient);
        telemetry.addData("Coefficient Change", coefficientChange);

        telemetry.addData("Current PIDF", "P:%.4f  I:%.4f  D:%.4f  F:%.4f", p, i, d, f);

        telemetry.update();

        //telemetry.clear();
    }

    private void varyPIDF(double change)
    {
        switch (coefficient)
        {
            case P:
                p += change;
                break;
            case I:
                i += change;
                break;
            case D:
                d += change;
                break;
            case F:
                f += change;
                break;
        }

        robot.hw.outtakeLeftMotor.setVelocityPIDFCoefficients(p, i, d, f);
        telemetry.log().add("Changed " + coefficient + " by " + change);
    }

    private void switchCoefficient()
    {
        int currentOrdinal = coefficient.ordinal();
        Coefficient[] values = Coefficient.values();
        int length = values.length;
        int nextOrdinal = (currentOrdinal + 1) % length;
        coefficient = values[nextOrdinal];
    }

    private void switchTeam()
    {
        int currentOrdinal = team.ordinal();
        Team[] values = Team.values();
        int length = values.length;
        int nextOrdinal = (currentOrdinal + 1) % length;
        team = values[nextOrdinal];
    }

    @Override
    protected void bindKeys()
    {
        telemetry.log().add("Bind keys ran.");

        input.bind
                (
                        () -> gamepad2.aWasPressed(),
                        this::switchCoefficient
                );

        input.bind
                (
                        () -> gamepad2.yWasPressed(),
                        this::switchTeam
                );

        input.bind
                (
                        () -> gamepad2.dpadUpWasPressed(),
                        () -> varyPIDF(coefficientChange)
                );

        input.bind
                (
                        () -> gamepad2.dpadDownWasPressed(),
                        () -> varyPIDF(-coefficientChange)
                );

        input.bind
                (
                        () -> gamepad2.dpadLeftWasPressed(),
                        () -> coefficientChange -= 0.05
                );

        input.bind
                (
                        () -> gamepad2.dpadRightWasPressed(),
                        () -> coefficientChange += 0.05
                );
        input.bind
                (
                        () -> gamepad2.right_trigger > 0.25,
                        () -> robot.outtake.setRPM()
                );

        input.bind
                (
                        () -> gamepad2.right_trigger <= 0.25,
                        () -> robot.outtake.stop()
                );

        input.bind
                (
                        () -> gamepad2.leftBumperWasPressed(),
                        () -> robot.outtake.varyTargetRPM(-100)
                );

        input.bind
                (
                        () -> gamepad2.rightBumperWasPressed(),
                        () -> robot.outtake.varyTargetRPM(100)
                );
    }
}
