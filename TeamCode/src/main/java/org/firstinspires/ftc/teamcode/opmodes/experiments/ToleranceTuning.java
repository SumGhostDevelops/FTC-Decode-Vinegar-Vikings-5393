package org.firstinspires.ftc.teamcode.opmodes.experiments;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.definitions.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.opmodes.teleop.Base;

@TeleOp(name = "Tolerance Tuning", group = "Experiments")
public class ToleranceTuning extends Base
{
    private enum Tolerance {RPM_TOLERANCE, STABILITY_TOLERANCE};

    private Tolerance tolerance = Tolerance.RPM_TOLERANCE;
    private int toleranceChange = 10;

    @Override
    public void runOpMode() throws InterruptedException
    {
        team = Team.BLUE;
        RobotConstants.TELEMETRY_SET_AUTOCLEAR = false;
        super.runOpMode();
    }

    @Override
    protected void run() throws InterruptedException
    {
        super.run();

        telemetry.addLine("\n-----Tolerance Tuning-----");

        telemetry.addData("Modifying Tolerance", tolerance);
        telemetry.addData("Tolerance Change", tolerance);

        telemetry.addData("OUTTAKE_RPM_TOLERANCE", RobotConstants.OUTTAKE_RPM_TOLERANCE);
        telemetry.addData("OUTTAKE_STABILITY_TOLERANCE", RobotConstants.OUTTAKE_STABILITY_TOLERANCE);

        telemetry.update();
        telemetry.clear();
    }

    private void switchToleranceMode()
    {
        int currentOrdinal = tolerance.ordinal();
        Tolerance[] values = Tolerance.values();
        int length = values.length;
        int nextOrdinal = (currentOrdinal + 1) % length;
        tolerance = values[nextOrdinal];
    }

    private void switchTeam()
    {
        int currentOrdinal = team.ordinal();
        Team[] values = Team.values();
        int length = values.length;
        int nextOrdinal = (currentOrdinal + 1) % length;
        team = values[nextOrdinal];
    }

    private void varyTolerance(int change)
    {
        switch (tolerance)
        {
            case RPM_TOLERANCE:
                RobotConstants.OUTTAKE_RPM_TOLERANCE += change;
                break;
            case STABILITY_TOLERANCE:
                RobotConstants.OUTTAKE_STABILITY_TOLERANCE += change;
                break;
        }
    }

    @Override
    protected void bindKeys()
    {
        input.bind
                (
                        () -> gamepad2.yWasPressed(),
                        this::switchTeam
                );

        input.bind
                (
                        () -> gamepad2.aWasPressed(),
                        this::switchToleranceMode
                );

        input.bind
                (
                        () -> gamepad2.dpadUpWasPressed(),
                        () -> varyTolerance(toleranceChange)
                );

        input.bind
                (
                        () -> gamepad2.dpadDownWasPressed(),
                        () -> varyTolerance(-toleranceChange)
                );

        input.bind
                (
                        () -> gamepad2.dpadLeftWasPressed(),
                        () -> toleranceChange -= 5
                );

        input.bind
                (
                        () -> gamepad2.dpadRightWasPressed(),
                        () -> toleranceChange += 5
                );
    }
}
