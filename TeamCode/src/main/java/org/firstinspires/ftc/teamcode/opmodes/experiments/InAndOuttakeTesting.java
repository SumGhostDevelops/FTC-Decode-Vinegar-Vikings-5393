package org.firstinspires.ftc.teamcode.opmodes.experiments;



import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.controls.InputHandler;
import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.util.MotorExPlus;
import org.firstinspires.ftc.teamcode.util.MotorExPlusGroup;

@TeleOp(name = "Intake + Outtake Testing", group = "Experiments")

public class InAndOuttakeTesting extends LinearOpMode {

    //Intake Variables
    protected InputHandler input;
    double power = 0.5;
    MotorExPlus intake;



    //Outtake Variables

    private RobotHardware robot;
    private MotorExPlusGroup outtakeGroup;

    // Tuning Parameters
    private double kS = 0.0;
    private double kV = 0.0;
    private double kA = 0.0;
    private double kP = 0.0;
    private double kI = 0.0;
    private double kD = 0.0;

    private double targetRPM = 5000;
    private double testPower = 0.8;

    // Menu Navigation
    private enum Param {
        TEST_POWER,
        KS,
        KV,
        KA,
        KP,
        KI,
        KD,
        TARGET_RPM
    }

    private final Param[] params = Param.values();
    private int paramIndex = 0;

    // Input State
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastY = false;







    public void runOpMode() throws InterruptedException
    {
        // Initialize Telemetry with Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        initSystems();

        telemetry.addLine("Ready to tune.");
        telemetry.addLine("Controls:");
        telemetry.addLine("D-Pad Up/Down: Adjust Value");
        telemetry.addLine("D-Pad Left/Right: Select Parameter");
        telemetry.addLine("Hold A: Run Motor");
        telemetry.addLine("Y: Copy Calculated Value (Context Sensitive)");
        telemetry.update();

        waitForStart();
        while (opModeIsActive())
        {
            input.update();

            run();

            telemetry.update();

            handleInput();
            updateMotorConfig();
            runLogic();
            displayTelemetry();
        }
    }

    protected void initSystems()
    {
        input = new InputHandler();
        RobotHardware hw = new RobotHardware(hardwareMap, telemetry);
        intake = hw.intake;
        robot = new RobotHardware(hardwareMap, telemetry);
        outtakeGroup = robot.getOuttakeMotorExPlusGroup();
        bindKeys();
    }

    protected void bindKeys()
    {
        input.bind(
                () -> gamepad1.right_trigger > 0.25,
                () -> intake.set(power)
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25,
                () -> intake.set(-power)
        );

        input.bind(
                () -> gamepad1.left_trigger > 0.25 && gamepad1.right_trigger > 0.25 || gamepad1.left_trigger < 0.25 && gamepad1.right_trigger < 0.25,
                () -> intake.set(0)
        );

        input.bind(
                () -> gamepad1.dpadUpWasPressed(),
                () -> power += 0.1
        );

        input.bind(
                () -> gamepad1.dpadDownWasPressed(),
                () -> power -= 0.1
        );
    }

    protected void run() throws InterruptedException
    {
        telemetry.addData("Power", power);
        telemetry.addData("Motor RPM", intake.getRPM());
    }





// Flywheel stuff



    private void handleInput() {
        // Navigation
        if (gamepad1.dpad_right && !lastRight) {
            paramIndex = (paramIndex + 1) % params.length;
        }
        if (gamepad1.dpad_left && !lastLeft) {
            paramIndex = (paramIndex - 1 + params.length) % params.length;
        }
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        Param current = params[paramIndex];
        double increment = getIncrement(current);

        // Value Adjustment
        if (gamepad1.dpad_up && !lastUp) {
            adjustValue(current, increment);
        }
        if (gamepad1.dpad_down && !lastDown) {
            adjustValue(current, -increment);
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;

        // Copy Value (Y button)
        if (gamepad1.y && !lastY) {
            if (current == Param.KS) {
                kS = testPower;
            } else if (current == Param.KV) {
                double currentRPM = getAverageRPM();
                if (Math.abs(currentRPM) > 10) {
                    kV = testPower / currentRPM;
                }
            }
        }
        lastY = gamepad1.y;
    }

    private double getIncrement(Param param) {
        switch (param) {
            case TEST_POWER: return 0.01;
            case KS: return 0.001;
            case KV: return 0.00001;
            case KA: return 0.00001;
            case KP: return 0.0001;
            case KI: return 0.0001;
            case KD: return 0.0001;
            case TARGET_RPM: return 50;
            default: return 0;
        }
    }

    private void adjustValue(Param param, double delta) {
        switch (param) {
            case TEST_POWER: testPower += delta; break;
            case KS: kS += delta; break;
            case KV: kV += delta; break;
            case KA: kA += delta; break;
            case KP: kP += delta; break;
            case KI: kI += delta; break;
            case KD: kD += delta; break;
            case TARGET_RPM: targetRPM += delta; break;
        }
    }

    private void updateMotorConfig() {
        if (outtakeGroup != null) {
            outtakeGroup.setVeloCoefficients(kP, kI, kD);
            outtakeGroup.setFeedforwardCoefficients(kS, kV, kA);
        }
    }

    private void runLogic() {
        if (outtakeGroup == null) return;

        Param current = params[paramIndex];
        boolean runOpenLoop = (current == Param.TEST_POWER || current == Param.KS || current == Param.KV);

        if (gamepad1.a) {
            if (runOpenLoop) {
                outtakeGroup.setRunMode(Motor.RunMode.RawPower);
                outtakeGroup.set(testPower);
            } else {
                outtakeGroup.setRunMode(Motor.RunMode.VelocityControl);
                outtakeGroup.setRPM(targetRPM);
            }
        } else {
            outtakeGroup.set(0);
            outtakeGroup.setRunMode(Motor.RunMode.RawPower);
        }
    }

    //will currently only work for one motor
    private double getAverageRPM() {
        if (robot.outtakeLeft == null) return 0;
        return (robot.outtakeLeft.getRPM()); // outtakeGroup.getRPM() does the same thing but i am too scared to change it
    }

    private void displayTelemetry() {
        telemetry.addData("Selected", params[paramIndex]);
        telemetry.addLine("-----------------");
        telemetry.addData("Test Power", "%.2f", testPower);
        telemetry.addData("Target RPM", "%.0f", targetRPM);
        telemetry.addLine("-----------------");
        telemetry.addData("kS", "%.5f", kS);
        telemetry.addData("kV", "%.6f", kV);
        telemetry.addData("kA", "%.6f", kA);
        telemetry.addData("kP", "%.5f", kP);
        telemetry.addData("kI", "%.5f", kI);
        telemetry.addData("kD", "%.5f", kD);
        telemetry.addLine("-----------------");

        double rpm = getAverageRPM();
        telemetry.addData("Actual RPM", "%.1f", rpm);

        if (params[paramIndex] == Param.KV && Math.abs(rpm) > 10) {
            telemetry.addData("Calc kV (Power/RPM)", "%.6f", testPower / rpm);
        }

        telemetry.update();
    }







}
