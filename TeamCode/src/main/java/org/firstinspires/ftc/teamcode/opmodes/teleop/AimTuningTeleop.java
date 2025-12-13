package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;
import org.firstinspires.ftc.teamcode.definitions.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Gamepads;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Localization;

@TeleOp(name = "Aim Tuning", group = "Tuning")
public class AimTuningTeleop extends LinearOpMode
{
    // Hardware and subsystems
    private RobotHardware hw;
    private Localization localization;
    private Drive drive;
    private Intake intake;
    private Outtake outtake;
    private Transfer transfer;
    private Gamepads gamepads;

    // Tunable parameters for aimToAngleFast
    private double coarseThreshold = 15.0;
    private double fineTolerance = 1.5;
    private double angularRateThreshold = 8.0;
    private int requiredSettledLoops = 2;

    private double kP_coarse = 0.05;
    private double kD_coarse = 0.003;
    private double maxPower_coarse = 1.0;
    private double minPower_coarse = 0.18;

    private double kP_fine = 0.02;
    private double kD_fine = 0.005;
    private double maxPower_fine = 0.4;

    private double derivativeFilterAlpha = 0.5;

    // Test target angle
    private double targetAngle = 0.0;
    private boolean isTurning = false;

    // Parameter selection
    private int selectedParam = 0;
    private String[] paramNames = {
        "coarseThreshold",
        "fineTolerance",
        "angularRateThreshold",
        "requiredSettledLoops",
        "kP_coarse",
        "kD_coarse",
        "maxPower_coarse",
        "minPower_coarse",
        "kP_fine",
        "kD_fine",
        "maxPower_fine",
        "derivativeFilterAlpha"
    };

    private double[] adjustSteps = {
        1.0,    // coarseThreshold
        0.1,    // fineTolerance
        0.5,    // angularRateThreshold
        1.0,    // requiredSettledLoops
        0.005,  // kP_coarse
        0.001,  // kD_coarse
        0.05,   // maxPower_coarse
        0.01,   // minPower_coarse
        0.002,  // kP_fine
        0.001,  // kD_fine
        0.05,   // maxPower_fine
        0.05    // derivativeFilterAlpha
    };

    @Override
    public void runOpMode()
    {
        // Initialize hardware
        hw = new RobotHardware(hardwareMap, telemetry);
        localization = new Localization(hw);
        drive = new Drive(hw, localization);
        outtake = new Outtake(hw);
        intake = new Intake(hw);
        transfer = new Transfer(hw);
        gamepads = new Gamepads(gamepad1, gamepad2);

        telemetry.setAutoClear(true);
        telemetry.addData("Status", "Initialized - Aim Tuning Mode");
        telemetry.addData("Controls", "");
        telemetry.addLine("  DPAD UP/DOWN: Select parameter");
        telemetry.addLine("  DPAD LEFT/RIGHT: Adjust value");
        telemetry.addLine("  A: Test turn to 0°");
        telemetry.addLine("  B: Test turn to 90°");
        telemetry.addLine("  X: Test turn to 180°");
        telemetry.addLine("  Y: Test turn to -90°");
        telemetry.addLine("  Right Stick: Manual drive");
        telemetry.update();

        waitForStart();

        ElapsedTime buttonCooldown = new ElapsedTime();
        buttonCooldown.reset();

        while (opModeIsActive())
        {
            // Update subsystems
            localization.update();

            // Parameter selection (DPAD UP/DOWN)
            if (buttonCooldown.seconds() > 0.2)
            {
                if (gamepad1.dpad_up)
                {
                    selectedParam = (selectedParam - 1 + paramNames.length) % paramNames.length;
                    buttonCooldown.reset();
                }
                else if (gamepad1.dpad_down)
                {
                    selectedParam = (selectedParam + 1) % paramNames.length;
                    buttonCooldown.reset();
                }
                // Parameter adjustment (DPAD LEFT/RIGHT)
                else if (gamepad1.dpad_left)
                {
                    adjustParameter(-1);
                    buttonCooldown.reset();
                }
                else if (gamepad1.dpad_right)
                {
                    adjustParameter(1);
                    buttonCooldown.reset();
                }
                // Test turn buttons
                else if (gamepad1.a)
                {
                    targetAngle = 0.0;
                    testTurn();
                    buttonCooldown.reset();
                }
                else if (gamepad1.b)
                {
                    targetAngle = 90.0;
                    testTurn();
                    buttonCooldown.reset();
                }
                else if (gamepad1.x)
                {
                    targetAngle = 180.0;
                    testTurn();
                    buttonCooldown.reset();
                }
                else if (gamepad1.y)
                {
                    targetAngle = -90.0;
                    testTurn();
                    buttonCooldown.reset();
                }
            }

            // Manual drive when not testing
            if (!isTurning)
            {
                double yaw = gamepad1.right_stick_x;
                drive.setDrivePowers(-yaw, -yaw, yaw, yaw);
            }

            // Display telemetry
            displayTelemetry();
            telemetry.update();
        }

        localization.close();
    }

    private void adjustParameter(int direction)
    {
        double step = adjustSteps[selectedParam] * direction;

        switch (selectedParam)
        {
            case 0: coarseThreshold = Math.max(1.0, coarseThreshold + step); break;
            case 1: fineTolerance = Math.max(0.1, fineTolerance + step); break;
            case 2: angularRateThreshold = Math.max(0.5, angularRateThreshold + step); break;
            case 3: requiredSettledLoops = (int) Math.max(1, requiredSettledLoops + direction); break;
            case 4: kP_coarse = Math.max(0.001, kP_coarse + step); break;
            case 5: kD_coarse = Math.max(0.0, kD_coarse + step); break;
            case 6: maxPower_coarse = Math.max(0.1, Math.min(1.0, maxPower_coarse + step)); break;
            case 7: minPower_coarse = Math.max(0.0, Math.min(0.5, minPower_coarse + step)); break;
            case 8: kP_fine = Math.max(0.001, kP_fine + step); break;
            case 9: kD_fine = Math.max(0.0, kD_fine + step); break;
            case 10: maxPower_fine = Math.max(0.1, Math.min(1.0, maxPower_fine + step)); break;
            case 11: derivativeFilterAlpha = Math.max(0.0, Math.min(1.0, derivativeFilterAlpha + step)); break;
        }
    }

    private void testTurn()
    {
        isTurning = true;

        double error;
        double motorPower;
        double lastError;
        double rawDerivative;
        double filteredDerivative = 0.0;
        double angularRate;
        int settledLoopCount = 0;
        boolean inFineStage;

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime totalTimer = new ElapsedTime();
        double loopTime;

        double currentAngle = localization.getHeading();
        error = targetAngle - currentAngle;
        lastError = error;

        loopTimer.reset();
        totalTimer.reset();

        telemetry.log().add("Starting turn to " + targetAngle + "°");

        do
        {
            // Update
            currentAngle = localization.getHeading();
            loopTime = loopTimer.seconds();
            loopTimer.reset();

            // Calculate error
            error = targetAngle - currentAngle;

            // Calculate angular rate
            if (loopTime > 0.0001)
            {
                rawDerivative = (error - lastError) / loopTime;
                filteredDerivative = (derivativeFilterAlpha * filteredDerivative) +
                                    ((1.0 - derivativeFilterAlpha) * rawDerivative);
                angularRate = Math.abs(filteredDerivative);
            }
            else
            {
                filteredDerivative = 0.0;
                angularRate = 0.0;
            }

            // Determine stage
            inFineStage = Math.abs(error) <= coarseThreshold;

            // Select gains
            double kP = inFineStage ? kP_fine : kP_coarse;
            double kD = inFineStage ? kD_fine : kD_coarse;
            double maxPower = inFineStage ? maxPower_fine : maxPower_coarse;

            // Calculate power
            motorPower = (error * kP) + (filteredDerivative * kD);

            // Apply minimum power
            if (!inFineStage && Math.abs(error) > fineTolerance)
            {
                if (Math.abs(motorPower) < minPower_coarse)
                {
                    motorPower = Math.signum(motorPower) * minPower_coarse;
                }
            }

            // Clamp power
            motorPower = Math.max(-maxPower, Math.min(maxPower, motorPower));

            // Apply power
            drive.setDrivePowers(-motorPower, -motorPower, motorPower, motorPower);

            // Settling detection
            boolean isPositionSettled = Math.abs(error) <= fineTolerance;
            boolean isRotationSettled = angularRate <= angularRateThreshold;

            if (isPositionSettled && isRotationSettled)
            {
                settledLoopCount++;
            }
            else
            {
                settledLoopCount = 0;
            }

            // Live telemetry during turn
            telemetry.clearAll();
            telemetry.addData("TURNING", "Target: " + targetAngle + "°");
            telemetry.addData("Stage", inFineStage ? "FINE" : "COARSE");
            telemetry.addData("Error", String.format("%.2f°", error));
            telemetry.addData("Angular Rate", String.format("%.1f°/s", angularRate));
            telemetry.addData("Power", String.format("%.3f", motorPower));
            telemetry.addData("Settling", settledLoopCount + "/" + requiredSettledLoops);
            telemetry.addData("Time", String.format("%.2fs", totalTimer.seconds()));
            telemetry.update();

            lastError = error;

        } while (settledLoopCount < requiredSettledLoops &&
                 opModeIsActive() &&
                 totalTimer.seconds() < 3.0);

        // Stop
        drive.stop();

        // Log results
        telemetry.log().add("Turn complete!");
        telemetry.log().add("Final error: " + String.format("%.2f°", error));
        telemetry.log().add("Time: " + String.format("%.2fs", totalTimer.seconds()));

        isTurning = false;
    }

    private void displayTelemetry()
    {
        telemetry.addLine("=== AIM TUNING ===");
        telemetry.addData("Current Heading", String.format("%.1f°", localization.getHeading()));
        telemetry.addLine();

        telemetry.addLine("--- STAGE THRESHOLDS ---");
        displayParam(0, "coarseThreshold", coarseThreshold, "°");
        displayParam(1, "fineTolerance", fineTolerance, "°");
        displayParam(2, "angularRateThreshold", angularRateThreshold, "°/s");
        displayParam(3, "requiredSettledLoops", (double) requiredSettledLoops, "");

        telemetry.addLine();
        telemetry.addLine("--- COARSE STAGE ---");
        displayParam(4, "kP_coarse", kP_coarse, "");
        displayParam(5, "kD_coarse", kD_coarse, "");
        displayParam(6, "maxPower_coarse", maxPower_coarse, "");
        displayParam(7, "minPower_coarse", minPower_coarse, "");

        telemetry.addLine();
        telemetry.addLine("--- FINE STAGE ---");
        displayParam(8, "kP_fine", kP_fine, "");
        displayParam(9, "kD_fine", kD_fine, "");
        displayParam(10, "maxPower_fine", maxPower_fine, "");

        telemetry.addLine();
        telemetry.addLine("--- FILTERING ---");
        displayParam(11, "derivativeFilterAlpha", derivativeFilterAlpha, "");

        telemetry.addLine();
        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("DPAD ↕: Select  |  DPAD ←→: Adjust");
        telemetry.addLine("A: 0°  B: 90°  X: 180°  Y: -90°");
    }

    private void displayParam(int index, String name, double value, String unit)
    {
        String marker = (index == selectedParam) ? ">>> " : "    ";
        String valueStr;

        if (index == 3) // requiredSettledLoops is an int
        {
            valueStr = String.format("%d", (int) value);
        }
        else
        {
            valueStr = String.format("%.3f", value);
        }

        telemetry.addLine(marker + name + ": " + valueStr + unit);
    }
}

