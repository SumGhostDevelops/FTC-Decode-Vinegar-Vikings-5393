package org.firstinspires.ftc.teamcode.opmodes.experiments;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

import org.firstinspires.ftc.teamcode.definitions.RobotHardware;

@TeleOp(name = "Actuator Testing", group = "Experiments")
@Disabled
public class ActuatorTesting extends LinearOpMode
{
    private final MotorSelection[] motorOptions = MotorSelection.values();
    private RobotHardware robot;
    private int menuIndex = 0;                          // Current menu position
    private MotorSelection selectedMotor = MotorSelection.NONE;  // Active motor (controlled by triggers)
    private double testPower = 0.5;                     // Power level when triggered
    // Input debounce state
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        initSystems();

        telemetry.addLine("=== Actuator Testing ===");
        telemetry.addLine("Controls:");
        telemetry.addLine("D-Pad Left/Right: Navigate menu");
        telemetry.addLine("A: Select/Deselect motor");
        telemetry.addLine("D-Pad Up/Down: Adjust power");
        telemetry.addLine("Right Trigger: Move forward");
        telemetry.addLine("Left Trigger: Move backward");
        telemetry.addLine("");
        telemetry.addLine("Safety: NONE is selected by default");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            handleInput();
            runMotor();
            displayTelemetry();
        }

        // Stop all motors on exit
        stopAllMotors();
    }

    private void initSystems()
    {
        robot = new RobotHardware(hardwareMap, telemetry);

        // Set all motors to RawPower run mode for direct control
        for (MotorSelection selection : motorOptions)
        {
            MotorEx motor = getMotorForSelection(selection);
            if (motor != null)
            {
                try
                {
                    motor.setRunMode(Motor.RunMode.RawPower);
                }
                catch (Exception ignored)
                {
                }
            }
        }
    }

    private void handleInput()
    {
        // Menu navigation (left/right)
        if (gamepad1.dpad_right && !lastRight)
        {
            menuIndex = (menuIndex + 1) % motorOptions.length;
        }
        if (gamepad1.dpad_left && !lastLeft)
        {
            menuIndex = (menuIndex - 1 + motorOptions.length) % motorOptions.length;
        }
        lastRight = gamepad1.dpad_right;
        lastLeft = gamepad1.dpad_left;

        // Select/deselect motor (A button)
        if (gamepad1.a && !lastA)
        {
            MotorSelection menuMotor = motorOptions[menuIndex];
            if (selectedMotor == menuMotor)
            {
                // Deselect current motor (return to NONE/safety)
                selectedMotor = MotorSelection.NONE;
            }
            else
            {
                // Check if motor is available before selecting
                MotorEx motor = getMotorForSelection(menuMotor);
                if (menuMotor == MotorSelection.NONE || motor != null)
                {
                    selectedMotor = menuMotor;
                }
            }
        }
        lastA = gamepad1.a;

        // Power adjustment (up/down) - only when a motor is selected
        if (selectedMotor != MotorSelection.NONE)
        {
            if (gamepad1.dpad_up && !lastUp)
            {
                testPower = Math.min(1.0, testPower + 0.05);
            }
            if (gamepad1.dpad_down && !lastDown)
            {
                testPower = Math.max(0.0, testPower - 0.05);
            }
        }
        lastUp = gamepad1.dpad_up;
        lastDown = gamepad1.dpad_down;
    }

    private void runMotor()
    {
        // Get the selected motor
        MotorEx motor = getMotorForSelection(selectedMotor);

        if (motor == null)
        {
            // No motor selected or motor not available - ensure nothing runs
            return;
        }

        // Read triggers
        boolean rightTrigger = gamepad1.right_trigger > 0.25;
        boolean leftTrigger = gamepad1.left_trigger > 0.25;

        // Handle conflicts: both pressed or neither pressed = stop
        if (rightTrigger && leftTrigger)
        {
            motor.set(0);
        }
        else if (rightTrigger)
        {
            motor.set(testPower);
        }
        else if (leftTrigger)
        {
            motor.set(-testPower);
        }
        else
        {
            motor.set(0);
        }
    }

    private MotorEx getMotorForSelection(MotorSelection selection)
    {
        switch (selection)
        {
            case FRONT_LEFT:
                return robot.frontLeft;
            case FRONT_RIGHT:
                return robot.frontRight;
            case BACK_LEFT:
                return robot.backLeft;
            case BACK_RIGHT:
                return robot.backRight;
            case INTAKE:
                return robot.intake;
            case TURRET:
                return robot.turret;
            case OUTTAKE_LEFT:
                return robot.outtake;
            case NONE:
            default:
                return null;
        }
    }

    private void displayTelemetry()
    {
        telemetry.addLine("=== Actuator Testing ===");
        telemetry.addLine("");

        // Menu display
        telemetry.addData("Menu Position", motorOptions[menuIndex].name());
        MotorEx menuMotor = getMotorForSelection(motorOptions[menuIndex]);
        if (motorOptions[menuIndex] != MotorSelection.NONE && menuMotor == null)
        {
            telemetry.addLine("  [!] Motor not available");
        }

        telemetry.addLine("-----------------");

        // Selected motor display
        telemetry.addData("SELECTED MOTOR", selectedMotor.name());
        if (selectedMotor == MotorSelection.NONE)
        {
            telemetry.addLine("  (Safety mode - no motor controlled)");
        }

        telemetry.addLine("-----------------");

        // Power level
        telemetry.addData("Test Power", "%.2f", testPower);

        // Trigger state
        boolean rightTrigger = gamepad1.right_trigger > 0.25;
        boolean leftTrigger = gamepad1.left_trigger > 0.25;

        String state;
        if (selectedMotor == MotorSelection.NONE)
        {
            state = "SAFE (no motor)";
        }
        else if (rightTrigger && leftTrigger)
        {
            state = "CONFLICT (stopped)";
        }
        else if (rightTrigger)
        {
            state = "FORWARD";
        }
        else if (leftTrigger)
        {
            state = "BACKWARD";
        }
        else
        {
            state = "STOPPED";
        }
        telemetry.addData("Motor State", state);

        // Motor velocity if available
        MotorEx selectedMotorEx = getMotorForSelection(selectedMotor);
        if (selectedMotorEx != null)
        {
            try
            {
                telemetry.addData("Motor Velocity", "%.1f", selectedMotorEx.getVelocity());
            }
            catch (Exception e)
            {
                telemetry.addData("Motor Velocity", "N/A");
            }
        }

        telemetry.addLine("-----------------");
        telemetry.addLine("Controls: L/R=Navigate, A=Select, U/D=Power");

        telemetry.update();
    }

    private void stopAllMotors()
    {
        for (MotorSelection selection : motorOptions)
        {
            MotorEx motor = getMotorForSelection(selection);
            if (motor != null)
            {
                try
                {
                    motor.set(0);
                }
                catch (Exception ignored)
                {
                }
            }
        }
    }

    // Motor selection options
    private enum MotorSelection
    {
        NONE,           // Safety default
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT,
        INTAKE,
        TURRET,
        OUTTAKE_LEFT,
        OUTTAKE_RIGHT
    }
}