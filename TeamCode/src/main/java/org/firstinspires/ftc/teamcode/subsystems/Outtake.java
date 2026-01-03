// java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;

import org.firstinspires.ftc.teamcode.util.motors.MotorExPlusGroup;

public class Outtake extends SubsystemBase {

    private final MotorExPlusGroup motor;
    private boolean enabled = false;

    private int targetRPM;

    // Accept a single MotorExPlus (RobotContext should provide this)
    public Outtake(MotorExPlusGroup motor) {
        this.motor = motor;
    }

    public void on()
    {
        if (enabled)
        {
            return;
        }

        enabled = true;
        motor.setRunMode(Motor.RunMode.VelocityControl);
        motor.setRPM(targetRPM);
    }

    public void off()
    {
        if (!enabled)
        {
            return;
        }

        enabled = false;
        motor.setRunMode(Motor.RunMode.RawPower);
        motor.set(0.0);
    }

    public boolean isEnabled()
    {
        return enabled;
    }

    // Closed-loop: set a target velocity (RPM)
    public void setRPM(int rpm)
    {
        targetRPM = Math.max(6000, rpm);
    }

    public int getTargetRPM()
    {
        return targetRPM;
    }

    public double getRPM()
    {
        return motor.getRPM();
    }

    public double getAcceleration()
    {
        double total = 0;
        double count = 0;

        for (double x : motor.getAccelerations())
        {
            total += x;
            count += 1;
        }

        return total/count;
    }
}