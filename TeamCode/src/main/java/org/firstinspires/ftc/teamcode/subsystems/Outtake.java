// java
package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlus;
import org.firstinspires.ftc.teamcode.util.motors.MotorExPlusGroup;

public class Outtake extends SubsystemBase {

    private final MotorExPlusGroup motor;

    // Accept a single MotorExPlus (RobotContext should provide this)
    public Outtake(MotorExPlusGroup motor) {
        this.motor = motor;
    }

    // Open-loop power control (keeps previous name)
    public void in(double power) {
        motor.set(power);
    }

    // Closed-loop: set a target velocity (RPM)
    public void setTargetVelocity(double rpm) {
        motor.setRPM(rpm);
    }

    // Read current motor velocity (RPM)
    public double getVelocity() {
        return motor.getRPM();
    }

    // Convenience check whether current velocity is within tolerance of target
    public boolean isAtVelocity(double targetRpm, double toleranceRpm) {
        return Math.abs(getVelocity() - targetRpm) <= toleranceRpm;
    }

    public void stop() {
        motor.set(0.0);
    }
}
