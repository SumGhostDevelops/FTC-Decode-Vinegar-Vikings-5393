package org.firstinspires.ftc.teamcode.definitions.hardware;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.measure.angle.generic.Angle;

public class Subsystems
{
    // Final fields are now nullable since they might not be built
    public final Drive drive;
    public final Intake intake;
    public final Transfer transfer;
    public final Outtake outtake;
    public final Turret turret;
    public final Odometry odometry;

    private Subsystems(Builder builder)
    {
        this.drive = builder.drive;
        this.intake = builder.intake;
        this.transfer = builder.transfer;
        this.outtake = builder.outtake;
        this.turret = builder.turret;
        this.odometry = builder.odometry;
    }

    public static class Builder
    {
        private final RobotHardware hw;
        private final Team team;

        private Drive drive;
        private Intake intake;
        private Transfer transfer;
        private Outtake outtake;
        private Turret turret;
        private Odometry odometry;

        public Builder(RobotHardware hw, Team team)
        {
            this.hw = hw;
            this.team = team;
        }

        public Builder withAll()
        {
            return this
                    .withDrive()
                    .withIntake()
                    .withTransfer()
                    .withOuttake()
                    .withTurret()
                    .withOdometry();
        }

        /**
         * Initializes Odometry.
         * NOTE: Requires Turret to be built *before* Odometry if you want
         * automatic angular velocity compensation wired up immediately.
         */
        public Builder withOdometry()
        {
            if (hw.pinpoint != null)
            {
                this.odometry = new Odometry(hw.pinpoint, hw.webcam);
            }

            // Wiring Turret Compensation if both exist
            if (this.turret != null && this.odometry != null)
            {
                this.turret.setAngularVelocitySupplier(() -> this.odometry.getHeadingVelocity().getDegrees());
            }
            return this;
        }

        public Builder withTurret()
        {
            if (hw.turret != null)
            {
                Angle forwardAngle = RobotConstants.Turret.FORWARD_ANGLE;
                this.turret = new Turret(hw.turret, forwardAngle);
            }
            return this;
        }

        public Builder withOuttake()
        {
            if (hw.outtake != null)
            {
                this.outtake = new Outtake(hw.outtake);
            }
            return this;
        }

        public Builder withTransfer()
        {
            if (hw.transfer != null)
            {
                this.transfer = new Transfer(hw.transfer);
            }
            return this;
        }

        public Builder withIntake()
        {
            if (hw.intake != null)
            {
                this.intake = new Intake(hw.intake);
            }
            return this;
        }

        public Builder withDrive()
        {
            // Only build if hardware exists
            if (hw.frontLeft != null && hw.frontRight != null && hw.backLeft != null && hw.backRight != null)
            {
                this.drive = new Drive(hw.frontLeft, hw.frontRight, hw.backLeft, hw.backRight);
            }
            return this;
        }

        public Subsystems build()
        {
            return new Subsystems(this);
        }
    }
}