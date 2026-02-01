package org.firstinspires.ftc.teamcode.definitions.hardware;

import org.firstinspires.ftc.teamcode.definitions.constants.RobotConstants;
import org.firstinspires.ftc.teamcode.definitions.constants.Team;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.odometry.Odometry;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

public class Subsystems
{
    public final Drive drive;
    public final Intake intake;
    public final Transfer transfer;
    public final Outtake outtake;
    public final Turret turret;

    public Odometry odometry;

    public Subsystems(RobotHardware hw, Team team)
    {
        this.drive = new Drive(hw.frontLeft, hw.frontRight, hw.backLeft, hw.backRight);
        this.intake = new Intake(hw.intake);
        this.transfer = new Transfer(hw.transfer);
        this.outtake = new Outtake(hw.outtake);
        this.turret = new Turret(hw.turret, RobotConstants.Turret.FORWARD_ANGLE);
        this.odometry = new Odometry(hw.pinpoint, hw.webcam, new Pose2d(team.base.coord, team.forwardAngle));

        // Wire turret rotation compensation from odometry angular velocity
        this.turret.setAngularVelocitySupplier(() -> this.odometry.getHeadingVelocity().getDegrees());
    }
}