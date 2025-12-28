package org.firstinspires.ftc.teamcode.subsystems.odometry;

import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Pinpoint;
import org.firstinspires.ftc.teamcode.subsystems.odometry.modules.Webcam;
import org.firstinspires.ftc.teamcode.util.measure.coordinate.Pose2d;

public class OdometryNew
{
    public final Webcam webcam;
    private final Pinpoint pinpoint;

    private Pose2d pose;

    public OdometryNew(Webcam webcam, Pinpoint pinpoint)
    {
        this.webcam = webcam;
        this.pinpoint = pinpoint;
    }
}
