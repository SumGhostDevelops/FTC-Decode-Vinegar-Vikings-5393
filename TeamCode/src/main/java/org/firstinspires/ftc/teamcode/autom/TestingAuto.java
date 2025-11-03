package org.firstinspires.ftc.teamcode.autom;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;


@Autonomous(name="SampleAuto")

public class TestingAuto extends LinearOpMode {
    int WheelDiameter = 104; //mm
    double WheelCircumference = WheelDiameter * Math.PI;
    double rotation;

    double ticksPerRev = 537.7;

    @Override
    public void runOpMode() throws InterruptedException {

        //create starting pose
        Pose2d beginPose = new Pose2d(new Vector2d(0, 63), Math.toRadians(270));

        // create RoadRunner drive object
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

        waitForStart();

        // creating actual autonomous path
        Action path = drive.actionBuilder(beginPose)
                .splineToSplineHeading(new Pose2d(40, 0, Math.toRadians(180)), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(40, -48), Math.toRadians(0))
                .lineToXLinearHeading(0, Math.toRadians(90))
                .build();
        Actions.runBlocking(new SequentialAction(path));


    }
}
