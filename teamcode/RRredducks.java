package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
@Autonomous
public class RRredducks extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-15, -64, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder (new Pose2d(-15, -64, Math.toRadians(90)))
                /*.forward(30)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(90))
                .forward(30)
                .turn(Math.toRadians(90))*/
                //.forward(50)
                // .back(50)
                .splineTo(new Vector2d(-60,-50) ,Math.toRadians(90))
                //new Pose2d(-60,-60, Math.toRadians(90)))
                .build();
                //.forward(50)
                //.build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .back(50)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        drive.followTrajectory(traj1);
        //drive.followTrajectory(traj2);


    }
}