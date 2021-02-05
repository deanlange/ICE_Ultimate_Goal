package org.firstinspires.ftc.teamcode.drive.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ICEMecanumDrive;
@Disabled

@Autonomous
public class BlueB extends LinearOpMode {
       @Override
    public void runOpMode() {
           ICEMecanumDrive drive = new ICEMecanumDrive(hardwareMap);

           Pose2d startPose = new Pose2d(-62,55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory targetZoneB = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d(-45.0, 55.0), Math.toRadians(0))
                   .splineTo(new Vector2d(34.0, 30.0), Math.toRadians(0))
                   .addDisplacementMarker(drive::releaseGoal)
                   .build();

           Trajectory bToGoal = drive.trajectoryBuilder(targetZoneB.end(),true)
                   .splineTo(new Vector2d(-30,29), Math.toRadians(180))
                   .addDisplacementMarker(drive::grabGoal)
                   .build();

           Trajectory goalToB = drive.trajectoryBuilder(bToGoal.end())
                   .splineTo(new Vector2d(40,15), Math.toRadians(270))
                   .addDisplacementMarker(drive::releaseGoal)
                   .build();

              Trajectory bToLine = drive.trajectoryBuilder(goalToB.end())
                      .splineTo(new Vector2d(10,15), Math.toRadians(180))
                      .build();


           init();


           waitForStart();

           if(isStopRequested()) return;

           drive.followTrajectory(targetZoneB);
           drive.releaseGoal();//Deploy Wobble Goal by setting servo to open
              drive.arm(.5);
              sleep(1000);
              drive.stopArm();
           drive.followTrajectory(bToGoal);
           drive.grabGoal();//Grab Goal by setting servo to close
           drive.followTrajectory(goalToB);
           drive.releaseGoal();
              sleep(500);//Release Goal by setting servo to open
           drive.followTrajectory(bToLine);

       }
}
