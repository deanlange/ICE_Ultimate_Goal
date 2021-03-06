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
public class BlueC extends LinearOpMode {
       @Override
    public void runOpMode() {
           ICEMecanumDrive drive = new ICEMecanumDrive(hardwareMap);

           Pose2d startPose = new Pose2d(-62,55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory targetZoneC = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d(58, 55), Math.toRadians(0))
                   .addDisplacementMarker(drive::releaseGoal)
                   .build();

           Trajectory cToGoal = drive.trajectoryBuilder(targetZoneC.end(),true)
                   .splineTo(new Vector2d(-30,29), Math.toRadians(180))
                   .addDisplacementMarker(drive::grabGoal)
                   .build();

           Trajectory goalToC = drive.trajectoryBuilder(cToGoal.end())
                   .splineTo(new Vector2d(58,45), Math.toRadians(270))
                   .addDisplacementMarker(drive::releaseGoal)
                   .build();

           Trajectory cToLine = drive.trajectoryBuilder(goalToC.end())
                   .splineTo(new Vector2d(10,50), Math.toRadians(180))
                   .build();

           init();
           

           waitForStart();

           if(isStopRequested()) return;

           drive.followTrajectory(targetZoneC);
           drive.releaseGoal();//Deploy Wobble Goal by setting servo to open
              drive.arm(.5);
              sleep(1000);
              drive.stopArm();
              drive.followTrajectory(cToGoal);
           drive.grabGoal();//Grab Goal by setting servo to close
           drive.followTrajectory(goalToC);
           drive.releaseGoal();//Release Goal by setting servo to open
              sleep(500);
              drive.followTrajectory(cToLine);

       }
}
