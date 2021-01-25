package org.firstinspires.ftc.teamcode.drive.Trajectories;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ICEMecanumDrive;

@Autonomous
public class BlueC extends LinearOpMode {
       @Override
    public void runOpMode() {
           ICEMecanumDrive drive = new ICEMecanumDrive(hardwareMap);

           Pose2d startPose = new Pose2d(-62,55, Math.toRadians(0));

           drive.setPoseEstimate(startPose);

           Trajectory targetZoneA = drive.trajectoryBuilder(startPose)
               .splineTo(new Vector2d(10, 55), Math.toRadians(0))
               .build();

           Trajectory aToGoal = drive.trajectoryBuilder(targetZoneA.end(),true)
                   .splineTo(new Vector2d(-24,31), Math.toRadians(180))
                   .build();

           Trajectory goalToA = drive.trajectoryBuilder(aToGoal.end())
                   .splineTo(new Vector2d(10,50), Math.toRadians(270))
                   .build();



           waitForStart();

           if(isStopRequested()) return;

           drive.followTrajectory(targetZoneA);
           //Deploy Wobble Goal by setting servo to open
           //Deploy Arm
           drive.followTrajectory(aToGoal);
           //Grab Goal by setting servo to close
           //drive.followTrajectory(goalToA);
           //Release Goal by setting servo to open

       }
}
