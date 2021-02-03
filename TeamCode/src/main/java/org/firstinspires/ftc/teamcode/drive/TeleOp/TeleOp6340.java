package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ICEMecanumDrive;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "drive")
public class TeleOp6340 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ICEMecanumDrive drive = new ICEMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();

            if (gamepad1.dpad_up)
                drive.arm(-.5);
            else if (gamepad1.dpad_down)
                 drive.arm(.5);
            else
                drive.arm(0);

            if (gamepad1.right_bumper)
                drive.grabGoal();
            if (gamepad1.left_bumper)
                drive.releaseGoal();

            if (gamepad1.right_trigger>.5)
                drive.shootRings();
            else if
                (gamepad1.right_trigger<.5)
                drive.shooter.setVelocity(0);

            if (gamepad1.x)
                drive.intakeRings();
            if (gamepad1.y)
                drive.outakeRings();
            else drive.intake.setPower(0);




                telemetry.addData("targetVelocity", drive.shooter.getVelocity());

            telemetry.update();


        }
    }

}
