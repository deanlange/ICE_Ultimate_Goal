package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

/**
 * This opmode is a teleop POV style control to control the mecanum drive train
 * left stick Y controls forward and backward
 * right stick X controls turning
 * left stick X controls strafing
 */

@TeleOp(name = "TeleOp6340", group = "TeleOp")
public class TeleOp6340 extends OpMode {

    public DriveTrain driveTrain;


    DcMotor lf;
    DcMotor rf;
    DcMotor lr;
    DcMotor rr;

    @Override
    public void init() {
        lf = hardwareMap.dcMotor.get("leftFront");
        rf = hardwareMap.dcMotor.get("rightFront");
        lr = hardwareMap.dcMotor.get("leftRear");
        rr = hardwareMap.dcMotor.get("rightRear");


        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);


        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        driveTrain = new DriveTrain(lf, rf, lr, rr, false);

    }

    @Override
    public void loop() {

        //Driving
        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double rightStickX = gamepad1.right_stick_x;

        double[] drivePowers = driveTrain.calcWheelPowers(leftStickX, leftStickY, rightStickX);

        driveTrain.applyPower(drivePowers[0], drivePowers[1], drivePowers[2], drivePowers[3]);

        //Control Maps

    }

}