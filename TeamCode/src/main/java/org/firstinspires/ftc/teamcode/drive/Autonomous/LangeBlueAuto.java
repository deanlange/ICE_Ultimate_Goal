/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.drive.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.ICEMecanumDrive;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "LangeBlueAuto", group = "Autonomous")
public class LangeBlueAuto extends LinearOpMode {
    public static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    public static final String LABEL_FIRST_ELEMENT = "Quad";
    public static final String LABEL_SECOND_ELEMENT = "Single";
    private String targetZone = "A";


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZWzerv/////AAABmZeKo4MkD08MoSz5oHB/JU6N1BsUWpfHgQeAeVZemAypSUGVQhvAHo6+v7kJ3MITd8530MhwxRx7GjRtdCs1qjPmdKiJK66dv0yN4Zh4NvKBfP5p4TJjM+G0GoMVgVK0pItm2U56/SVqQH2AYtczQ+giw6zBe4eNhHPJCMY5C2t5Cs6IxxjZlMkRF85l8YAUlKGLipnoZ1T/mX8DNuThQA57qsIB2EN6pGWe8GI64hcPItQ0j7Oyjp82lEN13rYQYsS3Ur4a6//D6yhwa0rogXAysG68G+VgC1mNlj1CjX60qDI84ZN0b/A081xXqjeyFqZK8A/jO8y7BGz9ZuuZNxxXIon6xRNeKYudpfTD23+5";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    protected VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    protected TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 16/9).
            tfod.setZoom(2.5, 16.0 / 9.0);
        }

        /** Wait for the game to begin */
        ICEMecanumDrive drive = new ICEMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-62, 55, Math.toRadians(0));

        drive.setPoseEstimate(startPose);

        Trajectory targetZoneA = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(10, 55), Math.toRadians(0))
                .addDisplacementMarker(drive::releaseGoal)
                .build();

        Trajectory aToGoal = drive.trajectoryBuilder(targetZoneA.end(), true)
                .splineTo(new Vector2d(-30, 29), Math.toRadians(180))
                .addDisplacementMarker(drive::grabGoal)
                .build();

        Trajectory goalToA = drive.trajectoryBuilder(aToGoal.end())
                .splineTo(new Vector2d(10, 50), Math.toRadians(270))
                .addDisplacementMarker(drive::releaseGoal)
                .build();

        Trajectory targetZoneB = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(-45.0, 55.0), Math.toRadians(0))
                .splineTo(new Vector2d(34.0, 30.0), Math.toRadians(0))
                .addDisplacementMarker(drive::releaseGoal)
                .build();

        Trajectory bToGoal = drive.trajectoryBuilder(targetZoneB.end(), true)
                .splineTo(new Vector2d(-30, 29), Math.toRadians(180))
                .addDisplacementMarker(drive::grabGoal)
                .build();

        Trajectory goalToB = drive.trajectoryBuilder(bToGoal.end())
                .splineTo(new Vector2d(40, 15), Math.toRadians(270))
                .addDisplacementMarker(drive::releaseGoal)
                .build();

        Trajectory bToLine = drive.trajectoryBuilder(goalToB.end())
                .splineTo(new Vector2d(10, 15), Math.toRadians(180))
                .build();

        Trajectory targetZoneC = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(58, 55), Math.toRadians(0))
                .addDisplacementMarker(drive::releaseGoal)
                .build();

        Trajectory cToGoal = drive.trajectoryBuilder(targetZoneC.end(), true)
                .splineTo(new Vector2d(-30, 29), Math.toRadians(180))
                .addDisplacementMarker(drive::grabGoal)
                .build();

        Trajectory goalToC = drive.trajectoryBuilder(cToGoal.end())
                .splineTo(new Vector2d(58, 45), Math.toRadians(270))
                .addDisplacementMarker(drive::releaseGoal)
                .build();

        Trajectory cToLine = drive.trajectoryBuilder(goalToC.end())
                .splineTo(new Vector2d(10, 50), Math.toRadians(180))
                .build();

/** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
            ElapsedTime timeout = new ElapsedTime();
            while (timeout.time() < 5.0) {
                if (tfod != null) ;
                {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            // check label to see which target zone to go after.
                            if (recognition.getLabel().equals("Single")) {
                                telemetry.addData("Target Zone", "B");
                                targetZone = "B";
                            } else if (recognition.getLabel().equals("Quad")) {
                                telemetry.addData("Target Zone", "C");
                                targetZone = "C";
                            } else {
                                targetZone = "A";
                            }

                        }

                    }
                }
                telemetry.update();


            }

            if (tfod != null) {
                tfod.shutdown();
            }

//Autonomous Stuff Goes Here

                if (targetZone.equals("C")) {
                    drive.grabGoal();
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
                } else if (targetZone.equals("B")) {
                    drive.grabGoal();
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
                } else if (targetZone.equals("A")); {
                    drive.grabGoal();
                    drive.followTrajectory(targetZoneA);
                    drive.arm(.5);
                    sleep(1000);
                    drive.stopArm();
                    drive.followTrajectory(aToGoal);
                    drive.followTrajectory(goalToA);
                }

            }
        }


    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
            "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
       tfodParameters.minResultConfidence = 0.8f;
       tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
       tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }
}
