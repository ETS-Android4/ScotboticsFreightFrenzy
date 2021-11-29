/* Copyright (c) 2017 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Autonomous extends LinearOpMode {
    protected int startingPosition=0;
    private DcMotorEx motorFL, motorFR, motorBL, motorBR;
    private OpenCvWebcam webcam;
    private int dropOffHeight;

    @Override
    public void runOpMode() {
        telemetry.addLine("Scotbot basic drive");

        waitForStart();

        initMotors();
        initWebcam();

        drive(-100*startingPosition);
        while ((motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() || motorBR.isBusy()) && opModeIsActive())
            telemetry.update();

        rotate(100*startingPosition);
        while ((motorFL.isBusy() || motorBR.isBusy() || motorBL.isBusy() || motorBR.isBusy()) && opModeIsActive())
            telemetry.update();

        while (opModeIsActive()) {
            telemetry.update();
        }

        webcam.stopStreaming();
    }

    private void initMotors() {
        motorFL = hardwareMap.get(DcMotorEx.class, "fl");
        motorFR = hardwareMap.get(DcMotorEx.class, "fr");
        motorBL = hardwareMap.get(DcMotorEx.class, "bl");
        motorBR = hardwareMap.get(DcMotorEx.class, "br");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
    }

    private void drive(double dist) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setTargetPosition((int) dist);
        motorFR.setTargetPosition((int) dist);
        motorBL.setTargetPosition((int) dist);
        motorBR.setTargetPosition((int) dist);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(1f);
        motorFR.setPower(1f);
        motorBL.setPower(1f);
        motorBR.setPower(1f);
    }

    private void rotate(double dist) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setTargetPosition((int) dist);
        motorFR.setTargetPosition((int) dist);
        motorBL.setTargetPosition(-(int) dist);
        motorBR.setTargetPosition(-(int) dist);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorFL.setPower(1);
        motorFR.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);
    }


    private void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        telemetry.addData("Webcam", webcam);

        webcam.openCameraDevice();
        webcam.setPipeline(new Pipeline());
        webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

    }


    class Pipeline extends OpenCvPipeline {
        private Boolean heightSet = false;

        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input, new Point(0, 100), new Point(input.width(), input.height()-100), new Scalar(0, 255, 0), 4);


            if (!heightSet || true) {
                double l = 0, m = 0, r = 0;

                for (int i = 0; i < input.width() / 3; i++) {
                    for (int j = 100; j < input.height()-100; j++) {
                        double[] p = input.get(j, i);
                        l += p[1]-p[0]-p[2];
                        p = input.get(j, i + input.width() / 3);
                        m += p[1]-p[0]-p[2];
                        p = input.get(j, i + input.width() * 2 / 3);
                        r += p[1]-p[0]-p[2];
                    }
                }
                dropOffHeight = 0;
                if (l > m && l > r) telemetry.addLine("Left");
                else if (r < m) telemetry.addLine("Middle");
                else telemetry.addLine("Right");
                heightSet = true;
            }

            return input;
        }
    }
}