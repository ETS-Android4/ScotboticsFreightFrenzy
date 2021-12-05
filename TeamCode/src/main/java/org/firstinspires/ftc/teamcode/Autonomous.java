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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotorEx motorFL, motorFR, motorBL, motorBR, lift;
    private Servo scoopServo;
    private OpenCvWebcam webcam; //todo: see if I can stop webcam when dropOffHeight is found
    private int dropOffHeight = -1; //todo: set to -1
    private int[] possibleDropOffHeight = {0, 0};


    protected int STARTING_POSITION() {return -1;}

    @Override
    public void runOpMode() {
        telemetry.addLine("Autonomous w/ green detection, SP="+STARTING_POSITION());
        telemetry.update();
        initMotors();
        scoopServo.setPosition(0.05);
        initWebcam();
        telemetry.addLine("Autonomous w/ green detection, SP="+STARTING_POSITION());
        telemetry.addLine("webcam inited");
        telemetry.update();
        waitForStart();
        while (possibleDropOffHeight[0] != possibleDropOffHeight[1]){
        }
        webcam.stopStreaming();
        dropOffHeight=possibleDropOffHeight[0];
        telemetry.addLine("DOH"+dropOffHeight);
        telemetry.update();

        run();

        telemetry.addLine("FINISHING MOVE!");
        telemetry.update();
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        boolean bool= true;
        while (opModeIsActive()) {
            bool = finishingMove(bool);
            telemetry.update();
        }
    }

    private void initMotors() {
        motorFL = hardwareMap.get(DcMotorEx.class, "fl");
        motorFR = hardwareMap.get(DcMotorEx.class, "fr");
        motorBL = hardwareMap.get(DcMotorEx.class, "bl");
        motorBR = hardwareMap.get(DcMotorEx.class, "br");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        scoopServo = hardwareMap.get(Servo.class, "scoop");

        motorFL.setDirection(DcMotor.Direction.REVERSE);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);
        scoopServo.setDirection(Servo.Direction.REVERSE);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setPower(1);
        motorFR.setPower(1);
        motorBL.setPower(1);
        motorBR.setPower(1);

        lift.setTargetPosition(0);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        lift.setPower(1f);
        scoopServo.setPosition(0f);
    }

    private void drive(int ticks) {

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setTargetPosition(ticks);
        motorFR.setTargetPosition(ticks);
        motorBL.setTargetPosition(ticks);
        motorBR.setTargetPosition(ticks);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void rotate(int ticks) {
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setTargetPosition(-ticks);
        motorFR.setTargetPosition(ticks);
        motorBL.setTargetPosition(-ticks);
        motorBR.setTargetPosition(ticks);

        motorFL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorFR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorBR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private boolean driving(){
        return motorFL.isBusy() || motorFR.isBusy() || motorBL.isBusy() || motorBR.isBusy();
    }
    
    private void initWebcam() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.openCameraDevice();
        webcam.setPipeline(new Pipeline());
        webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);

    }

    /** Slowlyish moves scoop to goal position
     *  @param goal should be between 0.0 and ~0.6
     */
    private void dumpTo(double goal){
        double cur = scoopServo.getPosition();
        if(cur > goal){
            while (cur > goal) {
                cur -= 0.0005;
                scoopServo.setPosition(cur);
            }
        } else{
            while (cur < goal) {
                cur += 0.0005;
                scoopServo.setPosition(cur);
            }
        }
    }

    private void liftTo(int goal){ //todo val:power
        lift.setTargetPosition(goal);
        if(goal > lift.getCurrentPosition()){
            lift.setPower(1);
        } else {
            lift.setPower(0.5);
        }
    }

    class Pipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            Imgproc.rectangle(input, new Point(0, 100), new Point(input.width(), input.height()-100), new Scalar(0, 255, 0), 4);
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
            possibleDropOffHeight[1]=possibleDropOffHeight[0];
            if (l > m && l > r) { //todo: make sure numbers corespond to barcode location
                telemetry.addLine("Left");
                possibleDropOffHeight[0]=2;
            }
            else if (r < m) {
                telemetry.addLine("Middle");
                possibleDropOffHeight[0]=1;
            }
            else {
                telemetry.addLine("Right");
                possibleDropOffHeight[0]=0;
            }
            telemetry.addLine(""+possibleDropOffHeight[0]);
            telemetry.addLine(""+possibleDropOffHeight[1]);
            telemetry.update();
            return input;
        }
    }
    
    private void run(){//TODO: make wheels stop spinning before next command
        //setup
        int sp = STARTING_POSITION();
        int sign;
        int[] opt;
        int[] mod;

        //lift in preparation to move
        mod = new int[]{0,0,0,0}; //todo: val
        liftTo(1480 + mod[sp]); //todo: val //^ move over turn table wheel

        //move next to the shipping hub
        mod = new int[]{0,0,0,0}; //todo: val
        drive(4400+mod[sp]); //todo: val
        while(driving());

        //rotate so back is facing shipping hub
        mod = new int[]{0,0,0,0}; //todo: val
        sign = (0==((sp^(sp>>1))&1) ? -1 : 1); //todo: sign
        rotate(sign*(1900+mod[sp])); //todo: val
        while(driving());

        //back up to shipping hub
        mod = new int[]{0, 0, 0, 0}; //todo: val
        opt=new int[]{100,0, 0};//todo:val
        drive(opt[dropOffHeight]+mod[sp]); //todo: val
        while(driving() || lift.isBusy());

        //move lift to correct height
        dumpTo(.4); //todo: val
        sleep(1000);
        mod = new int[]{0,0,0,0}; //todo: val
        opt = new int[]{-50, 650, 1480};
        liftTo(opt[dropOffHeight] + mod[sp]); //todo: val //^ move over turn table wheel
        while (lift.isBusy());

        //dumping precedure
        dumpTo(.6); //todo: val
        sleep(1500);
        dumpTo(0.05); //todo: val
        liftTo(1000);

        //undo back up to shipping hub
        mod = new int[]{0, 0, 0, 0}; //todo: val
        opt=new int[]{-100,0,0};//todo:val
        drive(opt[dropOffHeight]+mod[sp]); //todo: val
        while(driving());

        //rotate other way
        mod = new int[]{0,0,0,0}; //todo: val
        rotate(-sign*(1900+mod[sp])); //todo: val
        while(driving());

        //back up to get in warehouse (towards drivers)
        mod = new int[]{0,0,0,0}; //todo: val
        drive(-2400+mod[sp]); //todo: val
        while(driving());

        //rotate to warehouse
        mod = new int[]{0,0,0,0}; //todo: val
        sign = (0==(sp&2) ? 1 : -1);
        rotate(sign*(1900+mod[sp])); //todo: val

        if (0==(sp&1)) {
            //drive close to warehouse
            while(driving());
            mod = new int[]{0, 0, 0, 0}; //todo: val
            opt = new int[]{5440, 0};
            drive(opt[sp % 2] + mod[sp]); //todo: val
        }
        while (driving() || lift.isBusy());
    }

    private boolean finishingMove(boolean bool){ //todo: execution
        double high=1.0;//todo val
        double low=0.8;//todo val
        double left;
        double right;
        if(bool){
            left=high;
            right=low;
        } else {
            left=low;
            right=high;
        }
        motorFL.setPower(left);
        motorFR.setPower(right);
        motorBL.setPower(left);
        motorBR.setPower(right);
        return !bool;
    }
}