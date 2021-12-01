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
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Main class representing the scotbot.
 */
@TeleOp(name = "TestBot-DRIVE", group = "Pushbot")
public class TestBot extends LinearOpMode {
    // Distance between wheels 15.5 in
    // Wheel circumference 4 pi in
    // Encoder ticks per rev 1440 ticks
    private static final double INTAKE_POWER=1.0, TURN_TABLE_POWER=0.5, SCOOP_SERVO_MAX=0.6;

    private double x, y, rotation;

    private DcMotor motorFL, motorFR, motorBL, motorBR, intakeL, intakeR, turnTable;
    private DcMotorEx lift;
    private Servo scoopServo;
    private int liftTarget = 0;
    private double scoopTarget=0;

    private boolean aPrev=false, bPrev=false, xPrev=false, yPrev=false, guidePrev =false;

    /**
     * Calculate the inverse sqrt root of a number
     *
     * @param x the number to calculate on
     * @return the result
     * from: https://stackoverflow.com/questions/11513344/how-to-implement-the-fast-inverse-square-root-in-java
     */
    public static double invSqrt(double x) {
        double xHalf = 0.5d * x;
        long i = Double.doubleToLongBits(x);
        i = 0x5fe6ec85e7de30daL - (i >> 1);
        x = Double.longBitsToDouble(i);
        x *= (1.5d - xHalf * x * x);
        return x;
    }

    @Override
    public void runOpMode() {
        telemetry.addLine("TestBot basic drive");

        waitForStart();

        initMotors();

        while (opModeIsActive()) {
            run();
            telemetry.update();
        }
    }

    /**
     * Initializes motors and servos.
     */
    private void initMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "fl");
        motorFR = hardwareMap.get(DcMotor.class, "fr");
        motorBL = hardwareMap.get(DcMotor.class, "bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");
        intakeL = hardwareMap.get(DcMotor.class, "il");
        intakeR = hardwareMap.get(DcMotor.class, "ir");
        turnTable = hardwareMap.get(DcMotor.class, "tt");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        scoopServo = hardwareMap.get(Servo.class, "scoop");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.REVERSE);
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        turnTable.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.REVERSE);
        scoopServo.setDirection(Servo.Direction.REVERSE);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        intakeL.setPower(0);
        intakeR.setPower(0);
        turnTable.setPower(0);
        lift.setPower(0);
        scoopServo.setPosition(0);

        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turnTable.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    /**
     * Main loop of the bot.
     */
    private void run() {
        drive();
    }

    /**
     * Sets power of left motors.
     *
     * @param power the power to the motors to
     * @return the power the motors are set to
     */
    private double setPowerLeft(double power) {
        motorBL.setPower(power);
        motorFL.setPower(power);

        //telemetry.addLine("Left:"+power);
        telemetry.addLine("Left encoder:"+motorFL.getCurrentPosition());

        return power;
    }

    /**
     * Sets power of right motors.
     *
     * @param power the power to the motors to
     * @return the power the motors are set to
     */
    private double setPowerRight(double power) {
        motorBR.setPower(power);
        motorFR.setPower(power);

        //telemetry.addLine("Right:"+power);
        telemetry.addLine("Right encoder:"+motorFR.getCurrentPosition());

        return power;
    }

    /**
     * Toggles the Intake.
     */
    private double intakeToggle(int i) {
        double power = 0.0;
        if (intakeL.getPower() != 0){
        } else if (i == 0){
            power = INTAKE_POWER;
        } else if (i ==1){
            power = -INTAKE_POWER;
        }
        intakeL.setPower(power);
        intakeR.setPower(power);
        return power;
    }

    /**
     * Toggles the Intake.
     */
    private double turnTableToggle(int i) {
        double power = 0.0;
        if (turnTable.getPower() != 0){
        } else if (i == 0){
            power = TURN_TABLE_POWER;
        } else if (i == 1){
            power = -TURN_TABLE_POWER;
        }
        turnTable.setPower(power);
        return power;
    }

    /**
     * Set lift power to move towards target.
     */
    private double updateLift()
    {
        //todo
        lift.setTargetPosition(liftTarget);
        telemetry.addLine("Lift Target: "+lift.getTargetPosition());
        telemetry.addLine("Lift Position: "+lift.getCurrentPosition());
        return -1.0;
    }

    /**
     * Sets motor strength to reflect gamepad input.
     */
    private void drive() {
        // wheel control
        if (gamepad1.left_stick_x * gamepad1.left_stick_x < gamepad1.left_stick_y * gamepad1.left_stick_y){
            setPowerRight(setPowerLeft(-gamepad1.left_stick_y * gamepad1.left_stick_y * Math.signum(gamepad1.left_stick_y))); //drive
        } else {
            setPowerRight(-setPowerLeft(gamepad1.left_stick_x * gamepad1.left_stick_x * gamepad1.left_stick_x)); //rotate
        }
        
        // intake control
        if (gamepad1.a && !aPrev) {
            intakeToggle(0);
        } else if (gamepad1.b && !bPrev) {
            intakeToggle(1);
        }
        aPrev = gamepad1.a;
        bPrev = gamepad1.b;

        // turntable control
        if (gamepad1.x && !xPrev) {
            turnTableToggle(0);
        } else if (gamepad1.y && !yPrev) {
            turnTableToggle(1);
        }
        xPrev = gamepad1.x;
        yPrev = gamepad1.y;

        // lift control
        double liftPower=gamepad1.right_stick_y * gamepad1.right_stick_y * gamepad1.right_stick_y * 0.7;
        lift.setPower(liftPower);
        //telemetry.addLine("lift: "+liftPower);
        telemetry.addLine("Lift encoder:"+lift.getCurrentPosition());

        // scoop control
        if (gamepad1.right_bumper) scoopTarget+=0.0005;
        if (gamepad1.left_bumper) scoopTarget-=0.0005;
        if (scoopTarget < 0.0) scoopTarget = 0;
        if (scoopTarget > SCOOP_SERVO_MAX) scoopTarget = SCOOP_SERVO_MAX;
        telemetry.addLine("Scoop value:"+scoopTarget);

        // lock to prep to lift
        if (gamepad1.guide && !guidePrev){
            scoopTarget = 0.05;
            intakeToggle(-1);
        }
        guidePrev = gamepad1.guide;

        //scoop control p2
        scoopServo.setPosition(scoopTarget);
    }
}
