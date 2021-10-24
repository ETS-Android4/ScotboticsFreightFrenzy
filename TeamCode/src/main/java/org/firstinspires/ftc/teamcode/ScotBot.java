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
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Main class representing the scotbot.
 */
@TeleOp(name = "SCOTBOT-DRIVE", group = "Pushbot")
public class ScotBot extends LinearOpMode {
    // Distance between wheels 15.5 in
    // Wheel circumference 4 pi in
    // Encoder ticks per rev 1440 ticks

    private double x, y, rotation;

    private DcMotor motorFL, motorFR, motorBL, motorBR, intakeL, intakeR, lift;
    private double posFL, posFR, posBL, posBR;

    private boolean aPrev = false;

    /**
     * Calculate the inverse sqrt root of a number
     *
     * @param x the number to calculate on
     * @return the result
     * @see https://stackoverflow.com/questions/11513344/how-to-implement-the-fast-inverse-square-root-in-java
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
        telemetry.addLine("Scotbot basic drive");

        waitForStart();

        initMotors();

        while (opModeIsActive()) {
            run();
            telemetry.update();
        }
    }

    /**
     * initializes motors.
     */
    private void initMotors() {
        motorFL = hardwareMap.get(DcMotor.class, "fl");
        motorFR = hardwareMap.get(DcMotor.class, "fr");
        motorBL = hardwareMap.get(DcMotor.class, "bl");
        motorBR = hardwareMap.get(DcMotor.class, "br");
        intakeL = hardwareMap.get(DcMotor.class, "il");
        intakeR = hardwareMap.get(DcMotor.class, "ir");
        lift = hardwareMap.get(DcMotor.class, "lift");

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(DcMotor.Direction.REVERSE);
        intakeR.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        motorFL.setPower(0);
        motorFR.setPower(0);
        motorBL.setPower(0);
        motorBR.setPower(0);
        intakeL.setPower(0);
        intakeR.setPower(0);
        lift.setPower(0);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Main loop of the bot.
     */
    private void run() {
        updateLocation();
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

        return power;
    }

    /**
     * Toggles the Intake.
     */
    private void intakeToggle() {
        intakeL.setPower(1.0 - intakeL.getPower());
        intakeR.setPower(1.0 - intakeR.getPower());
    }

    /**
     * Sets motor strength to reflect gamepad input.
     */
    private void drive() {
        if (gamepad1.left_stick_x * gamepad1.left_stick_x < 0.7) {
            double power = invSqrt(1 - gamepad1.left_stick_x * gamepad1.left_stick_x);

            if (gamepad1.left_stick_x < 0) {
                setPowerLeft(setPowerRight(-gamepad1.left_stick_y) * power);
            } else {
                setPowerRight(setPowerLeft(-gamepad1.left_stick_y) * power);
            }
        } else {
            setPowerRight(-setPowerLeft(Math.signum(gamepad1.left_stick_x)));
        }

        if (gamepad1.a && !aPrev) {
            intakeToggle();
        }
        aPrev = gamepad1.a;

        lift.setPower(gamepad1.right_stick_y * 0.7);
    }

    /**
     * Updates the bots location based of motor encoders.
     */
    private void updateLocation() {
        // Get motor change and update location
        double deltaBL = posBL - (posBL = motorBL.getCurrentPosition()),
                deltaBR = posBR - (posBR = motorBR.getCurrentPosition()),
                deltaFL = posFL - (posFL = motorFL.getCurrentPosition()),
                deltaFR = posFR - (posFR = motorFR.getCurrentPosition());

        double deltaLeft = (deltaBL + deltaFL) / 2d;
        double deltaRight = (deltaBR + deltaFR) / 2d;

        double deltaRot = (deltaLeft - deltaRight);// * TICKS_TO_INCHES; // TICKS_TO_INCHES was ENCODER_TO_RADIANS

        rotation += deltaRot;

        double deltaLoc = (deltaLeft + deltaRight) / 2d;
    }
}
