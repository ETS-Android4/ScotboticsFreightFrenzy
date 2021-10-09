package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Class for calculating the PID error correction in discrete time.
 * 
 * @deprecated
 */
public class PIDController {

    private final double projected, integral, derivative;
    private double errorPrior, integralPrior;
    private double expected = Double.NaN;
    private final ElapsedTime period = new ElapsedTime();

    public PIDController(final double projected, final double integral, final double derivative) {
        this.projected = projected;
        this.integral = integral;
        this.derivative = derivative;
    }

    public void Reset() {
        errorPrior = 0;
        integralPrior = 0;
        expected = Double.NaN;
        period.reset();
    }

    public double getCorrection(final double error) {
        final long deltaTime = period.nanoseconds();
        period.reset();
        return projected * error + integral * (integralPrior += error * deltaTime)
                + derivative * (-errorPrior + (errorPrior = error)) / deltaTime;
    }

    public double getCorrection(final double current, final double change) {
        return getCorrection((Double.isNaN(expected) ? 0 : expected - current)) + 0 * (expected = current + change);
    }
}