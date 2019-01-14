package frc;

public class PIDController {
    // lastUpdated is measured in miliseconds
    double kp, ki, kd, integral, lastUpdated, previousError;
    public PIDController(double p, double i, double d) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        integral = 0;
        previousError = Double.NaN;
    }

    public void clear() {
        integral = 0;
        previousError = 0;
        lastUpdated = System.currentTimeMillis();
    }

    public double pid(double input, double setpoint) {
        double error = setpoint - input;
        double p = error * kp;
        integral += getTimeSinceLastUpdated() * error;
        double i = integral * ki;
        double d = previousError == Double.NaN ? 0 : ((error - previousError) / getTimeSinceLastUpdated()) * kd;
        lastUpdated = System.currentTimeMillis();
        previousError = error;
        return p + i + d;
    }

    private double getTimeSinceLastUpdated() {
        return (System.currentTimeMillis() - lastUpdated) / 1000.0;
    }
}
