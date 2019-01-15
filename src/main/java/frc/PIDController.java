package frc;

public class PIDController {
    // lastUpdated is measured in miliseconds
    double kp, ki, kd, integral, lastUpdated, previousError, dt;
    public PIDController(double p, double i, double d) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        integral = 0;
        previousError = Double.NaN;
        dt = 0;
    }

    public void clear() {
        dt = 0;
        integral = 0;
        previousError = Double.NaN;
    }

    public double pid(double input, double setpoint) {
        double error = setpoint - input;
        double p = error * kp;
        integral += dt * error;
        double i = integral * ki;
        double d = Double.isNaN(previousError) ? 0 : ((error - previousError) / dt) * kd;
        previousError = error;
        return p + i + d;
    }
    
    public void updateTime(double dt) {
        this.dt = dt;
    }
}
