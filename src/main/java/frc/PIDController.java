package frc;

public class PIDController {
    double kp, ki, kd, integral, previousError, dt;

    /**
     * Constructs a new pid controller with constants
     * @param p proportional gain
     * @param i integral gain
     * @param d derivative gain
     */
    public PIDController(double p, double i, double d) {
        this.kp = p;
        this.ki = i;
        this.kd = d;
        integral = 0;
        previousError = Double.NaN;
        dt = 0;
    }

    /** 
     * At any time you want to re-use a PID controller with new
     * constants, call this method.
     */
    public void clear() {
        dt = 0;
        integral = 0;
        previousError = Double.NaN;
    }

    /**
     * Runs a step of the PID loop with a certain input and setpoint
     * @param input the input to the controller
     * @param setpoint the desired setpoint
     * @return the output of the loop
     */
    public double pid(double input, double setpoint) {
        double error = setpoint - input;
        double p = error * kp;
        integral += dt * error;
        double i = integral * ki;
        double d = Double.isNaN(previousError) ? 0 : ((error - previousError) / dt) * kd;
        previousError = error;
        return p + i + d;
    }
    
    /**
     * Every time the loop goes forward by one interation, call this
     * method with a new dt.
     * @param dt change in time from the last time this PID controller
     * was run
     */
    public void updateTime(double dt) {
        this.dt = dt;
    }
}
