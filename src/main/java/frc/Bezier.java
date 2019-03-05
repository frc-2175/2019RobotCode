package frc;

public class Bezier {
    public double x0 = 0;
    public double y0 = 0;
    public double x1, y1, x2, y2, x3, y3;

    public Bezier(double targetx, double targety, double strength, double theta) {
        x3 = targetx;
        y3 = targety;
        x1 = 0;
        y1 = strength;
        x2 = targetx + Math.sin(Math.toRadians(theta)) * strength;
        y2 = targety - Math.cos(Math.toRadians(theta)) * strength;
    }

    public Vector[] generateBezier(int resolution) {
        Vector[] points = new Vector[resolution];
        for(int i = 0; i < resolution; i++) {
            points[i] = bezierParametric((double) i / (resolution - 1));
        }
        return points;
    }

    public Vector bezierParametric(double t) {
        double x = x0 + 3 * t * (x1 - x0) + 3 * Math.pow(t, 2) * (x0 + x2 - 2 * x1)
            + Math.pow(t, 3) * (x3 - x0 + 3 * x1 - 3 * x2);
        double y = y0 + 3 * t * (y1 - y0) + 3 * Math.pow(t, 2) * (y0 + y2 - 2 * y1)
            + Math.pow(t, 3) * (y3 - y0 + 3 * y1 - 3 * y2);
        return new Vector(x, y);
	}

	public static Vector[] getSamplePath() {
		Bezier bezier = new Bezier(36, 60, 12, 0);
		return bezier.generateBezier(30);
	}
}