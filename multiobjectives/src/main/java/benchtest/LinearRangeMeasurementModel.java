package benchtest;

public class LinearRangeMeasurementModel {

    public static class MeasurementModel {
        public double[] Hk_til; // 1x4 vector
        public double Gk;       // Scalar

        public MeasurementModel(double[] Hk_til, double Gk) {
            this.Hk_til = Hk_til;
            this.Gk = Gk;
        }
    }

    public static MeasurementModel generateHk(double[] X) {
        // X = [x, y, vx, vy]

        double posX = X[0];
        double posY = X[1];

        double Gk = Math.sqrt(posX * posX + posY * posY);

        // Avoid division by zero
        if (Gk == 0) {
            throw new ArithmeticException("Range Gk is zero, cannot normalize.");
        }

        double partialPosX = posX / Gk;
        double partialPosY = posY / Gk;

        // Measurement Jacobian Hk_til (1x4)
        double[] Hk_til = new double[] { partialPosX, partialPosY, 0.0, 0.0 };

        return new MeasurementModel(Hk_til, Gk);
    }
}