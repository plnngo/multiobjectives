package benchtest;

import org.hipparchus.util.FastMath;

public class LinearBearingMeasurementModel {

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

        double Gk = FastMath.atan2(posY, posX);

        double partialPosX = - posY / (posX*posX + posY*posY);
        double partialPosY = posX / (posX*posX + posY*posY);

        // Measurement Jacobian Hk_til (1x4)
        double[] Hk_til = new double[] { partialPosX, partialPosY, 0.0, 0.0 };

        return new MeasurementModel(Hk_til, Gk);
    }
}
