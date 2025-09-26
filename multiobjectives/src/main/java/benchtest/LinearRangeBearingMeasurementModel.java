package benchtest;

import org.hipparchus.util.FastMath;

public class LinearRangeBearingMeasurementModel {
    public static class MeasurementModel {
        public double[][] Hk_til; // 1x4 vector
        public double[] Gk;       // Scalar

        public MeasurementModel(double[][] Hk_til, double[] Gk) {
            this.Hk_til = Hk_til;
            this.Gk = Gk;
        }
    }
    public static MeasurementModel generateHk(double[] X) {
        // X = [x, y, vx, vy]

        double posX = X[0];
        double posY = X[1];

        //Gk first entry angle, second entry range
        double[] Gk = new double[]{FastMath.atan2(posY, posX), 
                                   FastMath.sqrt(posX * posX + posY * posY)};

        double H11 = - posY / (posX*posX + posY*posY);
        double H12 = posX / (posX*posX + posY*posY);
        double H21 = posX / Gk[1];
        double H22 = posY / Gk[1];

        // Measurement Jacobian Hk_til (2x4)
        double[][] Hk_til = new double[2][4];
        Hk_til[0] = new double[]{H11, H12, 0.0, 0.0};
        Hk_til[1] = new double[]{H21, H22, 0.0, 0.0};

        return new MeasurementModel(Hk_til, Gk);
    }
}
