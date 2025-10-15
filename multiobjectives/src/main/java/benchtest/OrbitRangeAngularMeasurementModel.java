package benchtest;

import org.hipparchus.util.FastMath;

public class OrbitRangeAngularMeasurementModel {

    public static class MeasurementModel {
        public double[][] Hk_til; // 3x6 vector
        public double[] Gk;       // ra, dec, range

        public MeasurementModel(double[][] Hk_til, double[] Gk) {
            this.Hk_til = Hk_til;
            this.Gk = Gk;
        }
    }

    /**
     * 
     * @param X         State vector.
     * @return          MeasurementModel containing observation matrix and simulated measurement.
     */
    public static MeasurementModel generateHk(double[] X) {
        // X = [x, y, z, vx, vy, vz]

        // Extract position
        double posX = X[0];
        double posY = X[1];
        double posZ = X[2];
        double r = FastMath.sqrt(posX*posX + posY*posY + posZ*posZ);

        // Extract auxilary parameter
        double r2 = r*r;
        double r3 = r*r*r;
        double posX2 = posX * posX;
        double posY2 = posY * posY;
        double posZ2 = posZ * posZ;

        //Gk first entry right ascension, second entry declination, third entry range
        double ra = FastMath.atan2(posY, posX);
        double dec = FastMath.asin(posZ/r);
        double[] Gk = new double[]{ra, dec, r};

        double H11 = - posY / (posX2 + posY2);
        double H12 = posX / (posX2 + posY2);
        double H21 = - posZ * posX/(r2*FastMath.sqrt(posX2 + posY2));
        double H22 = - posZ * posY/(r2*FastMath.sqrt(posX2 + posY2));
        double H23 = (1/r - posZ2/r3)/(FastMath.sqrt(1-posZ2/r2));
        double H31 = posX / r;
        double H32 = posY / r;
        double H33 = posZ / r;

        // Measurement Jacobian Hk_til (2x4)
        double[][] Hk_til = new double[Gk.length][X.length];
        Hk_til[0] = new double[]{H11, H12, 0.0, 0.0, 0.0, 0.0};
        Hk_til[1] = new double[]{H21, H22, H23, 0.0, 0.0, 0.0};
        Hk_til[2] = new double[]{H31, H32, H33, 0.0, 0.0, 0.0};

        return new MeasurementModel(Hk_til, Gk);
    }
}
