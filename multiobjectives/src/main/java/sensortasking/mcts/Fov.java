package sensortasking.mcts;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.PVCoordinates;

import lombok.Getter;

@Getter
public class Fov {

    /** Field of view type. */
    Type type;

    /** Height or diameter in [rad] of FOV.  */
    private double height;

    /** Width or diameter in [rad] of FOV. */
    private double width;

    /** Azimuth range [rad]. */ 
    public double azMin = Double.NaN;
    public double azMax = Double.NaN;
    
    /** Elevation range [rad]. */ 
    public double elMin = Double.NaN;
    public double elMax = Double.NaN;
    
    /** Center of the FoV. */
    public double azCenter = Double.NaN;
    public double elCenter = Double.NaN;

    /** Optional: unit vector in 3D. */
    public double[] centerVec = new double[3]; // [x, y, z]

    /** FOV corners. */
    public double[][] corners;     // 4 corners x 3 dim vectors in x,y,z

    /** Chance of detection. */
    public double chanceOfDetect = 0.;

    /** Counter on how often this cell has been visited. */
    public int visitCount = 0;

    /**
     * Simple constructor.
     * 
     * @param type              Type of FOV, either circular or rectangular.
     * @param height            Height or diameter in [rad] of FOV. 
     * @param width             Width or diameter in [rad] of FOV.
     */
    public Fov(Type type, double height, double width) {

        this.type = type;
        this.height = height;
        this.width = width;
    }

    /**
     * Construct a rectangular FOV from angular locations.
     * 
     * @param azMin             Minimum azimuth angle [rad].
     * @param azMax             Maximum azimuth angle [rad].
     * @param elMin             Minimum elevation angle [rad].
     * @param elMax             Maxmim elevation angle [rad].
     */
    public Fov(double azMin, double azMax, double elMin, double elMax){
        this.azMin = azMin;
        this.azMax = azMax;
        this.elMin = elMin;
        this.elMax = elMax;
        this.azCenter = (azMin + azMax) / 2.0;
        this.elCenter = (elMin + elMax) / 2.0;

        this.height = elMax - elMin;
        this.width = azMax - azMin;
        this.type = Type.RECTANGULAR;
        this.centerVec = computeUnitVector(azCenter, elCenter);

         // corners in order (az,el): LL, LR, UR, UL
        this.corners = new double[][]{
            computeUnitVector(azMin, elMin),
            computeUnitVector(azMax, elMin),
            computeUnitVector(azMax, elMax),
            computeUnitVector(azMin, elMax)
        };
    }

    /** Field of view type. */
    public enum Type {
        /** Circular field of view. */
        CIRCULAR,

        /** Rectangular field of. */
        RECTANGULAR
    }

    /**
     * Compute unit pointing direction towards (az, el).
     * 
     * @param az    
     * @param el
     * @return
     */
    private double[] computeUnitVector(double az, double el) {
        double x = FastMath.cos(el) * FastMath.cos(az);
        double y = FastMath.cos(el) * FastMath.sin(az);
        double z = FastMath.sin(el);
        return new double[] {x, y, z};
    }

    /**
     * Compute shadow-based reward for a FoV cell, taking into account
     * the actual sensor position above the Earth.
     *
     * @param sensorPV          PVCoordinates of the sensor in ECI frame
     * @param cellDir           AngularDirection (azimuth/elevation) of the FoV cell in the 
     *                          sensor's local frame
     * @param date              AbsoluteDate for Sun position
     * @param rMin              Minimum range along the LOS [m]
     * @param rMax              Maximum range along the LOS [m]
     * @param nBins             Number of range bins
     * @param sensorToInertial  Transform from sensor local frame to ECI frame
     * 
     * @return                  Fraction of range bins NOT in Earth shadow (0–1)
     */
    protected static double computeShadowRewardFromSensor(PVCoordinates sensorPV,
                                                          AngularDirection cellDir,
                                                          AbsoluteDate date,
                                                          double rMin, double rMax,
                                                          int nBins,
                                                          Transform sensorToInertial) {

        Frame eci = FramesFactory.getEME2000();

        // Get the LOS unit vector in the sensor frame and transform it into ECI
        Vector3D losSensor = new Vector3D(cellDir.getAngle1(), cellDir.getAngle2());
        Vector3D losECI = sensorToInertial.transformVector(losSensor).normalize();

        Vector3D sensorPosECI = sensorPV.getPosition();

        double dr = (rMax - rMin) / (nBins - 1);
        int litCount = 0;

        for (int i = 0; i < nBins; i++) {
            double range = rMin + i * dr;

            // Compute Earth-centered position of this bin point
            Vector3D pointECI = sensorPosECI.add(losECI.scalarMultiply(range));
            double pointRange = pointECI.getNorm();

            // Angular direction from Earth center to the point
            AngularDirection earthToPoint = 
                new AngularDirection(eci, new double[]{pointECI.getAlpha(), pointECI.getDelta()}, 
                                     AngleType.RADEC, pointRange);

            int shadowFlag = checkInEarthShadowConical(earthToPoint, date);
            if (shadowFlag == 0) { // 0 = lit
                litCount++;
            }
        }

        return (double) litCount / nBins;
    }

    private static int checkInEarthShadowConical(AngularDirection earthToPoint, AbsoluteDate date) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'checkInEarthShadowConical'");
    }

    protected static double computeChanceForDetection(TopocentricFrame sensorFrame,
                                                      AbsoluteDate current,
                                                      double azCenter,
                                                      double elCenter,
                                                      AngularDirection sunDir,
                                                      AngularDirection moonDir) {

        AngularDirection azEl = new AngularDirection(sensorFrame,
                new double[]{azCenter, elCenter}, AngleType.AZEL, 1.);

        // Distance from Moon direction
        double actualDist = azEl.getEnclosedAngle(moonDir);
        double weightMoon = (actualDist > TrackingObjective.minMoonDist) ? 1.0 : 0.0;

        // Solar phase weight
        double phiMod = sunDir.getEnclosedAngle(azEl);
        double weightSolarPhase = (FastMath.sin(phiMod) - phiMod * FastMath.cos(phiMod)) / phiMod;

        // Airmass weight
        double airmass = 1. / (FastMath.sin(elCenter)
                + 0.025 * FastMath.exp(-11 * FastMath.sin(elCenter)));
        double airmassMax = 1. / (FastMath.sin(0.)
                + 0.025 * FastMath.exp(-11 * FastMath.sin(0.)));
        double weightAirmass = (1. / airmass - 1. / airmassMax) / (1 - 1. / airmassMax);

        return weightMoon * weightSolarPhase * weightAirmass;
    }

    protected static double computeFoVCellReward(Fov fov,
                                                 TopocentricFrame sensorFrame,
                                                 PVCoordinates sensorPV,
                                                 AbsoluteDate current,
                                                 AngularDirection sunDir,
                                                 AngularDirection moonDir,
                                                 double rMin,
                                                 double rMax,
                                                 int nBins,
                                                 Transform sensorToInertial) {

        // Visibility / photometric weight
        double visibilityReward = computeChanceForDetection(sensorFrame, current, 
                                                            fov.getAzCenter(), 
                                                            fov.getElCenter(), sunDir, 
                                                            moonDir);

        // Shadow / illumination weight
        AngularDirection cellDir = 
            new AngularDirection(sensorFrame, 
                                 new double[]{fov.getAzCenter(), fov.getElCenter()},
                                 AngleType.AZEL, 1.);

        double shadowReward = computeShadowRewardFromSensor(sensorPV, cellDir, current, rMin, rMax, 
                                                            nBins, sensorToInertial);

        // Combine (product by default, can use weights if desired)
        return visibilityReward * shadowReward;
    }
}
