package sensortasking.mcts;

import org.hipparchus.util.FastMath;

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
}
