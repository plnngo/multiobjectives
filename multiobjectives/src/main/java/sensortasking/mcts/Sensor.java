package sensortasking.mcts;

import java.util.InputMismatchException;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import lombok.Getter;

@Getter
public class Sensor {

    /** Name of the sensor. */
    private String name;

    /** Field of view in [rad]. */
    private Fov fov;

    /** Geodetic point of the sensor. */
    private GeodeticPoint position;

    /** Exposure duration in [s]. */
    private double exposureT;

    /** Read-out duration in [s]. */
    private double readoutT;

    /** Slew velocity in [rad/s]. */
    private double slewVel;

    /** Elevation cut off angle in [rad] */
    private double elevCutOff;

    /** Settling duration in [s] after telescope has been slewed. */
    private double settlingT;

    /** Checker if slew velocity incorporates slew and settling time. */
    private boolean slewVelInclSensorSettle;

    /**
     * Constructor. 
     * Slewing and setting time is already incorprated in slew velocity parameter.
     * 
     * @param name                  Name of the sensor.
     * @param fov                   Field of view.
     * @param pos                   Geodetic position of sensor.
     * @param exposureT             Exposure duration in [s].
     * @param readoutT              Read-out duration in [s].
     * @param slewVel               Slew velocity in [rad/s]
     * @param elevCutOff            Elevation cut off angle in [rad}]
     */
    public Sensor(String name, Fov fov, GeodeticPoint pos, double exposureT, 
                  double readoutT, double slewVel, double elevCutOff) {

        this.name = name;
        this.fov = fov;
        this.position = pos;
        this.exposureT = exposureT;
        this.readoutT = readoutT;
        this.slewVel = slewVel;    
        this.elevCutOff = elevCutOff; 
        this.slewVelInclSensorSettle = true;             
    }

    /**
     * Constructor. 
     * Only slewing time is incorprated in slew velocity parameter.
     * 
     * @param name                  Name of the sensor.
     * @param fov                   Field of view.
     * @param pos                   Geodetic position of sensor.
     * @param exposureT             Exposure duration in [s].
     * @param readoutT              Read-out duration in [s].
     * @param slewVel               Slew velocity in [rad/s]
     * @param settlingT             Settling duration in [s].
     * @param elevCutOff            Elevation cut off angle in [rad}]
     */
    public Sensor(String name, Fov fov, GeodeticPoint pos, double exposureT, 
                  double readoutT, double slewVel, double settlingT, double elevCutOff) {

        this.name = name;
        this.fov = fov;
        this.position = pos;
        this.exposureT = exposureT;
        this.readoutT = readoutT;
        this.slewVel = slewVel;    
        this.elevCutOff = elevCutOff;     
        this.settlingT = settlingT;      
        this.slewVelInclSensorSettle = false;   
    }

    /**
     * Compute reposition time between two pointing directions under the assumption that the 
     * sensor takes the shortest path to reach the aimed pointing direction.
     * Both pointing directions need to be converted into a common reference frame.
     * The duration incorporates slewing and settling time.
     * 
     * @param origin                Original pointing direction.
     * @param dest                  Aimed pointing direction.
     * 
     * @return                      Reposition duration in [s].
     */
    public double computeRepositionT(AngularDirection origin, AngularDirection dest/*, 
                                     AbsoluteDate date*/) {
        // TODO: maybe overthink the need of date as input(only necessary to transform origin and dest into common frame)

        //Frame topoFrame = getTopoInertialFrame(date);        
        //Frame gcrf = FramesFactory.getGCRF();

/*         if(origin.getFrame() !=  topoFrame || dest.getFrame() != topoFrame) {
            originTopo = origin.transformRefernce(topoFrame, date, AngleType.RADEC);
            destTopo = dest.transformRefernce(topoFrame, date, AngleType.RADEC);
        } */

        if(origin.getFrame() != dest.getFrame()) {
            throw new InputMismatchException("Pointing directions were not defined in the same " 
                                                + "frame. Transformation necessary.");
        }

        // Transform angular direction into unit vector
        double xOrigin = FastMath.cos(origin.getAngle1()) 
                            * FastMath.sin(FastMath.PI/2 - origin.getAngle2());
        double yOrigin = FastMath.sin(origin.getAngle1())
                            * FastMath.sin(FastMath.PI/2 - origin.getAngle2());
        double zOrigin = FastMath.cos(FastMath.PI/2 - origin.getAngle2());
        Vector3D posOrigin = new Vector3D(xOrigin, yOrigin, zOrigin);

        double xDest = FastMath.cos(dest.getAngle1()) 
                            * FastMath.sin(FastMath.PI/2 - dest.getAngle2());
        double yDest = FastMath.sin(dest.getAngle1())
                            * FastMath.sin(FastMath.PI/2 - dest.getAngle2());
        double zDest = FastMath.cos(FastMath.PI/2 - dest.getAngle2());
        Vector3D posDest = new Vector3D(xDest, yDest, zDest);

        // Angle in [rad] between origin and destination vectors
        double theta = FastMath.acos(Vector3D.dotProduct(posOrigin, posDest));

        if(slewVelInclSensorSettle) {
            return theta/this.slewVel;
        } else {
            return (theta/this.slewVel) + settlingT;
        }
    }

    /**
     * Create topocentric inertial reference frame.
     * 
     * @param date                  Date related to the topocentric frame.
     * 
     * @return                      Topocentric frame.
     */
    public Frame getTopoInertialFrame(AbsoluteDate date) {
        

        // Get sensor station in ECI coordinates
        Vector3D posEci = getSensorPosEci(date);

        // The transform vector from ECI to topocentric frame is not the vector from ECI's origin 
        // to topocentric origin (expressed in ECI) but rather the negative
        Transform geo2topoEci = new Transform(date, posEci.negate());
        Frame topoFrame = new Frame(FramesFactory.getGCRF(), geo2topoEci, "Topocentric-Inertial", 
                     true);

        return topoFrame;
    }

    /**
     * Transform sensor's geodetic position into Cartesian coordinates in inertial frame (GCRF).
     * 
     * @param date                  Date related to the frame transformation from ECEF to GCRF.
     * 
     * @return                      Cartesian position of sensor in GCRF.
     */
    public Vector3D getSensorPosEci(AbsoluteDate date){

        // Get sensor station in ECEF coordinates
        Vector3D posEcef = getSensorPosEcef();

        // Transform position from ECEF to ECI
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame eci = FramesFactory.getGCRF();
        //Frame eci = FramesFactory.getEME2000();
        Transform ecef2eci = ecef.getTransformTo(eci, date);  
        return ecef2eci.transformPosition(posEcef);
    }

    /**
     * Transform sensor's geodetic position into Cartesian coordinates in the ECEF frame.
     *  
     * @return                      Cartesian position of sensor in ECEF.
     */
    public Vector3D getSensorPosEcef(){

        double lat = position.getLatitude();
        double lon = position.getLongitude();
        double alt = position.getAltitude();

        // Earth geometry
        double re = Constants.WGS84_EARTH_EQUATORIAL_RADIUS;
        double f = Constants.WGS84_EARTH_FLATTENING;
        double ecc = FastMath.sqrt(2*f - FastMath.pow(f, 2));

        // Radius of curvature in the meridian
        double C = re/FastMath.sqrt(1 - FastMath.pow(ecc, 2) * FastMath.pow(FastMath.sin(lat), 2));

        double S = (1-FastMath.pow(ecc, 2)) * C ;

        // Transform Geodetic point into position in ECEF
        double xEcef = (C + alt) * FastMath.cos(lat) * FastMath.cos(lon);
        double yEcef = (C + alt) * FastMath.cos(lat) * FastMath.sin(lon);
        double zEcef = (S + alt) * FastMath.sin(lat);

        return new Vector3D(xEcef, yEcef, zEcef);
    }
}
