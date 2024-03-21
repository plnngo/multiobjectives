package sensortasking.mcts;

import java.util.List;

import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

public class SpatialDensityModel {

    /** Reference sensor. */
    Sensor sensor;

    /** Reference epoch. */
    AbsoluteDate epoch;

    /** Topocentric horizontal frame with respect to reference sensor and epoch. */
    TopocentricFrame topoHorizon;

    /** Generated density model. */
    int[][] densityModel;

    /**
     * Constructor.
     * 
     * @param sensor            Reference sensor.
     * @param epoch             Reference epoch.
     */
    public SpatialDensityModel(Sensor sensor, AbsoluteDate epoch) {

        // Assign reference sensor and epoch
        this.sensor = sensor;
        this.epoch = epoch;

        // Set up topocentric horizon frame 
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               earthFrame);
        this.topoHorizon = new TopocentricFrame(earth, sensor.getPosition(), "station");

        // Discretise sensor's field of regard
        double height = sensor.getFov().getHeight();        // in [rad]
        double width = sensor.getFov().getWidth();          // in [rad]
        double cutOffEl = sensor.getElevCutOff();           // in [rad]

        int numPatchesEl = (int) Math.ceil(((FastMath.PI/2) - cutOffEl) / height);  // Number of patches along elevation axis
        int numPatchesAz = (int) Math.ceil((FastMath.PI*2)  / width);               // Number of patches along azimuth axis
        
        this.densityModel = new int[numPatchesEl][numPatchesAz];
    }

    
    /**
     * Generate density model of space environment with respect to {@link #sensor} and 
     * {@link #epoch}. 
     * 
     * @param tleSeries         Series of TLEs related to objects in space environment.
     * @return
     */
    protected int[][] createDensityModel(List<TLE> tleSeries){

        // Propagate TLE series to reference epoch
        for(TLE tle : tleSeries) {
            TLEPropagator propagator = TLEPropagator.selectExtrapolator(tle);
            SpacecraftState finalState = propagator.propagate(epoch);       // state in TEME
            PVCoordinates pvTeme = finalState.getPVCoordinates();
            PVCoordinates pvTopoHorizon = finalState.getFrame()
                                                    .getTransformTo(this.topoHorizon, epoch)
                                                    .transformPVCoordinates(pvTeme);

            // Extract angular position of space object 
            double azimuth = pvTopoHorizon.getPosition().getAlpha();        // between -Pi and +Pi
            double elevation = pvTopoHorizon.getPosition().getDelta();      // between -Pi/2 and +Pi/2
            AngularDirection azEl = new AngularDirection(topoHorizon, 
                                                         new double[]{azimuth, elevation}, 
                                                         AngleType.AZEL);

            // Extract indices in spatial density 2D array
            int[] indices = angularDirectionToGridPosition(azEl);
            int row = indices[0];
            int col = indices[1];

            // Increment number of object at corresponding patch position in spatial density model
            densityModel[row][col]++;
        }
        return densityModel;
    }


    /**
     * Retrieve position of the field whose angular position is the closest to {@code angles}.
     * 
     * @param angles            Angular position of the space object.
     * @return
     */
    private int[] angularDirectionToGridPosition(AngularDirection angles) {
        double azimuth = angles.getAngle1();
        double elevation =  angles.getAngle2();

        double heightFov = sensor.getFov().getHeight();
        double widthFov = sensor.getFov().getWidth();

        int row = (int) Math.round((elevation-sensor.getElevCutOff())/heightFov);
        int col = (int) Math.round(azimuth/widthFov);

        // Trow error if frame of angles is not the same as of the spatial density model
        if (!angles.getFrame().equals(topoHorizon)) {
            throw new IllegalArgumentException("Angular position is not defined in topocentric horizon frame.");
        }
        return new int[]{row, col};
    }
    
}
