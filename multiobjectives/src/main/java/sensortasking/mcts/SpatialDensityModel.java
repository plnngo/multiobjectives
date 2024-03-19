package sensortasking.mcts;

import java.util.List;

import org.hipparchus.util.FastMath;
import org.orekit.frames.Frame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;

public class SpatialDensityModel {

    /** Reference sensor. */
    Sensor sensor;

    /** Reference epoch. */
    AbsoluteDate epoch;

    /** Topocentric inertial frame with respect to reference sensor and epoch. */
    Frame topoInertial;

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
        this.topoInertial = sensor.getTopoInertialFrame(epoch);

        // Discretise sensor's field of regard
        double height = sensor.getFov().getHeight();        // in [rad]
        double width = sensor.getFov().getWidth();          // in [rad]
        double cutOffEl = sensor.getElevCutOff();         // in [rad]

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
            finalState.getPVCoordinates();
        }
        return null;
    }
    
}
