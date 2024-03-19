package sensortasking.mcts;

import java.util.List;

import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;

public class SpatialDensityModel {

    /** Reference sensor. */
    Sensor sensor;

    /** Reference epoch. */
    AbsoluteDate epoch;

    /** Generated density model. */
    int[][] densityModel;

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
            SpacecraftState finalState = propagator.propagate(epoch);
        }
        return null;
    }
    
}
