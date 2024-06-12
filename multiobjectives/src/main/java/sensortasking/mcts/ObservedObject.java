package sensortasking.mcts;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.orbits.Orbit;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.TimeStampedPVCoordinates;

import lombok.Getter;

@Getter
public class ObservedObject {

    /** Objects ID, can be Norad ID. */
    private long id;

    /** Derived state vector from observation.*/
    private StateVector state;

    /** Derived covariance from observation. */
    private CartesianCovariance covariance;

    /** Reference epoch. */
    private AbsoluteDate epoch;

    /** Reference frame. */
    private Frame frame;

    /** Reference TLE */
    private TLE pseudoTle;

    public ObservedObject(long id, StateVector state, CartesianCovariance covariance, 
                          AbsoluteDate epoch, Frame frame) {
        this.id = id;
        this.state = state;
        this.covariance = covariance;
        this.epoch = epoch;
        this.frame = frame;
    }

    public ObservedObject(long id, StateVector state, CartesianCovariance covariance, 
                          Frame frame, TLE tle) {
        this.id = id;
        this.state = state;
        this.covariance = covariance;
        this.epoch = tle.getDate();
        this.frame = frame;
        this.pseudoTle = tle;
    }

    public void setTle(TLE pseudoTle) {
        this.pseudoTle = pseudoTle;
    }

    public static StateVector spacecraftStateToStateVector(SpacecraftState spacecraftState, 
                                                           Frame stationFrame){

        TimeStampedPVCoordinates pv = spacecraftState.getPVCoordinates(stationFrame);
        StateVector state = new StateVector();

        // Set position
        Vector3D pos = pv.getPosition();
        state.setX(pos.getX());
        state.setY(pos.getY());
        state.setZ(pos.getZ());

        // Set velocity
        Vector3D vel = pv.getVelocity();
        state.setXdot(vel.getX());
        state.setYdot(vel.getY());
        state.setZdot(vel.getZ());
        
        return state;
    }

    public static CartesianCovariance stateCovToCartesianCov(Orbit orbit, StateCovariance stateCov, 
                                                      Frame stationFrame) {
        stateCov = stateCov.changeCovarianceFrame(orbit, stationFrame);

        CartesianCovariance output = new CartesianCovariance(null);
        
        int colDim = stateCov.getMatrix().getColumnDimension();
        int rowDim = stateCov.getMatrix().getRowDimension();
        for (int row=0; row<rowDim; row++) {
            for (int col=0; col<colDim; col++) {
                double entry = stateCov.getMatrix().getEntry(row, col);
                output.setCovarianceMatrixEntry(row, col, entry);
            }
        }
       
        return output;
    }
}
