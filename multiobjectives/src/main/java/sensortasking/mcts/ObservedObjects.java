package sensortasking.mcts;

import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;

public class ObservedObjects {

    /** Objects ID, can be Norad ID */
    long id;

    /** Derived state vector from observation.*/
    StateVector state;

    /** Derived covariance from observation. */
    CartesianCovariance covariance;

    public ObservedObjects(long id, StateVector state, CartesianCovariance covariance) {
        this.id = id;
        this.state = state;
        this.covariance = covariance;
    }
}
