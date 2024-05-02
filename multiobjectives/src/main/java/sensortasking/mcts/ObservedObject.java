package sensortasking.mcts;

import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;

public class ObservedObject {

    /** Objects ID, can be Norad ID */
    long id;

    /** Derived state vector from observation.*/
    StateVector state;

    /** Derived covariance from observation. */
    CartesianCovariance covariance;

    public ObservedObject(long id, StateVector state, CartesianCovariance covariance) {
        this.id = id;
        this.state = state;
        this.covariance = covariance;
    }
}
