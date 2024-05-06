package sensortasking.mcts;

import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;

import lombok.Getter;

@Getter
public class ObservedObject {

    /** Objects ID, can be Norad ID */
    private long id;

    /** Derived state vector from observation.*/
    private StateVector state;

    /** Derived covariance from observation. */
    private CartesianCovariance covariance;

    public ObservedObject(long id, StateVector state, CartesianCovariance covariance) {
        this.id = id;
        this.state = state;
        this.covariance = covariance;
    }
}
