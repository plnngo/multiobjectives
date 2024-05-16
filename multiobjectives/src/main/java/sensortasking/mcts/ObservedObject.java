package sensortasking.mcts;

import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.time.AbsoluteDate;

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

    public ObservedObject(long id, StateVector state, CartesianCovariance covariance, 
                          AbsoluteDate epoch) {
        this.id = id;
        this.state = state;
        this.covariance = covariance;
        this.epoch = epoch;
    }
}
