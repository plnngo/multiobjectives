package sensortasking.mcts;

import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.propagation.analytical.tle.TLE;
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
}
