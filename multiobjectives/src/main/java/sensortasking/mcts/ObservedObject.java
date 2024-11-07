package sensortasking.mcts;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.RealMatrix;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

import java.util.List;
import java.util.ArrayList;

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

    public void setState(StateVector newState) {
        this.state = newState;
    }

    public void setCovariance(CartesianCovariance newCovariance) {
        this.covariance = newCovariance;
    }

    public void setEpoch(AbsoluteDate newEpoch) {
        this.epoch = newEpoch;
    }

    public void setFrame(Frame newFrame) {
        this.frame = newFrame;
    }

    public void setTle(TLE pseudoTle) {
        this.pseudoTle = pseudoTle;
    }

    public static StateVector spacecraftStateToStateVector(SpacecraftState spacecraftState, 
                                                           Frame outputFrame){

        TimeStampedPVCoordinates pv = spacecraftState.getPVCoordinates(outputFrame);
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
                                                      Frame reference) {
        stateCov = stateCov.changeCovarianceFrame(orbit, reference);

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

    public static List<ObservedObject> propagateTargets(List<ObservedObject> objs, 
                                                        AbsoluteDate epoch) {

        // Initialise output                                                    
        List<ObservedObject> out = new ArrayList<ObservedObject>();

        for(ObservedObject obj : objs) {
            Vector3D pos = obj.getState().getPositionVector();
            Vector3D vel = obj.getState().getVelocityVector();
            PVCoordinates pv = new PVCoordinates(pos, vel);
            Orbit initialOrbit = 
                new CartesianOrbit(pv, obj.getFrame(), 
                                   obj.getEpoch(), Constants.WGS84_EARTH_MU);
            KeplerianPropagator kepPropo = new KeplerianPropagator(initialOrbit);

             // Set up covariance matrix provider and add it to the propagator
            final String stmName = "stm";
            final MatricesHarvester harvester = 
                kepPropo.setupMatricesComputation(stmName, null, null);

            // Propagate
            SpacecraftState predState = kepPropo.propagate(epoch);
            RealMatrix dYdY0 = harvester.getStateTransitionMatrix(predState);
            RealMatrix covInit = obj.getCovariance().getCovarianceMatrix();
            RealMatrix predictedCov = dYdY0.multiply(covInit).multiplyTransposed(dYdY0);
            StateCovariance stateCov = 
                new StateCovariance(predictedCov, predState.getDate(), predState.getFrame(), 
                                    OrbitType.CARTESIAN, PositionAngleType.MEAN);
            ObservedObject targetPred = 
                new ObservedObject(obj.getId(), 
                                   ObservedObject
                                    .spacecraftStateToStateVector(predState, predState.getFrame()),
                                   ObservedObject
                                    .stateCovToCartesianCov(predState.getOrbit(), stateCov, 
                                                            predState.getFrame()), 
                                   epoch, predState.getFrame());
            out.add(targetPred);
        }
        return out;

    }
}
