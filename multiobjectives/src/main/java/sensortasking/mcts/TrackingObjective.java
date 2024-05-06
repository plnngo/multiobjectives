package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.util.FastMath;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;

public class TrackingObjective implements MacroAction{

    @Override
    public AngularDirection setMicroAction() {
        return new AngularDirection(null, 
                            new double[]{FastMath.toRadians(88.), FastMath.toRadians(30.)}, 
                            AngleType.AZEL);
    }

    @Override
    public double computeGain() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'computeGain'");
    }

    @Override
    public double getUtility() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getUtility'");
    }

    @Override
    public double getExecusionDuration() {
        // TODO Auto-generated method stub
        return 60.*5.;
    }

    @Override
    public List<ObservedObject> propagateOutcome() {
        // Fake data
        StateVector state = new StateVector();
        state.setX(150.);
        state.setY(250.);
        state.setZ(350.);

        CartesianCovariance cov = new CartesianCovariance(null);
        for (int pos=0; pos<3; pos++) {
            cov.setCovarianceMatrixEntry(pos, pos, 15.);
            cov.setCovarianceMatrixEntry(pos+3, pos+3, 150.);
        }
        
        ObservedObject obj = new ObservedObject(345, state, cov);
        List<ObservedObject> out = new ArrayList<ObservedObject>();
        out.add(obj);
        return out;
    }
    
}
