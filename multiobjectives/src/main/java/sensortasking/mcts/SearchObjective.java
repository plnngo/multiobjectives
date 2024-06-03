package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.util.FastMath;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.FramesFactory;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;

@Getter
public class SearchObjective implements Objective{


    @Override
    public AngularDirection setMicroAction(AbsoluteDate current) {
        return new AngularDirection(null, new double[]{FastMath.toRadians(30.),  
                                    FastMath.toRadians(50.)}, AngleType.AZEL);
    }

/*     @Override
    public double computeGain() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'computeGain'");
    } */

    @Override
    public double getUtility() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getUtility'");
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        // TODO Auto-generated method stub
        //return 5.*60.;
        return null;
    }

    @Override
    public List<ObservedObject> propagateOutcome() {

        // Fake data
        StateVector state = new StateVector();
        state.setX(100.);
        state.setY(200.);
        state.setZ(300.);

        CartesianCovariance cov = new CartesianCovariance(null);
        for (int pos=0; pos<3; pos++) {
            cov.setCovarianceMatrixEntry(pos, pos, 10.);
            cov.setCovarianceMatrixEntry(pos+3, pos+3, 100.);
        }
        
        ObservedObject obj = new ObservedObject(0123, state, cov, new AbsoluteDate(), FramesFactory.getTEME());
        List<ObservedObject> out = new ArrayList<ObservedObject>();
        out.add(obj);
        return out;
    }
}
