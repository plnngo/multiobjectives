package sensortasking.mcts;

import java.util.List;

import org.hipparchus.util.FastMath;

public class SearchObjective implements MacroAction{

    @Override
    public AngularDirection setMicroAction() {
        return new AngularDirection(null, 
                                    new double[]{FastMath.toRadians(30.), FastMath.toRadians(50.)}, 
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
        throw new UnsupportedOperationException("Unimplemented method 'getExecusionDuration'");
    }

    @Override
    public List<ObservedObject> propagateOutcome() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'propagateOutcome'");
    }
    
}
