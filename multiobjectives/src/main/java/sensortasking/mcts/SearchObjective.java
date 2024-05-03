package sensortasking.mcts;

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
    
}
