package benchtest;

import sensortasking.mcts.DecisionNode;
import sensortasking.mcts.Sensor;

public class SatelliteRewardFunction extends TrackingRewardFunction<Satellite>{

    @Override
    public double computeRewardWrtSimEnd(DecisionNode last, DecisionNode initial, 
                                         double tCampaign) {
        return 0.;
    }

    @Override
    public double computeImmediateReward(DecisionNode current) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'computeImmediateReward'");
    }

    @Override
    public double computeRegretWrtFov(DecisionNode lastDecision, double tobs, Sensor sensor) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'computeRegretWrtFov'");
    }

    @Override
    public double computeRegretWrtSimEnd(DecisionNode lastDecision, double tobs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'computeRegretWrtSimEnd'");
    }
    
}
