package benchtest;

import sensortasking.mcts.DecisionNode;
import sensortasking.mcts.Sensor;

public abstract class TrackingRewardFunction<T> {

    public abstract double computeRewardWrtSimEnd(DecisionNode last, 
                                                  DecisionNode initial, double tCampaign);
    public abstract double computeImmediateReward(DecisionNode current);    
    public abstract double computeRegretWrtFov(DecisionNode lastDecision, double tobs, 
                                               Sensor sensor);
    public abstract double computeRegretWrtSimEnd(DecisionNode lastDecision, double tobs);
}
