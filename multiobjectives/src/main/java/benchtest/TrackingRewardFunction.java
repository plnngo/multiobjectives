package benchtest;

import sensortasking.mcts.DecisionNode;
import sensortasking.mcts.Sensor;

public abstract class TrackingRewardFunction<T> {

    /**
     * 
     * @param last          Last (simulated) node.
     * @param initial       Root node.
     * @param tCampaign     Duration of observation campaign
     * @return              Reward with respect to end of time horizon.
     */
    public abstract double computeRewardWrtSimEnd(DecisionNode last, 
                                                  DecisionNode initial, double tCampaign);
    
    /**
     * 
     * @param current
     * @return
     */
    public abstract double computeImmediateReward(DecisionNode current);    
    public abstract double computeRegretWrtFov(DecisionNode lastDecision, double tobs, 
                                               Sensor sensor);
    public abstract double computeRegretWrtSimEnd(DecisionNode lastDecision, double tobs);
}
