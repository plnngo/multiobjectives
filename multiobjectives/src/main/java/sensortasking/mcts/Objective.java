package sensortasking.mcts;

import java.util.List;

import org.orekit.time.AbsoluteDate;

import benchtest.Car;
import benchtest.CarRewardFunction;
import benchtest.RewardFunction;
import benchtest.Satellite;
import benchtest.SatelliteRewardFunction;
import benchtest.TrackingRewardFunction;

public abstract class Objective<T> {

    // Abstract method: must be implemented by subclasses
    public abstract AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing);

    // Abstract method: must be implemented by subclasses
    public abstract AbsoluteDate[] getExecusionDuration(AbsoluteDate current);

    // Abstract method: must be implemented by subclasses
    public abstract List<T> propagateOutcome();

    public static void computeTrackReward(boolean orbitMode, DecisionNode last, DecisionNode leaf, 
                                     DecisionNode initial, double tCampaign, 
                                     double discount, Sensor sensor, 
                                     RewardFunction selectedReward) {
        double accDiscountedR = 0.;
        TrackingRewardFunction rewardFunc;
        if (!orbitMode) {
            rewardFunc = new CarRewardFunction();
        } else {
            rewardFunc = new SatelliteRewardFunction();
        } 

        if (selectedReward.equals(RewardFunction.REWARD_WRT_SIMULATED_END)) {
            accDiscountedR = rewardFunc.computeRewardWrtSimEnd(last, initial, tCampaign);
        }

        // Propagate all targets from their intial state towards common epoch with Kepler dynamics
        Node futureBranch = last;
        AbsoluteDate leafEpoch = leaf.getEpoch();
        if(futureBranch.getEpoch().compareTo(leafEpoch)==0) {
            // no simulation phase took place
            futureBranch = leaf;
        }

        while (!futureBranch.equals(initial)) {
            if (futureBranch.getClass().getSimpleName().equals("DecisionNode")) {
                DecisionNode current = (DecisionNode)futureBranch;
                current.incrementNumVisits();
                current.getParent().incrementNumVisits();
                double tobs = current.getEpoch().durationFrom(initial.getEpoch());
                if (selectedReward.equals(RewardFunction.REGRET_WRT_SIMULATED_END)) {
                    accDiscountedR = rewardFunc.computeRegretWrtSimEnd(current, tCampaign)
                                        + discount * accDiscountedR;
                } else if (selectedReward.equals(RewardFunction.IMMEDIATE_REWARD)) {
                    accDiscountedR = rewardFunc.computeImmediateReward(current) 
                                        + discount * accDiscountedR;
                } else if (selectedReward.equals(RewardFunction.REGRET_WRT_FOV)) {
                    accDiscountedR = rewardFunc.computeRegretWrtFov(current, tobs, sensor) 
                                        + discount * accDiscountedR;
                }

                double utilityTrack = current.getUtilityVec()[1];       //0=search; 1=track
                utilityTrack = utilityTrack + (accDiscountedR - utilityTrack)
                                                        /current.getNumVisits();
                double[] utility = new double[]{current.getUtilityVec()[0], utilityTrack};
                
                current.setUtilityVec(utility);
                current.getParent().setUtilityVec(utility);
            }
            futureBranch = futureBranch.getParent();
        }
        initial.incrementNumVisits();
    }

}
