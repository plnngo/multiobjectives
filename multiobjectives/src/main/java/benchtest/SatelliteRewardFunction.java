package benchtest;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.distribution.continuous.NormalDistribution;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.orekit.time.AbsoluteDate;

import sensortasking.mcts.ChanceNode;
import sensortasking.mcts.DecisionNode;
import sensortasking.mcts.ObservedObject;
import sensortasking.mcts.Sensor;
import sensortasking.mcts.TrackingObjective;

public class SatelliteRewardFunction extends TrackingRewardFunction<Satellite>{

    @Override
    public double computeRewardWrtSimEnd(DecisionNode last, DecisionNode initial, 
                                         double tCampaign) {
        // Initialise output
        double reward = 0.;

        // Propagate all targets from their intial state towards common epoch with circular dynamics
        List<ObservedObject> targetsInitial = (initial).getEnvironment().getStateTracking();
        List<Satellite> targetsPredicted = new ArrayList<Satellite>();
        for(ObservedObject init : targetsInitial) {
            Satellite initialSat = (Satellite)init;
            Satellite propInit = 
                Satellite.propagateSatellite(initialSat, initial.getEpoch(), 
                                             initial.getEpoch().shiftedBy(tCampaign));
            targetsPredicted.add(propInit);
        }

        // Propagate all targets from their updated final state towards common epoch
        List<ObservedObject> trackedObjs = last.getEnvironment().getStateTracking();
        List<Satellite> targetsFinal = new ArrayList<Satellite>();
        for(ObservedObject finalTarget : trackedObjs) {
            Satellite finalSat = (Satellite)finalTarget;
            Satellite propFinal = 
                Satellite.propagateSatellite(finalSat, finalSat.getEpoch(), 
                                             initial.getEpoch().shiftedBy(tCampaign));
            targetsFinal.add(propFinal);
        }

        // Calculate information gain
        if(targetsPredicted.size() != targetsFinal.size()) {
            throw new IllegalArgumentException("Information gain cannot be computed due to " 
                                                + "dimension error in targets.");
        }

        for(int i=0; i<targetsPredicted.size(); i++) {
            int j=0;
            while(j<targetsFinal.size()) {

                // Make sure that ID of objects are the same when computing information gain
                if(targetsPredicted.get(i).getId() != targetsFinal.get(j).getId()){
                    // Move to next object in targetFinals                                                                          
                    j++;
                } else {
                    // Same ID found
                    double[][] targetPredCov = targetsPredicted.get(i)
                                                               .getCovariance()
                                                               .getCovarianceMatrix()
                                                               .getData();
                    double[][] targetFinalCov = targetsFinal.get(j)
                                                            .getCovariance()
                                                            .getCovarianceMatrix()
                                                            .getData();
                    reward += TrackingObjective.computeTraceChange(targetPredCov, targetFinalCov);
                    // No need to continue searching in targetFinals
                    targetsFinal.remove(j);
                    j=0;
                    break;
                }
            }
        }

        return reward;
    }

    @Override
    public double computeImmediateReward(DecisionNode current) {
        
        ChanceNode parent = (ChanceNode) current.getParent();
        double lastUpdatedIG = ((TrackingObjective)parent.getMacro()).getLastUpdatedIG();
        return lastUpdatedIG;
    }

    @Override
    public double computeRegretWrtFov(DecisionNode lastDecision, double tobs, Sensor sensor) {
        double halfFov = sensor.getFov().getHeight()/2.;
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        AbsoluteDate tObs = parent.getMicro().getDate();
        long lastUpdated = ((TrackingObjective)parent.getMacro()).getLastUpdated();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Satellite sibling = (Satellite)objEnv;
            if (sibling.getId() != lastUpdated) {
                Satellite propSibling = 
                    Satellite.propagateSatellite(sibling, sibling.getEpoch(), tObs); //TODO: check that covariance is same as Pk_bar

                RealMatrix Pk_bar = new Array2DRowRealMatrix(propSibling.getCovariance()
                                                                        .getCovarianceMatrix()
                                                                        .getData());
                double[] Xref = new double[]{propSibling.getState().getPositionVector().getX(),
                                             propSibling.getState().getPositionVector().getY(),
                                             propSibling.getState().getPositionVector().getZ(),
                                             propSibling.getState().getVelocityVector().getX(),
                                             propSibling.getState().getVelocityVector().getY(),
                                             propSibling.getState().getVelocityVector().getZ()};

                // Transform uncertainty from state space into measurement space
                double[][] H = OrbitRangeAngularMeasurementModel.generateHk(Xref).Hk_til;
                RealMatrix obsMatrix = new Array2DRowRealMatrix(H);
                RealMatrix Pk_bar_meas = obsMatrix.multiply(Pk_bar).multiplyTransposed(obsMatrix);

                // Extract standard deviation
                int dim = Pk_bar_meas.getColumnDimension();
                double[] std = new double[dim];
                for(int i=0; i<dim; i++) {
                    std[i] = FastMath.sqrt(Pk_bar_meas.getEntry(i, i));
                }

                // TODO: code assumes one dimensional measurement vector
                // Assume perfect pointing --> mean = 0
                NormalDistribution gaussian = new NormalDistribution(0., std[0]);

                // Compute probability of the sibling to be in the FOV
                double prob = gaussian.probability(-halfFov, halfFov);
                regret += prob;
            }
        }
        return regret;
    }

    @Override
    public double computeRegretWrtSimEnd(DecisionNode lastDecision, double tobs) {
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        AbsoluteDate tObs = parent.getMicro().getDate();
        long lastUpdated = ((TrackingObjective)parent.getMacro()).getLastUpdated();
        //List<Node> siblings = parent.getChildren();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Satellite sibling = (Satellite)objEnv;
            if (sibling.getId() != lastUpdated) {

                Satellite propSibling = Satellite.propagateSatellite(sibling, sibling.getEpoch(), tObs);
                double[][] predCovSibling = 
                    propSibling.getCovariance().getCovarianceMatrix().getData();
                List<Car> predNoMeasSiblings = 
                    ((CarTrackingObjective)parent.getMacro()).getPredictedTargets();
                
                // Extract predicted covariance of sibling without measurement updates propagated 
                // to endCampaign 
                double[][] predCovNoMeasSibling = 
                    new double[predCovSibling.length][predCovSibling.length];
                for (Car same : predNoMeasSiblings) {
                    if (same.getIdentifier() == sibling.getId()) {
                        predCovNoMeasSibling = same.getCov();
                        break;
                    }
                }
                regret += TrackingObjective.computeTraceChange(predCovNoMeasSibling, 
                                                                  predCovSibling);
            }
        }
        return regret;
    }
    
}
