package benchtest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.orekit.frames.FramesFactory;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;
import sensortasking.mcts.AngleType;
import sensortasking.mcts.AngularDirection;
import sensortasking.mcts.App;
import sensortasking.mcts.ChanceNode;
import sensortasking.mcts.DecisionNode;
import sensortasking.mcts.Node;
import sensortasking.mcts.Objective;
import sensortasking.mcts.ObservedObject;

@SuppressWarnings("rawtypes")
@Getter
public class CarTrackingObjective implements Objective{

    List<Car> updatedTargets = new ArrayList<Car>();

    AbsoluteDate start;

    AbsoluteDate end;

    char lastUpdated = 'O';

    double lastUpdatedIG = 0.;

    double regret = 0.;

    final double tstep = 60.;         // originally 1.

    final List<Car> predictedTargets = new ArrayList<>();

    final double iLLimit = 1e-7;

    public CarTrackingObjective(List<ObservedObject> targets, AbsoluteDate startCampaign, 
                                AbsoluteDate endCampaign, List<ObservedObject> targetsPred) {

        // Initialise list of targets
        for (ObservedObject target : targets) {
            updatedTargets.add((Car)target);
        }
        this.start = startCampaign;
        this.end = endCampaign;

        // Set predicted targets propagated to end date without considering measurement updates
        for (ObservedObject target : targetsPred) {
            predictedTargets.add((Car) target);
        }
    }

    @Override
    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing) {
        
        double time = current.durationFrom(this.start) + tstep;

        // No cars to track
        if (this.updatedTargets.isEmpty()) {
            return null;
        }

        // List of candidates that might be trackable
        Map<Car, Double[]> checkTrackable = new HashMap<Car, Double[]>();        
        for (Car obj : updatedTargets) {
            double[] state = new double[]{obj.getPosX(), obj.getPosY(), obj.getVelX(), obj.getVelY()};
            Car copy = new Car(obj.getIdentifier(), state, obj.getCov(), obj.getTime());

            // Simulate measuremement
            double simMeas = generateMeasurement(time, state);
            Filter est = new Filter();
            est.run_ckf(state, copy.getCov(), copy.getTime(), time, simMeas);
            if (FastMath.abs(copy.getVelY())>0.00001) {
                throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                        + "Y axis");
            }

            // Compute information gain
/*             System.out.println("predicted:");
            App.printCovariance(new Array2DRowRealMatrix(est.covPred));
            System.out.println("corrected:");
            App.printCovariance(new Array2DRowRealMatrix(est.covCorr)); */
            /* double iG = 
                computeKLDivergence(est.statePred, est.stateCorr, 
                                                 est.covPred, est.covCorr); */
            double iG = computeTraceChange(est.covPred, est.covCorr);

            // propagate predicted state to end date and compute loss of information update
            Filter estLoss = new Filter();
            //double timeUntilEnd = this.end.durationFrom(current) - tstep;
            double timeUntilEnd = this.end.durationFrom(this.start);

            double simMeasPred = generateMeasurement(timeUntilEnd, est.statePred);
            estLoss.run_ckf(est.statePred, est.covPred, time, timeUntilEnd, simMeasPred);

            // search for the corresponding target in predicted targets
            double[][] predCovNoMeas = new double[estLoss.covCorr.length][estLoss.covCorr.length];
            for (Car objPred : predictedTargets) {
                if (objPred.getIdentifier() == obj.getIdentifier()) {
                    predCovNoMeas = objPred.getCov();
                }
            }

            double iL = computeTraceChange(predCovNoMeas, estLoss.covPred );
            Car copyUpdated = new Car(copy.getIdentifier(), est.stateCorr, est.covCorr, time);
            if (FastMath.abs(copyUpdated.getVelY())>0.00001) {
                throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                        + "Y axis");
            }
            checkTrackable.put(copyUpdated, new Double[]{iG, iL});
        }

        // Step 0: extract reward (iG-iL)
        Map<Car, Double> checkTrackableReward = new HashMap<Car, Double>();        

        for (Entry<Car, Double[]> entry : checkTrackable.entrySet()) {
            double noRegret = 0;
            for (Entry<Car, Double[]> other : checkTrackable.entrySet()) {
                if (other.getKey().getIdentifier() != entry.getKey().getIdentifier()) {
                    // define lost
                    noRegret += other.getValue()[1];
                }
            }
            //double reward = noRegret;     //entry.getValue()[0];
            double reward = entry.getValue()[0];
            /* if (checkTrackable.entrySet().size() != 1) {
                // No alternative candidate
                //reward = reward - 1./lost;
                if (noRegret < iLLimit) {
                    noRegret = iLLimit;
                }
                reward = FastMath.abs(1./noRegret);
                System.out.println(1./noRegret);
            } */
            checkTrackableReward.put(entry.getKey(), reward);
        }


        // Step 1: Find max IG
        Random rand = new Random();
        double iGmax = -Double.MAX_VALUE;
        for (Entry<Car, Double> entry : checkTrackableReward.entrySet()) {
            if (entry.getValue() > iGmax) {
                iGmax = entry.getValue();
            }
        }

        // Step 2: Collect all cars with max IG
        List<Car> bestCandidates = new ArrayList<>();
        for (Entry<Car, Double> entry : checkTrackableReward.entrySet()) {
            if (entry.getValue() == iGmax) {
                bestCandidates.add(entry.getKey());
            }
        }

        // Step 3: Pick one randomly
        Car selectedRaw = bestCandidates.get(rand.nextInt(bestCandidates.size()));
        //System.out.println("selected car: " + selectedRaw.getIdentifier());

        // Step 4: Construct the selected Car object
        double[] stateUpdated = new double[]{
            selectedRaw.getPosX(), selectedRaw.getPosY(), 
            selectedRaw.getVelX(), selectedRaw.getVelY()
        };
        Car selected = new Car(
            selectedRaw.getIdentifier(),
            stateUpdated,
            selectedRaw.getCov(),
            time
        );

        // Step 5: Update targets
        for (Car candidate : updatedTargets) {
            if (candidate.getIdentifier() == selected.getIdentifier()) {
                candidate.setState(selected.getPosX(), selected.getPosY(),
                                selected.getVelX(), selected.getVelY());
                candidate.setCov(selected.getCov());
                candidate.setTime(selected.getTime());
                candidate.setEpoch(selected.getEpoch());
                if (FastMath.abs(selected.getVelY())>0.00001) {
                    throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                            + "Y axis");
                }

                this.lastUpdated = selected.getIdentifier();
                this.lastUpdatedIG = iGmax;
                for (Entry<Car, Double[]> entry : checkTrackable.entrySet()) {
                    if (entry.getKey().getIdentifier() == candidate.getIdentifier()) {
                        this.regret = entry.getValue()[1];
                        break;
                    }
                }
                break;
            }
        }  

        // Compute pointing angle
        double alpha = FastMath.atan2(selected.getPosX(), selected.getPosY());
        double range = FastMath.sqrt(selected.getPosX() * selected.getPosX() 
                                        + selected.getPosY() * selected.getPosY());
        AngularDirection angle = new AngularDirection(FramesFactory.getEME2000(), 
                                                      new double[]{alpha, 0.}, 
                                                      AngleType.RADEC, range);
        angle.setDate(this.start.shiftedBy(time));
        return angle;
    }

    public static double computeTraceChange(double[][] covPrior, double[][] covPost) {

        // Retrieve covariances
        RealMatrix covP = new Array2DRowRealMatrix(covPrior);
        RealMatrix covQ = new Array2DRowRealMatrix(covPost);

        //App.printCovariance(covP);
        //App.printCovariance(covQ);
        double change = covP.getTrace() - covQ.getTrace();
        //double changeNorm = change/covP.getTrace();
        return change;
    }

    protected static double computeKLDivergence(double[] statePrior, 
                                                             double[] statePost, 
                                                             double[][] covPrior, 
                                                             double[][] covPost) {
        
        // Retrieve covariances
        RealMatrix covP = new Array2DRowRealMatrix(covPrior);
        RealMatrix covQ = new Array2DRowRealMatrix(covPost);
/*         System.out.println("Predicted:");
        App.printCovariance(covP);
        System.out.println("Corrected:");
        App.printCovariance(covQ); */

        // Compute determinant
        LUDecomposition decomP = new LUDecomposition(covP);
        LUDecomposition decomQ = new LUDecomposition(covQ);
        double detP = decomP.getDeterminant();
        double detQ = decomQ.getDeterminant();
        //detQ = 3.8462e-18;

        double logDetCovQByDetCovP = FastMath.log(detQ/detP);
        double traceCovQ = covQ.getTrace();

        // Compute inverse of covQ
        RealMatrix invCovQ = MatrixUtils.inverse(covQ);

        double traceInvCovQCovP = invCovQ.multiply(covP).getTrace();

        // Substract means of probability distributions
        double[] meanQMinusMeanP = new double[statePrior.length];
        
        for (int i=0; i<meanQMinusMeanP.length; i++) {
            meanQMinusMeanP[i] = statePost[i] - statePrior[i];
        }

        // Means transposed multiplied by inverse covariance of Q
        double[] meanTMultiplyInvCovQ = new double[]{0.,0.,0.,0.,0.,0.};
        for (int numCol=0; numCol<invCovQ.getColumnDimension(); numCol++) {
            double[] colInvCovQ = invCovQ.getColumn(numCol);
            for (int i=0; i<meanQMinusMeanP.length; i++) {
                meanTMultiplyInvCovQ[numCol] += meanQMinusMeanP[i] * colInvCovQ[i];
            }
        }
        
        // Multiply with mean again
        double meanTMultiplyInvCovQMultiplyMean = 0.;
        for (int i=0; i<meanQMinusMeanP.length; i++) {
            meanTMultiplyInvCovQMultiplyMean += meanTMultiplyInvCovQ[i] * meanQMinusMeanP[i];
        }
        
        double dKL = 0.5 * (logDetCovQByDetCovP + traceInvCovQCovP 
                                + meanTMultiplyInvCovQMultiplyMean - meanQMinusMeanP.length);
                    
        return dKL;
    }

    public static double generateMeasurement(double tobs, double[] initialState) {
        // Ensure that object only moves with constant velocity along X axis
        if (FastMath.abs(initialState[3])>0.00001) {
            throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                    + "Y axis");
        } else {
            double velX = initialState[2];
            double dist = tobs * velX;
            double posXNew = dist;
            double[] stateNew = new double[]{posXNew, initialState[1], velX, initialState[3]}; 
            double simMeas = LinearRangeMeasurementModel.generateHk(stateNew).Gk;
            return simMeas;
        }
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {

        double exeTime = tstep + 1.;
/*         if(current.durationFrom(this.start) < 0.9) {
            exeTime = 2.;       // Integration in setMicroAction fails for delta t = 0;
        }  */
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current.shiftedBy(exeTime)};
        return interval;
    }

    @Override
    public List propagateOutcome() {
        // return copy of updated targets
        List<Car> out = new ArrayList<Car>();
        for (Car obj : this.updatedTargets) {
            Car copy = new Car(obj.getIdentifier(), obj.getPosX(), obj.getPosY(), 
                               obj.getVelX(), obj.getVelY(), obj.getCov(), obj.getTime());
            out.add(copy);
        }
        
        return out;
    }

    /**
     * 
     * @param last          last simulated node.
     * @param leaf          last extisting node (without simulated nodes).
     * @return
     */
    public static double[] computeTrackReward(DecisionNode last, DecisionNode leaf, double tCampaign) {

        // Convert observedObject to car
        //List<ObservedObject> trackedObjs = last.getEnvironment().getStateTracking();
        //List<Car> trackedCars = transformObservedObjectsToCars(trackedObjs);

        // Initialise output
        double[] out = new double[1];

        // Propagate all targets from their intial state towards common epoch with Kepler dynamics
        Node root = last;
        AbsoluteDate preLeaf = leaf.getParent().getParent().getEpoch();

        // Check if simulation phase was entered  
        while (root.getEpoch().compareTo(preLeaf)!=0) {
            if (root.getClass().getSimpleName().equals("DecisionNode")) {
                DecisionNode current = (DecisionNode)root;
                //out[0] += ((CarTrackingObjective)current.getMacro()).getLastUpdatedIG();
                out[0] += CarTrackingObjective.computeRegret(current, tCampaign);
            }
            root = root.getParent();
        }

        // Add leaf reward too
        //ChanceNode parentLeaf = (ChanceNode)leaf.getParent();
        //out[0] += ((CarTrackingObjective)parentLeaf.getMacro()).getLastUpdatedIG();
        //out[0] += CarTrackingObjective.computeRegret(leaf, tCampaign);

        return out;
    }

    private static double computeRegret(DecisionNode lastDecision, double timeUntilEnd) {
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        char lastUpdated = ((CarTrackingObjective)parent.getMacro()).getLastUpdated();
        //List<Node> siblings = parent.getChildren();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Car sibling = (Car)objEnv;
            if (sibling.getIdentifier() != lastUpdated) {

                double simMeasPred = 
                    CarTrackingObjective.generateMeasurement(timeUntilEnd, 
                                                             sibling.getStateArray());
                Filter estLoss = new Filter();

                // Extract predicted covariance of sibling propagated to endCampaign 
                estLoss.run_ckf(sibling.getStateArray(), sibling.getCov(), sibling.getTime(), 
                                timeUntilEnd, simMeasPred);
                double[][] predCovSibling = estLoss.getCovPred();
                List<Car> predNoMeasSiblings = 
                    ((CarTrackingObjective)parent.getMacro()).getPredictedTargets();
                
                // Extract predicted covariance of sibling without measurement updates propagated 
                // to endCampaign 
                double[][] predCovNoMeasSibling = 
                    new double[predCovSibling.length][predCovSibling.length];
                for (Car same : predNoMeasSiblings) {
                    if (same.getIdentifier() == sibling.getIdentifier()) {
                        predCovNoMeasSibling = same.getCov();
                        break;
                    }
                }
                regret += CarTrackingObjective.computeTraceChange(predCovNoMeasSibling, 
                                                                  predCovSibling);
            }
        }
        return regret;
    }

    private static List<Car> transformObservedObjectsToCars(List<ObservedObject> trackedObjs) {
        List<Car> out = new ArrayList<Car>();
        for(ObservedObject obj : trackedObjs) {
            Car car = new Car(((Car)obj).getIdentifier(), ((Car)obj).getStateArray(), 
                              ((Car)obj).getCov(), ((Car)obj).getTime());
            out.add(car);
        }
        return out;
    }
    
}
