package benchtest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import org.apache.commons.lang3.ArrayUtils;
import org.hipparchus.distribution.continuous.NormalDistribution;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.ExpandableODE;
import org.hipparchus.ode.ODEIntegrator;
import org.hipparchus.ode.ODEState;
import org.hipparchus.ode.ODEStateAndDerivative;
import org.hipparchus.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.hipparchus.util.FastMath;
import org.orekit.frames.FramesFactory;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;
import sensortasking.mcts.AngleType;
import sensortasking.mcts.AngularDirection;
import sensortasking.mcts.ChanceNode;
import sensortasking.mcts.DecisionNode;
import sensortasking.mcts.Node;
import sensortasking.mcts.Objective;
import sensortasking.mcts.ObservedObject;
import sensortasking.mcts.Sensor;

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

    final static double epsilon = 1e-7;

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
        
        double tobs = current.durationFrom(this.start) + tstep;

        // No cars to track
        if (this.updatedTargets.isEmpty()) {
            return null;
        }

        // List of candidates that might be trackable
        Map<Car, Double> checkTrackable = new HashMap<Car, Double>();        
        for (Car obj : updatedTargets) {
            double[] state = new double[]{obj.getPosX(), obj.getPosY(), obj.getVelX(), obj.getVelY()};
            Car copy = new Car(obj.getIdentifier(), state, obj.getCov(), obj.getTime());

            // Simulate measuremement
            Filter est = new Filter();
            est.run_ckf(state, copy.getCov(), obj.getEpoch(), current.shiftedBy(tstep));

            // Compute information gain
            double iG = computeTraceChange(est.covPred, est.covCorr);
            Car copyUpdated = new Car(copy.getIdentifier(), est.stateCorr, est.covCorr, tobs); 

            checkTrackable.put(copyUpdated, iG);
        }

        // Step 1: Find max IG
        Random rand = new Random();
        double iGmax = -Double.MAX_VALUE;
        for (Entry<Car, Double> entry : checkTrackable.entrySet()) {
            if (entry.getValue() > iGmax) {
                iGmax = entry.getValue();
            }
        }

        // Step 2: Collect all cars with max IG
        List<Car> bestCandidates = new ArrayList<>();
        for (Entry<Car, Double> entry : checkTrackable.entrySet()) {
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
            tobs
        );

        // Step 5: Update targets
        for (Car candidate : updatedTargets) {
            if (candidate.getIdentifier() == selected.getIdentifier()) {
                candidate.setState(selected.getPosX(), selected.getPosY(),
                                selected.getVelX(), selected.getVelY());
                candidate.setCov(selected.getCov());
                candidate.setTime(selected.getTime());
                candidate.setEpoch(selected.getEpoch());
                
                this.lastUpdated = selected.getIdentifier();
                this.lastUpdatedIG = iGmax;
                break;
            }
        }  

        // Compute pointing angle
        double alpha = FastMath.atan2(selected.getPosY(), selected.getPosX());
        double range = FastMath.sqrt(selected.getPosX() * selected.getPosX() 
                                        + selected.getPosY() * selected.getPosY());
        AngularDirection angle = new AngularDirection(FramesFactory.getEME2000(), 
                                                      new double[]{alpha, 0.}, 
                                                      AngleType.RADEC, range);
        angle.setDate(this.start.shiftedBy(tobs));
        return angle;
    }

    public static double computeTraceChange(double[][] covPrior, double[][] covPost) {

        // Retrieve covariances
        RealMatrix covP = new Array2DRowRealMatrix(covPrior);
        RealMatrix covQ = new Array2DRowRealMatrix(covPost);

/*         App.printCovariance(covP);
        System.out.println("Prior: " + covP.getTrace());
        App.printCovariance(covQ);
        System.out.println("Post:" + covQ.getTrace()); */
        double change = covP.getTrace() - covQ.getTrace();
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

        double logDetCovQByDetCovP = FastMath.log(detQ/detP);

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

    public static double generateRangeMeasurement(double tobs, double[] initialState) {
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

    public static double generateBearingMeasurement(double tobs, double[] initialState, Car car) {
        int n = initialState.length;
        double[] stateNew = new double[n];
        double[] y = propagateStateAndSTM(tobs, initialState, car);

        for (int i=0; i<n; i++) {

            // Extract state vector
            double rounded = FastMath.rint(y[i] / epsilon) * epsilon;
            stateNew[i] = rounded;
        }

        double simMeas = LinearBearingMeasurementModel.generateHk(stateNew).Gk;
        return simMeas;
        
    }

    private static double[] propagateStateAndSTM(double tobs, double[] initialState, Car car) {
        int n = initialState.length;

        // Combine initial state and STM (identity matrix)
        double[][] identity = new double[n][n];
        for (int col=0; col<n; col++) {
            for (int row=0; row<n; row++) {
                if(row==col) {
                    identity[row][col] = 1.;
                } else {
                    identity[row][col] = 0;
                }
            }
        }
        RealMatrix ones = new Array2DRowRealMatrix(identity);
        double[] ones_arr = Filter.flattenRowMajor(ones.getData());
        double[] Xref_Stm0 = ArrayUtils.addAll(initialState, ones_arr);
        double[] y = new double[Xref_Stm0.length];

        ExpandableODE expandable = new ExpandableODE(car);
        ODEIntegrator integrator = new ClassicalRungeKuttaIntegrator(0.01);
        ODEState initial = new ODEState(car.getTime(), Xref_Stm0);

        if(car.getTime() == tobs) {
            y = Xref_Stm0;
        } else {
            ODEStateAndDerivative finalState = integrator.integrate(expandable, initial, tobs);
            y = finalState.getPrimaryState();
        }
        return y;
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {

        double exeTime = tstep;     // + 1.;
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

    protected static double computeRewardWrtSimEnd(DecisionNode last, DecisionNode initial, double tCampaign) {

        // Initialise output
        double reward = 0.;

        // Propagate all targets from their intial state towards common epoch with circular dynamics
        List<ObservedObject> targetsInitial = (initial).getEnvironment().getStateTracking();
        List<Car> targetsPredicted = new ArrayList<Car>();
        for(ObservedObject init : targetsInitial) {
            Car initialCar = (Car)init;
            Car propInit = propagateCar(initialCar, tCampaign);
            targetsPredicted.add(propInit);
        }

        // Propagate all targets from their updated final state towards common epoch
        List<ObservedObject> trackedObjs = last.getEnvironment().getStateTracking();
        List<Car> targetsFinal = new ArrayList<Car>();
        for(ObservedObject finalTarget : trackedObjs) {
            Car finalCar = (Car)finalTarget;
            Car propFinal = propagateCar(finalCar, tCampaign);
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
                if(targetsPredicted.get(i).getIdentifier() != targetsFinal.get(j).getIdentifier()){
                    // Move to next object in targetFinals                                                                          
                    j++;
                } else {
                    // Same ID found
                    reward += CarTrackingObjective
                                .computeTraceChange(targetsPredicted.get(i).getCov(), 
                                                    targetsFinal.get(j).getCov());                        
                    // No need to continue searching in targetFinals
                    targetsFinal.remove(j);
                    j=0;
                    break;
                }
            }
        }

        return reward;
    }

    private static Car propagateCar(Car initialCar, double tCampaign) {
        RealMatrix P0 = new Array2DRowRealMatrix(initialCar.getCov());
        double[] y = propagateStateAndSTM(tCampaign, initialCar.getStateArray(), initialCar);
        int n = initialCar.getStateArray().length;
        double[] Xref = new double[n];

        for (int i=0; i<n; i++) {

            // Extract state vector
            double rounded = FastMath.rint(y[i] / epsilon) * epsilon;
            Xref[i] = rounded;
        }
    
        // Extract phi matrix from X (column-major to 2D array)
        double[][] Phik_arr = new double[4][4];
        for (int col = 0; col < n; col++) {
            for (int row = 0; row < n; row++) {
                Phik_arr[row][col] = y[n + col * n + row];
            }
        }
        // Compute propagated uncertainty
        RealMatrix Phik = new Array2DRowRealMatrix(Phik_arr).transpose();

        double[][] gamma = Filter.computeGamma(tCampaign-initialCar.getTime());
        RealMatrix Gamma = new Array2DRowRealMatrix(gamma);
        RealMatrix mappedUnmodelAcc =  Gamma.scalarMultiply(Filter.Q).multiplyTransposed(Gamma);

        RealMatrix Pk_bar = Phik.multiply(P0).multiplyTransposed(Phik)
                                .add(mappedUnmodelAcc);
        Car propInit = 
            new Car(initialCar.getIdentifier(), Xref, Pk_bar.getData(), tCampaign);
        return propInit;
    }

    /**
     * 
     * @param last          last simulated node.
     * @param leaf          last extisting node (without simulated nodes).
     * @return
     */
    public static void computeTrackReward(DecisionNode last, DecisionNode leaf, 
                                          DecisionNode initial, double tCampaign, 
                                          double discount, Sensor sensor, 
                                          RewardFunction selectedReward) {

        double accDiscountedR = 0.;

        if (selectedReward.equals(RewardFunction.REWARD_WRT_SIMULATED_END)) {
            accDiscountedR = computeRewardWrtSimEnd(last, initial, tCampaign);
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
                    accDiscountedR = CarTrackingObjective.computeRegretWrtSimEnd(current, tCampaign)
                                        + discount * accDiscountedR;
                } else if (selectedReward.equals(RewardFunction.IMMEDIATE_REWARD)) {
                    accDiscountedR = CarTrackingObjective.computeImmediateReward(current) 
                                        + discount * accDiscountedR;
                } else if (selectedReward.equals(RewardFunction.REGRET_WRT_FOV)) {
                    accDiscountedR = CarTrackingObjective.computeRegretWrtFov(current, tobs, sensor) 
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

    private static double computeRegretWrtFov(DecisionNode lastDecision, double tobs, 
                                              Sensor sensor){

        double halfFov = sensor.getFov().getHeight()/2.;
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        char lastUpdated = ((CarTrackingObjective)parent.getMacro()).getLastUpdated();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Car sibling = (Car)objEnv;
            if (sibling.getIdentifier() != lastUpdated) {
                Car propSibling = propagateCar(sibling, tobs); //TODO: check that covariance is same as Pk_bar

                RealMatrix Pk_bar = new Array2DRowRealMatrix(propSibling.getCov());
                double[] Xref = propSibling.getStateArray();

                // Transform uncertainty from state space into measurement space
                //double[] H = LinearBearingMeasurementModel.generateHk(Xref).Hk_til;
                double[][] H = LinearRangeBearingMeasurementModel.generateHk(Xref).Hk_til;
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

    private static double computeImmediateReward(DecisionNode current) {
        ChanceNode parent = (ChanceNode) current.getParent();
        double lastUpdatedIG = ((CarTrackingObjective)parent.getMacro()).getLastUpdatedIG();
        return lastUpdatedIG;
    }

    private static double computeRegretWrtSimEnd(DecisionNode lastDecision, double tobs) {
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        char lastUpdated = ((CarTrackingObjective)parent.getMacro()).getLastUpdated();
        //List<Node> siblings = parent.getChildren();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Car sibling = (Car)objEnv;
            if (sibling.getIdentifier() != lastUpdated) {

                Car propSibling = propagateCar(sibling, tobs);
                double[][] predCovSibling = propSibling.getCov();
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

/*     private static double computeRegretWrtFOV(DecisionNode lastDecision, double timeUntilEnd) {
        List<ObservedObject> env = lastDecision.getEnvironment().getStateTracking();
        ChanceNode parent = (ChanceNode) lastDecision.getParent();
        char lastUpdated = ((CarTrackingObjective)parent.getMacro()).getLastUpdated();
        double regret = 0.;
        for(ObservedObject objEnv : env) {
            // Extract sibling 
            Car sibling = (Car)objEnv;
            if (sibling.getIdentifier() != lastUpdated) {
                /* double simMeasPred = 
                    CarTrackingObjective.generateBearingMeasurement(timeUntilEnd, 
                                                                    sibling.getStateArray()); 
                double simMeasPred = 
                    CarTrackingObjective.generateRangeMeasurement(timeUntilEnd, 
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
    } */

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
