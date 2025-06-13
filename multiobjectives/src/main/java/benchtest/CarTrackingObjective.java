package benchtest;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

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

    public CarTrackingObjective(List<ObservedObject> targets, AbsoluteDate startCampaign, AbsoluteDate endCampaign) {

        // Initialise list of targets
        for (ObservedObject target : targets) {
            updatedTargets.add((Car)target);
        }
        this.start = startCampaign;
        this.end = endCampaign;
    }

    @Override
    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing) {

        double time = current.durationFrom(this.start) + 1.;

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
            double simMeas = generateMeasurement(time, state);
            Filter est = new Filter();
            est.run_ckf(state, copy.getCov(), copy.getTime(), time, simMeas);

            // Compute information gain
            double iG = 
                computeKLDivergence(est.statePred, est.stateCorr, 
                                                 est.covPred, est.covCorr);
            Car copyUpdated = new Car(copy.getIdentifier(), est.stateCorr, est.covCorr, time);
            checkTrackable.put(copyUpdated, iG);
        }

        // Compare IG
        Car selected = new Car('f', new double[]{0.,0.,0.,0.}, new double[4][4], time);
        Double iGmax = Double.MIN_VALUE;
        
        for (Entry<Car, Double> entry : checkTrackable.entrySet()) {

            if(entry.getValue() > iGmax) {
                iGmax = entry.getValue();
                double[] stateUpdated = 
                    new double[]{entry.getKey().getPosX(), entry.getKey().getPosY(), 
                                 entry.getKey().getVelX(), entry.getKey().getVelY()};
                selected = new Car(entry.getKey().getIdentifier(), stateUpdated, 
                                   entry.getKey().getCov(),time);
            }
        }

        // Update targets
        for(Car candidate : updatedTargets) {
            if(candidate.getIdentifier() == selected.getIdentifier()) {
                candidate.setState(selected.getPosX(), selected.getPosY(), 
                                   selected.getVelX(), selected.getVelY());
                candidate.setCov(selected.getCov());
                candidate.setTime(selected.getTime());
                candidate.setEpoch(selected.getEpoch());
                this.lastUpdated = selected.getIdentifier();
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

    protected static double computeKLDivergence(double[] statePrior, 
                                                             double[] statePost, 
                                                             double[][] covPrior, 
                                                             double[][] covPost) {
        
        // Retrieve covariances
        RealMatrix covP = new Array2DRowRealMatrix(covPrior);
        RealMatrix covQ = new Array2DRowRealMatrix(covPost);

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

    private double generateMeasurement(double time, double[] state) {
        // Ensure that object only moves with constant velocity along X axis
        if (FastMath.abs(state[3])>0.1) {
            throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                    + "Y axis");
        } else {
            //double posXCurrent = state[0];
            double velX = state[2];
            double dist = time * velX;
            double posXNew = dist;
            double[] stateNew = new double[]{posXNew, state[1], velX, state[3]}; 
            double simMeas = LinearRangeMeasurementModel.generateHk(stateNew).Gk;
            return simMeas;
        }
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {

        double exeTime = 2.;
        if(current.durationFrom(this.start) < 0.9) {
            exeTime = 2.;       // Integration in setMicroAction fails for delta t = 0;
        } 
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

    public static double[] computeTrackReward(DecisionNode last, AbsoluteDate end) {

        // Convert observedObject to car
        List<ObservedObject> trackedObjs = last.getEnvironment().getStateTracking();
        List<Car> trackedCars = transformObservedObjectsToCars(trackedObjs);

        // Initialise output
        double[] out = new double[trackedObjs.size()];

        // Propagate all targets from their intial state towards common epoch with Kepler dynamics
        Node root = last;
        while (root.getParent() != null) {
            root = root.getParent();
        }
        List<ObservedObject> targetsInitial = 
            ((DecisionNode)root).getEnvironment().getStateTracking();
        List<Car> carsInitial = transformObservedObjectsToCars(targetsInitial);
        List<Car> targetsPredicted = Car.propagateCars(carsInitial, end);

        // Propagate all targets from their updated final state towards common epoch
        //List<ObservedObject> targetsUpdated = last.getEnvironment().getStateTracking();
        List<Car> targetsFinal = Car.propagateCars(trackedCars, end);

        // Calculate information gain
        if(targetsPredicted.size() != targetsFinal.size()) {
            throw new IllegalArgumentException("Information gain cannot be computed due to " 
                                                + "dimension error in targets.");
        }
        double accumulatedIG = 0;

        for(int i=0; i<targetsPredicted.size(); i++) {
            int j=0;
            while(j<targetsFinal.size()) {

                // Make sure that ID of objects are the same when computing information gain
                if(targetsPredicted.get(i).getIdentifier() != targetsFinal.get(j).getIdentifier()) {
                    // Move to next object in targetFinals                                                                          
                    j++;
                } else {
                    // Same ID found
                    // TODO: check if i=0 is always A and i=1 is B
                    out[i] = computeKLDivergence(targetsPredicted.get(i).getStateArray(), 
                                                 targetsFinal.get(j).getStateArray(), 
                                                 targetsPredicted.get(i).getCov(), 
                                                 targetsFinal.get(j).getCov());
                    accumulatedIG += out[i];
                        
                    // No need to continue searching in targetFinals
                    targetsFinal.remove(j);
                    j=0;
                    break;
                }
            }
        }

        return out;
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
