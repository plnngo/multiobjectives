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

import sensortasking.mcts.AngleType;
import sensortasking.mcts.AngularDirection;
import sensortasking.mcts.Objective;
import sensortasking.mcts.ObservedObject;

@SuppressWarnings("rawtypes")
public class CarTrackingObjective implements Objective{

    List<Car> updatedTargets = new ArrayList<Car>();

    AbsoluteDate start;

    AbsoluteDate end;

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
        // List of candidates that might be trackable
        Map<Car, Double> checkTrackable = new HashMap<Car, Double>();        
        for (Car obj : updatedTargets) {
            double[] state = new double[]{obj.getPosX(), obj.getPosY(), obj.getVelX(), obj.getVelY()};
            Car copy = new Car(obj.getIdentifier(), state, obj.getCov(), obj.getTime());
            double time = current.durationFrom(this.start);

            // Simulate measuremement
            double simMeas = generateMeasurement(time, state);
            Filter est = new Filter();
            est.run_ckf(state, copy.getCov(), time, simMeas);

            // Compute information gain
            double iG = 
                computeKullbackLeiblerDivergence(est.statePred, est.stateCorr, 
                                                 est.covPred, est.covCorr);
            Car copyUpdated = new Car(copy.getIdentifier(), est.stateCorr, est.covCorr, time);
            checkTrackable.put(copyUpdated, iG);
        }

        // Compare IG
        Car selected = new Car('f', null, null, 0);
        Double iGmax = Double.MIN_VALUE;
        
        for (Entry<Car, Double> entry : checkTrackable.entrySet()) {

            if(entry.getValue() > iGmax) {
                iGmax = entry.getValue();
                double[] stateUpdated = 
                    new double[]{entry.getKey().getPosX(), entry.getKey().getPosY(), 
                                 entry.getKey().getVelX(), entry.getKey().getVelY()};
                selected = new Car(entry.getKey().getIdentifier(), stateUpdated, 
                                   entry.getKey().getCov(), entry.getKey().getTime());
            }
        }
        double alpha = FastMath.atan2(selected.getPosX(), selected.getPosY());
        double range = FastMath.sqrt(selected.getPosX() * selected.getPosX() 
                                        + selected.getPosY() * selected.getPosY());
        AngularDirection angle = new AngularDirection(FramesFactory.getEME2000(), 
                                                      new double[]{alpha, 0.}, 
                                                      AngleType.RADEC, range);
        return angle;
    }

    protected static double computeKullbackLeiblerDivergence(double[] statePrior, 
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
        double[] meanQMinusMeanP = new double[6];
        
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
            
        if (Double.isNaN(dKL)) {
            System.out.println("why nan");
        }
        
        return dKL;
    }

    private double generateMeasurement(double time, double[] state) {
        // Ensure that object only moves with constant velocity along X axis
        if (state[3]>0 || state[3]<0) {
            throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                    + "Y axis");
        } else {
            double posXCurrent = state[0];
            double velX = state[2];
            double dist = time * velX;
            double posXNew = dist + posXCurrent;
            double[] stateNew = new double[]{posXNew, state[1], velX, state[3]}; 
            double simMeas = LinearRangeMeasurementModel.generateHk(stateNew).Gk;
            return simMeas;
        }
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current.shiftedBy(1.)};
        return interval;
    }

    @Override
    public List propagateOutcome() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'propagateOutcome'");
    }
    
}
