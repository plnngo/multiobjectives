package sensortasking.mcts;

import java.util.Map;
import java.util.HashMap;

import org.orekit.time.AbsoluteDate;


public class DecisionNode extends Node{

    /** Sensor pointing location. */
    private AngularDirection sensorPointing;

    /** Prioritisation weight vector. 1st entry refers to search, 2nd to tracking objective. */
    double[] weights;

    /** Time durations dedicated for each objective. 1st entry refers to search, 2nd to tracking 
     * objective. */
    double[] timeResources;

    /** Propoagated environment under the influence of the last tracking action. */
    PropoagatedEnvironment environment;

    /** User-defined weights of searching tasks. First entry stripe scan, second bullseye scan. */
    protected double[] weightsSearch = new double[]{1., 0.};

    /** Search discrepancy vectors of all existing leaf nodes */
    Map<Long, double[]> searchDiscrepancyVec = new HashMap<Long, double[]>();

    /** Node ID counter. */
    protected long idCounter = 1;


    public DecisionNode(double utility, int numVisits, AngularDirection pointing, double[] weights,
                        double[] timeResources, AbsoluteDate epoch, PropoagatedEnvironment environment, long id) {

        this.sensorPointing = pointing;
        this.weights = weights;
        this.timeResources = timeResources;
        super.utility = utility;
        super.numVisits = numVisits;
        this.environment = environment;
        super.setEpoch(epoch);
        super.setId(id);
    }

    public DecisionNode setWeightsSearchingTask(double[] weights) {
        this.weightsSearch = weights;
        return this;
    }

    public long incrementIdCounter() {
        this.idCounter++;
        return this.idCounter;
    }

    public double[] getWeightsSearch() {
        return this.weightsSearch;
    }

    public void addSearchDiscrepancyVec(long id, double[] toAdd) {
        searchDiscrepancyVec.put(Long.valueOf(id), toAdd);
    }

    public Map<Long, double[]> getSearchDiscrepancyVecs() {
        return this.searchDiscrepancyVec;
    }

    public AngularDirection getSensorPointing() {
        return this.sensorPointing;
    }

    public PropoagatedEnvironment getEnvironment() {
        return this.environment;
    }

    public double[] getWeights() {
        return this.weights;
    }

    public double[] getTimeResources() {
        return this.timeResources;
    }
}
