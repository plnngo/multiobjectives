package sensortasking.mcts;

import java.util.Map;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.orekit.time.AbsoluteDate;


public class DecisionNode extends Node{

    /** Sensor pointing location. */
    private AngularDirection sensorPointing;

    /** Propagated environment under the influence of the last tracking action. */
    PropoagatedEnvironment environment;

    /** User-defined weights of searching tasks. First entry stripe scan, second bullseye scan. */
    protected double[] weightsSearch = new double[]{1., 0.};

    /** Search discrepancy vectors of all existing leaf nodes */
    Map<Long, double[]> searchDiscrepancyVec = new HashMap<Long, double[]>();

    /** Utility vectors of all existing leaf nodes.*/
    Map<Long, double[]> allUtilityVecs = new HashMap<Long, double[]>();

    /** IDs of leafs that have been removed as they are not considered as optimal anymore. */
    List<Long> removedLeafs = new ArrayList<Long>();

    /** Node ID counter. */
    protected long idCounter = 0;

    /** Time spent on stripe scanning method. */
    double timeStripes = 0.;

    public DecisionNode(double utility, int numVisits, AngularDirection pointing, 
                        AbsoluteDate epoch, PropoagatedEnvironment environment, long id, 
                        double depth, double timeStripes) {

        this.sensorPointing = pointing;
        super.utility = utility;
        super.numVisits = numVisits;
        this.environment = environment;
        super.setEpoch(epoch);
        super.setId(id);
        int numObj = 1;         // Search by default always activated
        if(!environment.getStateSearching().isEmpty()) {
            numObj++;
        }
        if(!environment.getStateTracking().isEmpty()) {
            numObj = numObj + environment.getStateTracking().size();
        }
        super.setUtilityVec(new double[numObj]);
        super.setDepth(depth);
        this.timeStripes = timeStripes;
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

    public void addUtilityVec(long id, double[] toAdd) {
        allUtilityVecs.put(id, toAdd);
    }

    public void removeUtilityVec(long id) {
        if (this.allUtilityVecs.containsKey(id)) {
            this.allUtilityVecs.remove(id);
            removedLeafs.add(id);
        } else if (this.allUtilityVecs.size()==0 && id ==0) {
            // nothing to be removed
        }
        else {
            for (int i=0; i<removedLeafs.size(); i++) {
                if (removedLeafs.get(i) == id) {
                    // nothing to be removed
                    break;
                }
                if (i == removedLeafs.size()-1) {
                    throw new IllegalArgumentException("List of utility vector does not contain the ID " 
                                                + "that shall get removed");
                }
            }
        }
    }

    public void setEpochSensorPointing(AbsoluteDate epoch) {
        this.sensorPointing.setDate(epoch);
    }

    public Map<Long, double[]> getAllUtilityVecs() {
        return this.allUtilityVecs;
    }

    public AngularDirection getSensorPointing() {
        return this.sensorPointing;
    }

    public PropoagatedEnvironment getEnvironment() {
        return this.environment;
    }

    public double getTimeSpentStripe() {
        return this.timeStripes;
    }

/*     public double[] getWeights() {
        return this.weights;
    }

    public double[] getTimeResources() {
        return this.timeResources;
    } */
}
