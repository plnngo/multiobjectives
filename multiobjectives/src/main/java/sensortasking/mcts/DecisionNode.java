package sensortasking.mcts;

import java.util.List;

import org.orekit.time.AbsoluteDate;

import lombok.Getter;

@Getter
public class DecisionNode extends Node{

    /** Sensor pointing location. */
    AngularDirection sensorPointing;

    /** Prioritisation weight vector. 1st entry refers to search, 2nd to tracking objective. */
    double[] weights;

    /** Time durations dedicated for each objective. 1st entry refers to search, 2nd to tracking 
     * objective. */
    double[] timeResources;

    /** Propoagated environment under the influence of the last macro/micro action. */
    List<ObservedObject> propEnvironment;

    public DecisionNode(double utility, int numVisits, AngularDirection pointing, double[] weights,
                        double[] timeResources, AbsoluteDate epoch, List<ObservedObject> propEnvironment) {

        this.sensorPointing = pointing;
        this.weights = weights;
        this.timeResources = timeResources;
        super.utility = utility;
        super.numVisits = numVisits;
        this.propEnvironment = propEnvironment;
        super.setEpoch(epoch);
    }
}
