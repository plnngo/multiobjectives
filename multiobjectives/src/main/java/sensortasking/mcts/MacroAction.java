package sensortasking.mcts;

import java.util.List;

public interface MacroAction {
    public double utilityWeight = 0;

    //public static final Map<Outcome, Double> possibleOutcome = new HashMap<Outcome, Double>();

    public AngularDirection setMicroAction();

    public double computeGain();

    public double getUtility();

    public double getExecusionDuration();

    public List<ObservedObject> propagateOutcome();    
}
