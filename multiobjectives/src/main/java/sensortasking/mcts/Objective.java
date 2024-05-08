package sensortasking.mcts;

import java.util.List;

public interface Objective {
    public double utilityWeight = 0;

    public AngularDirection pointingDirection = null;

    public AngularDirection setMicroAction();

    //public double computeGain();

    public double getUtility();

    public double getExecusionDuration();

    public List<ObservedObject> propagateOutcome();    
}
