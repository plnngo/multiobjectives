package sensortasking.mcts;

import java.util.List;

import org.orekit.time.AbsoluteDate;

public interface Objective {
    public double utilityWeight = 0;

    public AngularDirection pointingDirection = null;

    public AngularDirection setMicroAction(AbsoluteDate current);

    //public double computeGain();

    public double getUtility();

    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current);

    public List<ObservedObject> propagateOutcome();    
}
