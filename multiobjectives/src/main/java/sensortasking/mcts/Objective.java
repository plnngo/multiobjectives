package sensortasking.mcts;

import java.util.List;

import org.orekit.time.AbsoluteDate;

public interface Objective {

    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing);

    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current);

    public List<ObservedObject> propagateOutcome();    
}
