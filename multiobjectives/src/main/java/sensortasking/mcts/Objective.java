package sensortasking.mcts;

import java.util.List;

import org.orekit.time.AbsoluteDate;

public interface Objective<T> {

    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing);

    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current);

    public List<T> propagateOutcome();    
}
