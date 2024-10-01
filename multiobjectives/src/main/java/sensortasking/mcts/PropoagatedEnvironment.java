package sensortasking.mcts;

import java.util.List;

import lombok.Getter;
import lombok.Setter;

@Getter
@Setter
public class PropoagatedEnvironment {
    
    /** Updated states of tracking objects. */
    protected List<ObservedObject> stateTracking;

    /** Number of completed tasks inside searching objective. */
    protected List<Integer> stateSearching;

    public PropoagatedEnvironment(List<ObservedObject> targets, List<Integer> completedSearchTasks) {
        this.stateTracking = targets;
        this.stateSearching = completedSearchTasks;
    }
}
