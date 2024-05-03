package sensortasking.mcts;

import java.util.HashMap;
import java.util.Map;

public interface MacroAction {
    public double utilityWeight = 0;

    //public static final Map<Outcome, Double> possibleOutcome = new HashMap<Outcome, Double>();

    public ChanceNode setMicroAction();

    public double computeGain();

    public double getUtility();

    //public Map.Entry<Outcome, Double> sampleOutcome();
    
}
