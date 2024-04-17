package sensortasking.mcts;

public interface MacroAction {
    public double utilityWeight = 0;

    public ChanceNode setMicroAction();

    public double computeGain();

    public double getUtility();

    
}
