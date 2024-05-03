package sensortasking.mcts;


public interface MacroAction {
    public double utilityWeight = 0;

    //public static final Map<Outcome, Double> possibleOutcome = new HashMap<Outcome, Double>();

    public AngularDirection setMicroAction();

    public double computeGain();

    public double getUtility();

    //public Map.Entry<Outcome, Double> sampleOutcome();
    
}
