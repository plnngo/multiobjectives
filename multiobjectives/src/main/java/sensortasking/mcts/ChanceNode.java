package sensortasking.mcts;

import java.util.Map.Entry;

import lombok.Getter;

@Getter
public class ChanceNode extends Node{

    public ChanceNode(MacroAction objective, ChanceNode microAction, Node leaf, int i,
            Entry<Outcome, Double> outcomeReward) {
        //TODO Auto-generated constructor stub
    }

    /** Micro action realised as pointing direction. */
    AngularDirection micro;

    /** Macro Action realised as tasking objective. */
    MacroAction macro;
}
