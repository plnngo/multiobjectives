package sensortasking.mcts;

import lombok.Getter;

@Getter
public class ChanceNode extends Node{

    /** Micro action realised as pointing direction. */
    AngularDirection micro;

    /** Macro Action realised as tasking objective. */
    MacroAction macro;
}
