package sensortasking.mcts;

import lombok.Getter;

@Getter
public class Fov {

    /** Field of view type. */
    Type type;

    /** Height or diameter in [rad] of FOV.  */
    private double height;

    /** Width or diameter in [rad] of FOV. */
    private double width;

    /**
     * Simple constructor.
     * 
     * @param type              Type of FOV, either circular or rectangular.
     * @param height            Height or diameter in [rad] of FOV. 
     * @param width             Width or diameter in [rad] of FOV.
     */
    public Fov(Type type, double height, double width) {

        this.type = type;
        this.height = height;
        this.width = width;
    }

    /** Field of view type. */
    enum Type {
        /** Circular field of view. */
        CIRCULAR,

        /** Rectangular field of. */
        RECTANGULAR
    }
}
