package benchtest;

import org.hipparchus.ode.OrdinaryDifferentialEquation;
import org.hipparchus.util.FastMath;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import sensortasking.mcts.ObservedObject;

public class Satellite extends ObservedObject implements OrdinaryDifferentialEquation{

    private long identifier;
    
    private double posX;

    private double posY;

    private double posZ;

    private double velX;

    private double velY;

    private double velZ;

    private double[][] cov;

    static final int dim = 6;

    private static double mu = Constants.WGS84_EARTH_MU;

    public Satellite(long id, StateVector state, CartesianCovariance covariance, 
                     AbsoluteDate epoch, Frame frame) {

        super(id, state, covariance, epoch, frame);

        this.identifier = id;
        this.posX = state.getPositionVector().getX();
        this.posY = state.getPositionVector().getY();   
        this.posZ = state.getPositionVector().getZ();
        this.velX = state.getVelocityVector().getX();
        this.velY = state.getVelocityVector().getY();
        this.velZ = state.getVelocityVector().getZ();   
        this.cov = covariance.getCovarianceMatrix().getData();        
    }

    private static double[] int_stm(double[] X) {

        // X = [x, y, vx, vy, phi (4x4 matrix flattened in column-major order)]

        int stateSize = 4;
        int stmSize = 16; // 4x4 STM
        double[] dX = new double[stateSize + stmSize];

        // Define the constant velocity system matrix A
        double[][] A = twoBodyDynamics(X);

        // Extract phi matrix from X (column-major to 2D array)
        double[][] phi = new double[4][4];
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                phi[row][col] = X[4 + col * 4 + row];
            }
        }

        // Compute state derivative
        dX = computeTwoBodyDerivative(A, X, dX);
        
        // Compute dphi = A * phi
        double[][] dphi = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                dphi[i][j] = 0;
                for (int k = 0; k < 4; k++) {
                    dphi[i][j] += A[i][k] * phi[k][j];
                }
            }
        }

        // Flatten dphi (column-major) into dX
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                dX[4 + col * 4 + row] = dphi[row][col];
            }
        }
        return dX;
    }

  

    private static double[] computeTwoBodyDerivative(double[][] A, double[] X, double[] dX) {

        // Compute powers of range
        double r = FastMath.sqrt(X[0]*X[0] + X[1]*X[1] + X[2]*X[2]);
        double r3 = r * r * r;

        return new double[]{X[3], X[4], X[5], -X[0]*mu/r3, -X[1]*mu/r3, -X[2]*mu/r3};
    }

    private static double[][] twoBodyDynamics(double[] X) {

        // Compute powers of range
        double r = FastMath.sqrt(X[0]*X[0] + X[1]*X[1] + X[2]*X[2]);
        double r3 = r * r * r;
        double r5 = r3 * r * r;

        // Compute A matrix entries
        double A41 = -mu * (1/r3 - 3*X[0]*X[0]/r5);
        double A42 = 3*mu*X[0]*X[1]/r5;
        double A43 = 3*mu*X[0]*X[2]/r5;
        double A52 = -mu * (1/r3 - 3*X[1]*X[1]/r5);
        double A53 = 3*mu*X[1]*X[2]/r5;
        double A63 = -mu * (1/r3 - 3*X[2]*X[2]/r5);

        double[][] A = new double[dim][dim];
        A[0] = new double[]{0, 0, 0, 1, 0, 0};
        A[1] = new double[]{0, 0, 0, 0, 1, 0};
        A[2] = new double[]{0, 0, 0, 0, 0, 1};
        A[3] = new double[]{A41, A42, A43, 0, 0, 0};
        A[4] = new double[]{A42, A52, A53, 0, 0, 0};
        A[5] = new double[]{A43, A53, A63, 0, 0, 0};

        return A;
    }

    @Override
    public int getDimension() {
        return dim + dim*dim;
    }

    @Override
    public double[] computeDerivatives(double t, double[] state) {
        return int_stm(state);
    }
    
}
