package benchtest;

import org.apache.commons.lang3.ArrayUtils;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.ExpandableODE;
import org.hipparchus.ode.ODEIntegrator;
import org.hipparchus.ode.ODEState;
import org.hipparchus.ode.ODEStateAndDerivative;
import org.hipparchus.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.hipparchus.util.FastMath;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;

import sensortasking.mcts.ObservedObject;

public class Satellite extends ObservedObject{

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

        // X = [x, y, z, vx, vy, vz phi (6x6 matrix flattened in column-major order)]

        int stateSize = dim;
        int stmSize = dim*dim; // 6x6 STM
        double[] dX = new double[stateSize + stmSize];

        // Define the constant velocity system matrix A
        double[][] A = twoBodyDynamics(X);

        // Extract phi matrix from X (column-major to 2D array)
        double[][] phi = new double[dim][dim];
        for (int col = 0; col < dim; col++) {
            for (int row = 0; row < dim; row++) {
                phi[row][col] = X[dim + col * dim + row];
            }
        }

        // Compute state derivative
        dX = computeTwoBodyDerivative(A, X, dX);
        
        // Compute dphi = A * phi
        double[][] dphi = new double[dim][dim];
        for (int i = 0; i < dim; i++) {
            for (int j = 0; j < dim; j++) {
                dphi[i][j] = 0;
                for (int k = 0; k < dim; k++) {
                    dphi[i][j] += A[i][k] * phi[k][j];
                }
            }
        }

        // Flatten dphi (column-major) into dX
        for (int col = 0; col < dim; col++) {
            for (int row = 0; row < dim; row++) {
                dX[dim + col * dim + row] = dphi[row][col];
            }
        }
        return dX;
    }

  

    private static double[] computeTwoBodyDerivative(double[][] A, double[] X, double[] dX) {

        // Initialise output
        double[] out = new double[dX.length];

        // Compute powers of range
        double r = FastMath.sqrt(X[0]*X[0] + X[1]*X[1] + X[2]*X[2]);
        double r3 = r * r * r;

        out[0] = X[3];
        out[1] = X[4];
        out[2] = X[5];
        out[3] = -X[0]*mu/r3;
        out[4] = -X[1]*mu/r3;
        out[5] = -X[2]*mu/r3;

        return out;
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

    public static Satellite propagateSatellite(Satellite initialSat, AbsoluteDate start, AbsoluteDate end) {
        RealMatrix P0 = 
            new Array2DRowRealMatrix(initialSat.getCovariance().getCovarianceMatrix().getData());
        double[] initStateVec = new double[]{initialSat.getState().getPositionVector().getX(),
                                             initialSat.getState().getPositionVector().getY(),
                                             initialSat.getState().getPositionVector().getZ(),
                                             initialSat.getState().getVelocityVector().getX(),
                                             initialSat.getState().getVelocityVector().getY(),
                                             initialSat.getState().getVelocityVector().getZ()};

        double[] y = propagateStateAndSTM(start, end, initStateVec, initialSat);
        int n = initStateVec.length;
        double[] Xref = new double[n];
    
        // Extract phi matrix from X (column-major to 2D array)
        double[][] Phik_arr = new double[4][4];
        for (int col = 0; col < n; col++) {
            for (int row = 0; row < n; row++) {
                Phik_arr[row][col] = y[n + col * n + row];
            }
        }
        // Compute propagated uncertainty
        RealMatrix Phik = new Array2DRowRealMatrix(Phik_arr).transpose();

        double[][] gamma = Filter.computeGamma(end.durationFrom(start));
        RealMatrix Gamma = new Array2DRowRealMatrix(gamma);
        RealMatrix mappedUnmodelAcc =  Gamma.scalarMultiply(Filter.Q).multiplyTransposed(Gamma);

        RealMatrix Pk_bar = Phik.multiply(P0).multiplyTransposed(Phik)
                                .add(mappedUnmodelAcc);
        Satellite propInit = new Satellite(initialSat.getId(), 
                                           ObservedObject.arrayToStateVector(Xref), 
                                           ObservedObject.arrayToCartesianCov(Pk_bar.getData()), 
                                           end,
                                           initialSat.getFrame());
        return propInit;
    }

    private static double[] propagateStateAndSTM(AbsoluteDate start, AbsoluteDate end, 
                                                 double[] initState, Satellite sat) {
        int n = initState.length;

        // Combine initial state and STM (identity matrix)
        double[][] identity = new double[n][n];
        for (int col=0; col<n; col++) {
            for (int row=0; row<n; row++) {
                if(row==col) {
                    identity[row][col] = 1.;
                } else {
                    identity[row][col] = 0;
                }
            }
        }
        RealMatrix ones = new Array2DRowRealMatrix(identity);
        double[] ones_arr = Filter.flattenRowMajor(ones.getData());
        double[] Xref_Stm0 = ArrayUtils.addAll(initState, ones_arr);
        double[] y = new double[Xref_Stm0.length];

        ExpandableODE expandable = new ExpandableODE(sat);
        ODEIntegrator integrator = new ClassicalRungeKuttaIntegrator(0.01);
        ODEState initial = new ODEState(0., Xref_Stm0);

        if(end.durationFrom(start) < 1e-16) {
            y = Xref_Stm0;
        } else {
            ODEStateAndDerivative finalState = integrator.integrate(expandable, initial, 
                                                                    end.durationFrom(start));
            y = finalState.getPrimaryState();
        }
        return y;
    }
}
