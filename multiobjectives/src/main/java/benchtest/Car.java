package benchtest;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.ArrayUtils;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.ExpandableODE;
import org.hipparchus.ode.ODEIntegrator;
import org.hipparchus.ode.ODEState;
import org.hipparchus.ode.ODEStateAndDerivative;
import org.hipparchus.ode.OrdinaryDifferentialEquation;
import org.hipparchus.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.orekit.frames.FramesFactory;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;
import lombok.Setter;
import sensortasking.mcts.ObservedObject;

@Getter
@Setter
public class Car extends ObservedObject implements OrdinaryDifferentialEquation{

    private char identifier;
    
    private double posX;

    private double posY;

    private double velX;

    private double velY;

    private double time;

    private double[][] cov;

    static final int dim = 4;

    static AbsoluteDate origin = new AbsoluteDate();

    public Car(char id, double x, double y, double xdot, double ydot, double[][] cov, double t) {
        super(id, null, null, new AbsoluteDate().shiftedBy(t), FramesFactory.getEME2000());
        this.identifier = id;
        this.posX = x;
        this.posY = y;
        this.velX = xdot;
        this.velY = ydot;
        this.time = t;
        this.cov = new double[cov.length][];
        for (int i = 0; i < cov.length; i++) {
            this.cov[i] = cov[i].clone();
        }
    }

    public Car(char id, double[] state, double[][] cov, double t) {
        super(id, null, null, new AbsoluteDate().shiftedBy(t), FramesFactory.getEME2000());
        this.identifier = id;
        this.posX = state[0];
        this.posY = state[1];
        this.velX = state[2];
        this.velY = state[3];
        this.time = t;
        this.cov = new double[cov.length][];
        for (int i = 0; i < cov.length; i++) {
            this.cov[i] = cov[i].clone();
        }
    }

    public void setState(double posX, double posY, double velX, double velY){

        this.posX = posX;
        this.posY = posY;
        this.velX = velX;
        this.velY = velY;
    }



    private static double[] int_constant_vel_stm(double[] X) {

        // X = [x, y, vx, vy, phi (4x4 matrix flattened in column-major order)]

        int stateSize = 4;
        int stmSize = 16; // 4x4 STM
        double[] dX = new double[stateSize + stmSize];

        // Define the constant velocity system matrix A
        double[][] A = {
            {0, 0, 1, 0},
            {0, 0, 0, 1},
            {0, 0, 0, 0},
            {0, 0, 0, 0}
        };

        // Extract phi matrix from X (column-major to 2D array)
        double[][] phi = new double[4][4];
        for (int col = 0; col < 4; col++) {
            for (int row = 0; row < 4; row++) {
                phi[row][col] = X[4 + col * 4 + row];
            }
        }

        // Compute A * X(1:4)
        for (int i = 0; i < 4; i++) {
            dX[i] = 0;
            for (int j = 0; j < 4; j++) {
                dX[i] += A[i][j] * X[j];
            }
        }

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

    @Override
    public double[] computeDerivatives(double time, double[] state) {
        return int_constant_vel_stm(state);
    }

    @Override
    public int getDimension() {
        return dim + dim*dim;
    }

    public double[] getStateArray(){
        return new double[]{this.posX, this.posY, this.velX, this.velY};
    }

    public static List<Car> propagateCars(List<Car> initial, AbsoluteDate end) {

        // Simulation duration
        double simDuration = end.durationFrom(Car.origin);

        // Initialise output                                                    
        List<Car> out = new ArrayList<Car>();

        // Combine initial state and STM (identity matrix)
        double[][] identity = new double[Car.dim][Car.dim];
        for (int col=0; col<Car.dim; col++) {
            for (int row=0; row<Car.dim; row++) {
                if(row==col) {
                    identity[row][col] = 1.;
                } else {
                    identity[row][col] = 0;
                }
            }
        }
        RealMatrix ones = new Array2DRowRealMatrix(identity);
        double[] ones_arr = Filter.flattenRowMajor(ones.getData());
        
        for (Car car : initial) {

            ExpandableODE expandable = new ExpandableODE(car);
            double[] X0_ref = new double[]{car.getPosX(), car.posY, car.getVelX(), car.getVelY()};
            double[] Xref_Stm0 = ArrayUtils.addAll(X0_ref, ones_arr);
            double step = 0.01;

            // Extract propagated state
            double[] Xref = new double[Car.dim];
            double[][] Phik_arr = new double[4][4];

            if (simDuration-car.getTime() < step) {

                // prevent propagation to initial time stamp
                Xref = X0_ref;
                Phik_arr = identity;

            } else {
                ODEIntegrator integrator = new ClassicalRungeKuttaIntegrator(step);
                ODEState initialState = new ODEState(0., Xref_Stm0);
                ODEStateAndDerivative finalState = 
                    integrator.integrate(expandable, initialState, simDuration-car.getTime());
                double[] y = finalState.getPrimaryState();
                for (int i=0; i<Car.dim; i++) {

                    // Extract state vector
                    Xref[i] = y[i];
                }

                // Extract phi matrix from X (column-major to 2D array)
                for (int col = 0; col < Car.dim; col++) {
                    for (int row = 0; row < Car.dim; row++) {
                        Phik_arr[row][col] = y[Car.dim + col * Car.dim + row];
                    }
                }
            }
            RealMatrix Phik = new Array2DRowRealMatrix(Phik_arr).transpose();

            // Predicted covariance
            RealMatrix P0 = new Array2DRowRealMatrix(car.getCov());
            RealMatrix Pk_bar = Phik.multiply(P0).multiplyTransposed(Phik);

            Car propCar = new Car(car.getIdentifier(), Xref, Pk_bar.getData(), simDuration);
            out.add(propCar);
        }
        
        return out;
    }
}
