package benchtest;

import org.hipparchus.ode.OrdinaryDifferentialEquation;

public class Car implements OrdinaryDifferentialEquation{

    private char id;
    
    private double posX;

    private double posY;

    private double velX;

    private double velY;

    private double time;

    final int dim = 4;

    public Car(char id, double x, double y, double xdot, double ydot, double t) {
        this.id = id;
        this.posX = x;
        this.posY = y;
        this.velX = xdot;
        this.velY = ydot;
        this.time = t;
    }

    public Car(char id, double[] state, double t) {
        this.id = id;
        this.posX = state[0];
        this.posY = state[1];
        this.velX = state[2];
        this.velY = state[3];
        this.time = t;
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
}
