package benchtest;

import org.apache.commons.lang3.ArrayUtils;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.ode.ExpandableODE;
import org.hipparchus.ode.ODEIntegrator;
import org.hipparchus.ode.ODEState;
import org.hipparchus.ode.ODEStateAndDerivative;
import org.hipparchus.ode.nonstiff.ClassicalRungeKuttaIntegrator;
import org.hipparchus.util.FastMath;

import benchtest.LinearRangeMeasurementModel.MeasurementModel;


public class Filter {

    double[] stateCorr;

    double[][] covCorr;

    double[] statePred;

    double[][] covPred;

    /** Process noise. */
    double Q = 1e-8;

    /** Measurement noise. */
    double Rk = 0.02;

    public void run_ckf(double[] X0_ref, double[][] P_pre, double t_obs, double obs_data) {

        RealMatrix P0 = new Array2DRowRealMatrix(P_pre);
        RealMatrix Rk_mat = new Array2DRowRealMatrix(new double[]{Rk});

        // State dimension
        int n = X0_ref.length;

        // Error state
        double[] xhat_pre = new double[n];

        // Combine initial state and STM (identity matrix)
        RealMatrix ones = new DiagonalMatrix(new double[]{1., 1., 1., 1.});
        double[] ones_arr = flattenRowMajor(ones.getData());
        double[] Xref_Stm0 = ArrayUtils.addAll(X0_ref, ones_arr);

        Car carA_0 = new Car('A', X0_ref, P_pre, 0);
        ExpandableODE expandable = new ExpandableODE(carA_0);

        ODEIntegrator integrator = new ClassicalRungeKuttaIntegrator(0.01);
        ODEState initialState = new ODEState(0., Xref_Stm0);
        ODEStateAndDerivative finalState = integrator.integrate(expandable, initialState, t_obs);
        if (FastMath.abs(t_obs-finalState.getTime())>1e-2) {
            throw new IllegalArgumentException("Did not propagate the state to the observation" 
                                                + "epoch");
        }

        // Extract propagated state
        double[] y = finalState.getPrimaryState();
        double[] Xref = new double[n];
        for (int i=0; i<n; i++) {

            // Extract state vector
            Xref[i] = y[i];
        }
        RealVector Xref_vec = new ArrayRealVector(Xref);
        this.statePred = Xref.clone();

        // Extract phi matrix from X (column-major to 2D array)
        double[][] Phik_arr = new double[4][4];
        for (int col = 0; col < n; col++) {
            for (int row = 0; row < n; row++) {
                Phik_arr[row][col] = y[n + col * n + row];
            }
        }
        RealMatrix Phik = new Array2DRowRealMatrix(Phik_arr);
        double[][] gamma = computeGamma(0, t_obs);
        RealMatrix Gamma = new Array2DRowRealMatrix(gamma);

        // Predicted correction 
        double[] xk_bar = Phik.operate(xhat_pre);
        RealMatrix xk_bar_mat = new Array2DRowRealMatrix(xk_bar);

        // Predicted covariance
        RealMatrix mappedUnmodelAcc =  Gamma.scalarMultiply(Q).multiply(Gamma.transpose());
        RealMatrix Pk_bar = Phik.multiply(P0).multiplyTransposed(Phik).add(mappedUnmodelAcc);
        this.covPred = Pk_bar.getData();

        // Compute system noise mapping matrix
        MeasurementModel measModel = LinearRangeMeasurementModel.generateHk(Xref); 
        double innov = obs_data - measModel.Gk;
        double[] hk_til = measModel.Hk_til;
        RealMatrix Hk_til = new Array2DRowRealMatrix(hk_til).transpose();

        // Kalman gain
        RealMatrix S = Hk_til.multiply(Pk_bar).multiplyTransposed(Hk_til).add(Rk_mat);
        RealMatrix Kk = Pk_bar.multiplyTransposed(Hk_til).multiply(MatrixUtils.inverse(S));

        // Correction
        double[] xhat = xk_bar_mat.add(Kk.scalarMultiply(innov - Hk_til.operate(xk_bar)[0]))
                                  .getColumn(0);
        RealVector xhat_vec = new ArrayRealVector(xhat);
        RealVector Xref_out = Xref_vec.add(xhat_vec);
        this.stateCorr = Xref_out.toArray();

        // Joseph-form covariance update 
        RealMatrix kalmanCorr = ones.subtract(Kk.multiply(Hk_til));
        RealMatrix P_out = kalmanCorr.multiply(Pk_bar)
                                     .multiplyTransposed(kalmanCorr)
                                     .add(Kk.multiply(Rk_mat).multiplyTransposed(Kk));
        this.covCorr = P_out.getData();
    }

    public static double[] flattenRowMajor(double[][] matrix) {
        int rows = matrix.length;
        int cols = matrix[0].length;
        double[] flat = new double[rows * cols];

        int index = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                flat[index++] = matrix[i][j];
            }
        }
        return flat;
    }

    public static double[][] computeGamma(double t_pre, double t_cur) {
        double dt = t_cur - t_pre;

        double[][] Gamma = new double[4][2];
        double halfDtSquared = 0.5 * dt * dt;

        // Top 2x2 block: (dt^2 / 2) * I
        Gamma[0][0] = halfDtSquared;
        Gamma[1][1] = halfDtSquared;

        // Bottom 2x2 block: dt * I
        Gamma[2][0] = dt;
        Gamma[3][1] = dt;

        return Gamma;
    }
}
