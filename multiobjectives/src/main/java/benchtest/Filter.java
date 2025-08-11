package benchtest;

import org.apache.commons.lang3.ArrayUtils;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
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
import lombok.Getter;
import sensortasking.mcts.App;

@Getter
public class Filter {

    double[] stateCorr;

    double[][] covCorr;

    double[] statePred;

    double[][] covPred;

    /** Process noise. */
    double Q = 0.;      //1E-15;

    /** Measurement noise. */
    //double Rk = 0.02;
    double Rk = FastMath.pow(1 * FastMath.PI/(180*3600), 2);     // 1 arcsec

    double epsilon = 1e-7;

    public void run_ckf(double[] X0_ref, double[][] P_pre, double t0, double t_obs, double obs_data) {

        //System.out.println(new ArrayRealVector(X0_ref) + " time: " + t_obs);

        RealMatrix P0 = new Array2DRowRealMatrix(P_pre);
        RealMatrix Rk_mat = new Array2DRowRealMatrix(new double[]{Rk});

        // State dimension
        int n = X0_ref.length;

        // Error state
        double[] xhat_pre = new double[n];

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
        double[] ones_arr = flattenRowMajor(ones.getData());
        double[] Xref_Stm0 = ArrayUtils.addAll(X0_ref, ones_arr);

        Car carA_0 = new Car('f', X0_ref, P_pre, t0);
        ExpandableODE expandable = new ExpandableODE(carA_0);
        RealMatrix Phik;
        double[] Xref = new double[n];
        if(t0 == t_obs) {
            Phik = ones;
            this.statePred = X0_ref.clone();
            Xref = X0_ref.clone();
        } else {
            ODEIntegrator integrator = new ClassicalRungeKuttaIntegrator(0.01);
            ODEState initialState = new ODEState(t0, Xref_Stm0);
            ODEStateAndDerivative finalState = integrator.integrate(expandable, initialState, t_obs);
            if (FastMath.abs(t_obs-finalState.getTime())>1e-2) {
                throw new IllegalArgumentException("Did not propagate the state to the observation" 
                                                    + "epoch");
            }

            // Extract propagated state
            double[] y = finalState.getPrimaryState();
            
            for (int i=0; i<n; i++) {

                // Extract state vector
                double rounded = FastMath.rint(y[i] / epsilon) * epsilon;
                Xref[i] = rounded;
            }
            this.statePred = Xref.clone();

            // Extract phi matrix from X (column-major to 2D array)
            double[][] Phik_arr = new double[4][4];
            for (int col = 0; col < n; col++) {
                for (int row = 0; row < n; row++) {
                    Phik_arr[row][col] = y[n + col * n + row];
                }
            }
            Phik = new Array2DRowRealMatrix(Phik_arr).transpose();
        }

        double[][] gamma = computeGamma(0, t_obs);
        RealMatrix Gamma = new Array2DRowRealMatrix(gamma);

        // Predicted correction 
        RealVector Xref_vec = new ArrayRealVector(Xref);
        double[] xk_bar = Phik.operate(xhat_pre);
        RealMatrix xk_bar_mat = new Array2DRowRealMatrix(xk_bar);

        // Predicted covariance
        RealMatrix mappedUnmodelAcc =  Gamma.scalarMultiply(Q).multiplyTransposed(Gamma);
        RealMatrix Pk_bar = Phik.multiply(P0).multiplyTransposed(Phik).add(mappedUnmodelAcc);
        //App.printCovariance(Pk_bar);
        this.covPred = Pk_bar.getData();

        // Compute system noise mapping matrix
        //MeasurementModel measModel = LinearRangeMeasurementModel.generateHk(Xref); 
        benchtest.LinearBearingMeasurementModel.MeasurementModel measModel = 
            LinearBearingMeasurementModel.generateHk(Xref);
        double innov = obs_data - measModel.Gk;
/*         if (innov < 1e-12) {
            innov = 0.;
        } */
        double[] hk_til = measModel.Hk_til;
/*         System.out.println("Innovation: " + innov);
        System.out.println(obs_data);
        System.out.println(measModel.Gk); */
        RealMatrix Hk_til = new Array2DRowRealMatrix(hk_til).transpose();

        // Kalman gain
        RealMatrix S = Hk_til.multiply(Pk_bar).multiplyTransposed(Hk_til).add(Rk_mat);
        /* System.out.println("S covariance:");
        App.printCovariance(S); */
        RealMatrix Kk = Pk_bar.multiplyTransposed(Hk_til).multiply(MatrixUtils.inverse(S));

        // Correction
        double[] xhat = xk_bar_mat.add(Kk.scalarMultiply(innov - Hk_til.operate(xk_bar)[0]))
                                  .getColumn(0);
        RealVector xhat_vec = new ArrayRealVector(xhat);
        RealVector Xref_out = Xref_vec.add(xhat_vec);
        this.stateCorr = Xref_out.toArray();
        if (FastMath.abs(Xref_out.getEntry(3))>0.00001) {
            throw new IllegalArgumentException("Object is moving with non-zero velocity along "
                                                    + "Y axis");
        }

        // Joseph-form covariance update 
        RealMatrix kalmanCorr = ones.subtract(Kk.multiply(Hk_til));
        //App.printCovariance(kalmanCorr);
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
