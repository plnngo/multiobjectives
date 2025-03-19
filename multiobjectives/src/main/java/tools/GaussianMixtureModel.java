package tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.special.Gamma;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.MerweUnscentedTransform;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.estimation.sequential.ConstantProcessNoise;
import org.orekit.estimation.sequential.CovarianceMatrixProvider;
import org.orekit.estimation.sequential.KalmanEstimation;
import org.orekit.estimation.sequential.UnscentedKalmanEstimator;
import org.orekit.estimation.sequential.UnscentedKalmanEstimatorBuilder;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.propagation.conversion.KeplerianPropagatorBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

import lombok.Getter;

@Getter
public class GaussianMixtureModel {

    double[] weights;

    double[][] means;

    List<double[][]> P;

    public GaussianMixtureModel(double[] weights, double[][] means, List<double[][]> P) {

        // Check dimensions
        int N = weights.length;
        if (N != means.length) {
            throw new IllegalArgumentException("Weights and means do not have the same size");
        } else if (P.size() != N) {
            throw new IllegalArgumentException("P and weights do not have the same size");
        }

        // Initialise global variables
        this.weights = new double[weights.length];
        this.means = new double[means.length][means[0].length];
        this.P = new ArrayList<double[][]>();

        for (int i=0; i<N; i++) {
            this.weights[i] = weights[i];
            
            // Deep copy means
            for (int j=0; j<means[0].length; j++) {
                this.means[i][j] = means[i][j];
            }

            // Deep copy P
            double[][] Pcopy = P.get(i);
            double[][] Pi = new double[Pcopy.length][Pcopy[0].length];
            for (int m=0; m<Pi.length; m++) {
                for (int n=0; n<Pi[0].length; n++) {
                    Pi[m][n] = Pcopy[m][n];
                }
            }
            this.P.add(Pi);
        }
    }

    /**
     * Gaussian Mixture Unscented Kalman Filter.
     * 
     * @param gmmEci        Gaussian Mixture Model in ECI.
     * @see                 Kyle DeMars et al. "Probabilistic Initial Orbit Determination Using 
     *                      Gaussian Mixture Model."
     *        
     */
    public static void gmm_ukf_own(GaussianMixtureModel gmmEci) {

        // Number of GMM components
        int L = gmmEci.getWeights().length;

        // Dimension of state
        int n = gmmEci.getMeans()[0].length;

        // Output 
        double[][] m_list = new double[L][n];
        List<double[][]> P_list = new ArrayList<double[][]>();
        // For each GM component perform 

        // Prediction


        // Update


    }

    /**
     * Gaussian Mixture Unscented Kalman Filter.
     * 
     * @param gmmEci        Gaussian Mixture Model in ECI.
     * @return              Propagatedand updated GMM by a measurement.
     * 
     * @see                 Kyle DeMars et al. "Probabilistic Initial Orbit Determination Using 
     *                      Gaussian Mixture Model."
     */
    public static GaussianMixtureModel gmm_ukf(GaussianMixtureModel gmmEci) {

        // Number of GMM components
        int L = gmmEci.getWeights().length;

        // Dimension of state
        int n = gmmEci.getMeans()[0].length;

        // Output
        double[][] meansUpdated = new double[L][n];
        List<double[][]> PUpdated = new ArrayList<double[][]>();

        // Load measurement
        List<ObservedMeasurement<?>> meas = 
            new ArrayList<ObservedMeasurement<?>>();
        // TODO: set angular values

        // List of measurement likelihood
        double[] beta_list = new double[L];

        // For each GM component perform UKF
        for (int i=0; i<L; i++) {

            // Process noise
            final RealMatrix orbitalQ = 
                MatrixUtils.createRealDiagonalMatrix(new double[] {1.0e-6, 1.0e-6, 1.0e-6, 
                                                                   1.0e-9, 1.0e-9, 1.0e-9});
            // Cartesian initial covariance matrix
            final RealMatrix orbitalP = new Array2DRowRealMatrix(gmmEci.getP().get(i));

            // Build covariance provider
            final CovarianceMatrixProvider provider = new ConstantProcessNoise(orbitalP, orbitalQ);

            // Initialise propagator
            AbsoluteDate epoch = new AbsoluteDate();            // TODO: change!
            double[] pv = gmmEci.getMeans()[i];
            PVCoordinates pvCoord = new PVCoordinates(new Vector3D(pv[0], pv[1], pv[2]), 
                                                      new Vector3D(pv[3], pv[4], pv[5]));
            CartesianOrbit orbit = new CartesianOrbit(pvCoord, 
                                                      FramesFactory.getGCRF(),  // TODO: careful with frame
                                                      epoch, 
                                                      Constants.WGS84_EARTH_MU);
            final KeplerianPropagatorBuilder kepProp = 
                new KeplerianPropagatorBuilder(orbit, null, 1.);

            // Initialize builder
            final UnscentedKalmanEstimatorBuilder builder = new UnscentedKalmanEstimatorBuilder();

            // Add the propagation configuration
            builder.addPropagationConfiguration(kepProp, provider);

            // Unscented transform provider
            double alpha = 1.;
            double pnorm = 2.;
            double kurt = Gamma.gamma(5./pnorm) * Gamma.gamma(1./pnorm) 
                        / FastMath.pow(Gamma.gamma(3./pnorm), 2);
            double beta = kurt - 1.;
            double kappa = kurt - (double)L;
            builder.unscentedTransformProvider(new MerweUnscentedTransform(n, alpha, beta, kappa));

            // Build filter
            final UnscentedKalmanEstimator estimator = builder.build();
            ModelLogger modelLogger = new ModelLogger();
            estimator.setObserver(modelLogger);

            // Estimation
            estimator.processMeasurements(meas);

            // Process estimation output
            KalmanEstimation model = modelLogger.estimation;
            RealVector ybar = 
                new ArrayRealVector(model.getPredictedMeasurement().getEstimatedValue());
            RealMatrix Pyy = model.getPhysicalInnovationCovarianceMatrix();
            double beta_j = getMultivariateGaussianLikelihood(meas, ybar, Pyy);

            // Updated state and covariance
            PVCoordinates meanUpdated = 
                model.getCorrectedSpacecraftStates()[model.getCorrectedSpacecraftStates().length-1]
                     .getPVCoordinates();
            meansUpdated[i] = new double[]{meanUpdated.getPosition().getX(), 
                                           meanUpdated.getPosition().getY(),
                                           meanUpdated.getPosition().getZ(),
                                           meanUpdated.getVelocity().getX(),
                                           meanUpdated.getVelocity().getY(),
                                           meanUpdated.getVelocity().getZ()};
            PUpdated.add(model.getPhysicalEstimatedCovarianceMatrix().getData());       // TODO: Check if this is indeed the corrected cov

            // Fill up array of measurement array
            beta_list[i] = beta_j;
        }

        // Compute new weights of updated GMM components
        RealVector alpha_vect = new ArrayRealVector(gmmEci.getWeights());
        RealVector beta_vect = new ArrayRealVector(beta_list);
        double[] updated_alpha_list = new double[L];
        double[] alphaBetaProduct = alpha_vect.ebeMultiply(beta_vect).toArray();
        double denominator = Arrays.stream(alphaBetaProduct).sum();

        for (int i=0; i<L; i++) {
            updated_alpha_list[i] = alpha_vect.getEntry(i) * alpha_vect.getEntry(i) / denominator;
        }

        // Updated GMM by a measurement
        return new GaussianMixtureModel(updated_alpha_list, meansUpdated, PUpdated);
    }
    
    /**
     * 
     * @param meas
     * @param ybar
     * @param pyy
     * @return
     */
    private static double getMultivariateGaussianLikelihood(List<ObservedMeasurement<?>> meas, 
                                                          RealVector ybar, RealMatrix pyy) {
        
        // Get dimension of measurement
        int k = ybar.getDimension();

        // LUP-decomposition of a square matrix
        LUDecomposition lud = new LUDecomposition(pyy);

        // Formate measurement
        double[] m = meas.get(0).getObservedValue();
        RealVector diffVec = new ArrayRealVector(m).subtract(ybar);
        RealMatrix diff = new Array2DRowRealMatrix(diffVec.toArray());
        RealMatrix diffT = new Array2DRowRealMatrix(diffVec.toArray()).transpose();

        // Exponent
        double exp = MatrixUtils.inverse(pyy).preMultiply(diffT).multiply(diff).getEntry(0, 0);

        double mgl = FastMath.pow(2 * FastMath.PI, -0.5*k) 
                        * FastMath.sqrt(lud.getDeterminant())
                        * FastMath.exp(exp);
        return mgl;
    }    
}
