package tools;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;

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
     * This function examines a GMM containing multiple components. It removes
     * components with weights below a given threshold, and merges components that
     * are close together (small NL2 distance).

     * @param gmm0
     * @return
     */
    public static GaussianMixtureModel mergeAndPrune(GaussianMixtureModel gmm0) {

        // Merge and pruning parameters
        final double prune_T = 1e-3;
        final double merge_U = 1.e6;

        // Number of states
        int nstates = gmm0.getMeans()[0].length;

        // Number of GMM components
        int L = gmm0.getWeights().length;

        // Only keep GM components whose weight is above the threshold   
        // This applies DeMars threshold instead of Vo which just uses T
        double wmax = Arrays.stream(gmm0.getWeights()).max().orElse(Double.NaN);
        List<Double> wPruned = new ArrayList<Double>();
        List<double[]> meansPruned = new ArrayList<double[]>();
        List<double[][]> PPruned = new ArrayList<double[][]>();
        for (int ii=0; ii<L; ii++) {
            if (gmm0.getWeights()[ii] > prune_T*wmax) {
                wPruned.add(gmm0.getWeights()[ii]);
                meansPruned.add(gmm0.getMeans()[ii]);
                PPruned.add(gmm0.getP().get(ii));
            }
        }

        // Normalise weights
        double sumW = wPruned.stream().mapToDouble(Double::doubleValue).sum();
        double sumW0 = Arrays.stream(gmm0.getWeights()).sum();
        double[] w = new double[wPruned.size()];
        for (int i=0; i<w.length; i++) {
            w[i] = sumW0 * wPruned.get(i)/sumW;             
        }

        // Output: final GMM
        List<Double> wf = new ArrayList<Double>();
        List<double[]> mf = new ArrayList<double[]>();
        List<double[][]> Pf = new ArrayList<double[][]>();

        // Loop to merge components that are close
        Set<Integer> I = IntStream.rangeClosed(0, w.length)     
                                  .boxed()                    // Convert to Integer (unboxing)
                                  .collect(Collectors.toSet()); 

        while (!I.isEmpty()) {                              
            // Loop over components to see if they are close to j
            // Note, at least one will be when i == j  
            
            double wsum = 0.;
            double[] msum_array = new double[nstates];
            RealVector msum = new ArrayRealVector(msum_array);

            // Find index of maximum in w
            int jj = 0;                             
            for (int i = 1; i < w.length; i++) {
                if (w[i] > w[jj]) {
                    jj = i;
                }
            }

            // merge into new L components
            List<Integer> Lnew = new ArrayList<Integer>();

            for (int ii=0; ii<w.length; ii++) {
                double[][] Pii = PPruned.get(ii);
                RealMatrix invP = MatrixUtils.inverse(new Array2DRowRealMatrix(Pii));
                RealVector mii = new ArrayRealVector(meansPruned.get(ii));
                RealVector diff = mii.subtract(new ArrayRealVector(meansPruned.get(jj)));
                // Compute Mahalanobis distance
                double prod = diff.dotProduct(invP.operate(diff));          
                if (prod <= merge_U) {
                    Lnew.add(ii);
                    wsum += w[ii];
                    msum = msum.add(mii.mapMultiply(w[ii]));
                }
            }

            // Compute final w,m,P
            wf.add(wsum);
            RealVector mf_bar = msum.mapMultiply(1./wsum);
            mf.add(mf_bar.toArray());

            double[][] Psum_array = new double[nstates][nstates];
            RealMatrix Psum = new Array2DRowRealMatrix(Psum_array);
            for (int ii=0; ii<Lnew.size(); ii++) {
                RealMatrix PLii = new Array2DRowRealMatrix(PPruned.get(Lnew.get(ii)));
                RealVector diff = 
                    mf_bar.subtract(new ArrayRealVector(meansPruned.get(Lnew.get(ii))));
                RealMatrix updateP = 
                    PLii.add(diff.outerProduct(diff)).scalarMultiply(w[Lnew.get(ii)]);
                Psum = Psum.add(updateP);
            }
            RealMatrix Pf_bar = Psum.scalarMultiply(1./wsum);
            Pf.add(Pf_bar.getData());

            Set<Integer> setL = new HashSet<>(Lnew);
            I.removeAll(setL);            
        } 

        // Normalise weights
        double[] wf_norm = new double[wf.size()];
        double sumWf = wf.stream().mapToDouble(Double::doubleValue).sum();
        for (int i=0; i<wf_norm.length; i++) {
            wf_norm[i] = sumW0 * wf.get(i)/sumWf;
        }

        // Turn mf into a multidimensional array
        double[][] m_array = new double[mf.size()][];
        for (int i = 0; i < mf.size(); i++) {
            m_array[i] = mf.get(i);
        }

        return new GaussianMixtureModel(wf_norm, m_array, Pf);
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
