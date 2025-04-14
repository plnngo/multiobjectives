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
import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixDecomposer;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.QRDecomposer;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.special.Gamma;
import org.hipparchus.util.FastMath;
import org.hipparchus.util.MerweUnscentedTransform;
import org.hipparchus.util.UnscentedTransformProvider;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.estimation.sequential.ConstantProcessNoise;
import org.orekit.estimation.sequential.CovarianceMatrixProvider;
import org.orekit.estimation.sequential.KalmanEstimation;
import org.orekit.estimation.sequential.MeasurementDecorator;
import org.orekit.estimation.sequential.UnscentedKalmanEstimator;
import org.orekit.estimation.sequential.UnscentedKalmanEstimatorBuilder;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.PropagatorsParallelizer;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.conversion.KeplerianPropagatorBuilder;
import org.orekit.propagation.conversion.PropagatorBuilder;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

import lombok.Getter;
import sensortasking.mcts.App;

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
        final double merge_U = 1.e9;

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
        Set<Integer> I = IntStream.range(0, w.length)     
                                  .boxed()                    // Convert to Integer (unboxing)
                                  .collect(Collectors.toSet()); 

        while (!I.isEmpty()) {                              
            // Loop over components to see if they are close to j
            // Note, at least one will be when i == j  
            System.out.println(I.size());
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

        double sum = 0.;
        for (int i=0; i<wf_norm.length;i++) {
            sum = sum + wf_norm[i];
        }

        return new GaussianMixtureModel(wf_norm, m_array, Pf);
    }

    /**
     * Gaussian Mixture Unscented Kalman Filter with orekit
     * 
     * @param gmmTopoInertial        Gaussian Mixture Model in topocentric inertial frame.
     * @return              Propagatedand updated GMM by a measurement.
     * 
     * @see                 Kyle DeMars et al. "Probabilistic Initial Orbit Determination Using 
     *                      Gaussian Mixture Model."
     */
    public static GaussianMixtureModel gmm_ukf(GaussianMixtureModel gmmTopoInertial,
                                               Frame topoInertial, 
                                               List<ObservedMeasurement<?>> meas) {


        double sum = 0.;
        for (int i=0; i<gmmTopoInertial.getWeights().length;i++) {
            sum = sum + gmmTopoInertial.getWeights()[i];
        }

        // Number of GMM components
        int L = gmmTopoInertial.getWeights().length;

        // Dimension of state
        int n = gmmTopoInertial.getMeans()[0].length;

        // Output
        double[][] meansUpdated = new double[L][n];
        List<double[][]> PUpdated = new ArrayList<double[][]>();

        // List of measurement likelihood
        double[] beta_list = new double[L];

        // For each GM component perform UKF
        for (int i=0; i<L; i++) {

            // Process noise
            final RealMatrix orbitalQ = 
                MatrixUtils.createRealDiagonalMatrix(new double[] {1.0e-6, 1.0e-6, 1.0e-6, 
                                                                   1.0e-9, 1.0e-9, 1.0e-9});
            // Cartesian initial covariance matrix
            final RealMatrix orbitalP = new Array2DRowRealMatrix(gmmTopoInertial.getP().get(i));
            App.printCovariance(orbitalP);

            // Build covariance provider
            final CovarianceMatrixProvider provider = new ConstantProcessNoise(orbitalP, orbitalQ);

            // Initialise propagator
            AbsoluteDate epoch = meas.get(0).getDate();            
            double[] pv = gmmTopoInertial.getMeans()[i];
            PVCoordinates pvCoord = new PVCoordinates(new Vector3D(pv[0], pv[1], pv[2]), 
                                                      new Vector3D(pv[3], pv[4], pv[5]));
            System.out.println(pvCoord.getPosition().getAlpha());
            System.out.println(pvCoord.getPosition().getDelta());
            CartesianOrbit orbit = new CartesianOrbit(pvCoord, 
                                                      topoInertial,  // TODO: careful with frame
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
            double kappa = kurt - pv.length;
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
            System.out.println(FastMath.toDegrees(ybar.getEntry(0)) + 360);
            System.out.println(FastMath.toDegrees(ybar.getEntry(1)));
            RealMatrix Pyy = model.getPhysicalInnovationCovarianceMatrix();
            App.printCovariance(Pyy);
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
            App.printCovariance(model.getPhysicalEstimatedCovarianceMatrix());

            // Fill up array of measurement array
            beta_list[i] = beta_j;
        }

        // Compute new weights of updated GMM components
        RealVector alpha_vect = new ArrayRealVector(gmmTopoInertial.getWeights());
        RealVector beta_vect = new ArrayRealVector(beta_list);
        double[] updated_alpha_list = new double[L];
        double[] alphaBetaProduct = alpha_vect.ebeMultiply(beta_vect).toArray();
        double denominator = Arrays.stream(alphaBetaProduct).sum();

        double sumWeights = 0.;
        for (int i=0; i<L; i++) {
            updated_alpha_list[i] = alpha_vect.getEntry(i) * alpha_vect.getEntry(i) / denominator;
            sumWeights = sumWeights + updated_alpha_list[i];
        }

        // Updated GMM by a measurement
        return new GaussianMixtureModel(updated_alpha_list, meansUpdated, PUpdated);
    }

    public static GaussianMixtureModel gmm_ukf_own(GaussianMixtureModel gmm,
                                                   Frame frame, 
                                                   List<ObservedMeasurement<?>> meas) {
        
        // Number of GMM components
        int L = gmm.getWeights().length;

        // Dimension of state
        int n = gmm.getMeans()[0].length;

        // Output
        double[][] meansUpdated = new double[L][n];
        List<double[][]> PUpdated = new ArrayList<double[][]>();

        // List of measurement likelihood
        double[] beta_list = new double[L];

        // For each GM component perform UKF
        for (int i=0; i<L; i++) {

            // Process noise
            final RealMatrix orbitalQ = 
                MatrixUtils.createRealDiagonalMatrix(new double[] {1.0e-6, 1.0e-6, 1.0e-6, 
                                                                   1.0e-9, 1.0e-9, 1.0e-9});
            // Cartesian initial covariance matrix
            final RealMatrix orbitalP = new Array2DRowRealMatrix(gmm.getP().get(i));
            App.printCovariance(orbitalP);

            // Build covariance provider
            //final CovarianceMatrixProvider provider = new ConstantProcessNoise(orbitalP, orbitalQ);

            // Initialise propagator
            AbsoluteDate epoch = meas.get(0).getDate();            
            double[] pv = gmm.getMeans()[i];
            PVCoordinates pvCoord = new PVCoordinates(new Vector3D(pv[0], pv[1], pv[2]), 
                                                      new Vector3D(pv[3], pv[4], pv[5]));

            CartesianOrbit orbit = new CartesianOrbit(pvCoord, 
                                                      frame,  // TODO: careful with frame
                                                      epoch, 
                                                      Constants.WGS84_EARTH_MU);
            final KeplerianPropagatorBuilder kepProp = 
                new KeplerianPropagatorBuilder(orbit, null, 1.);

            // Unscented transform provider
            double alpha = 1.;
            double pnorm = 2.;
            double kurt = Gamma.gamma(5./pnorm) * Gamma.gamma(1./pnorm) 
                        / FastMath.pow(Gamma.gamma(3./pnorm), 2);
            double beta = kurt - 1.;
            double kappa = kurt - pv.length;
            //double kappa = 0.;
            UnscentedTransformProvider prov = new MerweUnscentedTransform(n, alpha, beta, kappa);

            // Generate the current sigma points
            RealVector[] sigmaPoints = prov.unscentedTransform(new ArrayRealVector(pv), orbitalP);

            // Initialize arrays of predicted states and measurements
            final RealVector[] predictedStates = new RealVector[sigmaPoints.length];

            // Mean weights
            final RealVector wm = prov.getWm();

            // Check if weights are normalised
            double sumWeights = 0.;
            double[] weights = wm.toArray();
/*             for (int k=0; k<weights.length; k++) {
                sumWeights = sumWeights + weights[k];
            }
            if (FastMath.abs(sumWeights - 1.) > 0.1) {
                throw new Error("Weights in GmmECI are not normalised");
            }
 */
            // Propagate sigma points and get predicted states
            List<SpacecraftState> states = predictStates(sigmaPoints, kepProp, meas.get(0));

            // Loop on states
            for (int j=0; j<states.size(); j++) {
                predictedStates[j] = new ArrayRealVector(sigmaPoints[j].getDimension());

                // Current predicted state
                final SpacecraftState predicted = states.get(j);
                Vector3D posPred = 
                    predicted.getPVCoordinates(FramesFactory.getEME2000()).getPosition();
                
                Vector3D velPred = 
                    predicted.getPVCoordinates(FramesFactory.getEME2000()).getVelocity();
                double[] pvPred = new double[]{posPred.getX(), posPred.getY(), posPred.getZ(),
                                               velPred.getX(), velPred.getY(), velPred.getZ()};


                // First, convert the predicted state to an array
                //final double[] predictedArray = new double[sigmaPoints[j].getDimension()];
                //OrbitType.CARTESIAN.mapOrbitToArray(predicted.getOrbit(), null, predictedArray, null);
                predictedStates[j] = new ArrayRealVector(pvPred);
                //predictedStates[j].setSubVector(0, new ArrayRealVector(predictedArray));
            }
            // Initialize the weighted mean parameter
            RealVector predictedState = new ArrayRealVector(n);

            // Compute weighted mean parameter
            for (int j = 0; j<wm.getDimension(); j++) {
                predictedState = predictedState.add(predictedStates[j].mapMultiply(wm.getEntry(j)));
            }

            RealMatrix covarianceMatrix = MatrixUtils.createRealMatrix(n, n);

            // Covariance weights
            final RealVector wc = prov.getWc();

            // Compute the predicted covariance matrix
            for (int j = 0; j<=2*n; j++) {
                final RealVector diff = predictedStates[j].subtract(predictedState);
                covarianceMatrix = covarianceMatrix.add(outer(diff, diff).scalarMultiply(wc.getEntry(j)));
            }
            RealMatrix predictedCovariance = covarianceMatrix.add(orbitalQ);

            // Generate sigma points derived from predicted state
            //RealVector[] predictedSigmaPoints = prov.unscentedTransform(predictedState, predictedCovariance);

            // Predicted measurements 
            final RealVector[] predictedMeasurements = getPredictedMeasurement(predictedStates);

            // Compute weighted predicted measurement
            RealVector predictedMeasurement = new ArrayRealVector(predictedMeasurements[0].getDimension());

            for (int j = 0; j<wm.getDimension(); j++) {
                predictedMeasurement = 
                    predictedMeasurement.add(predictedMeasurements[j].mapMultiply(wm.getEntry(j)));
            }

            // Computation of the innovation covariance matrix
            RealMatrix innovMatrix = 
                MatrixUtils.createRealMatrix(predictedMeasurement.getDimension(), 
                                             predictedMeasurement.getDimension());

            for (int j = 0; j<=2*n; j++) {      // n=6
                final RealVector diff = predictedMeasurements[j].subtract(predictedMeasurement);
                innovMatrix = innovMatrix.add(outer(diff, diff).scalarMultiply(wc.getEntry(j)));
            }
            // Add the measurement covariance
            double[] rVec = meas.get(0).getTheoreticalStandardDeviation();
            RealMatrix r = new DiagonalMatrix(rVec);
            RealMatrix Pyy = innovMatrix.add(r);

            // Compute cross correlation matrix
            RealMatrix   Pxy = computeCrossCovarianceMatrix(predictedStates, predictedState,
                                                            predictedMeasurements, 
                                                            predictedMeasurement,
                                                            prov.getWc());

            // Compute innovation
            RealVector observed = new ArrayRealVector(meas.get(0).getObservedValue());
            RealVector innov = observed.subtract(predictedMeasurement);

            // Compute Kalman gain 
            MatrixDecomposer decomposer = new QRDecomposer(1.0e-15);
            final RealMatrix k = decomposer.decompose(Pyy).solve(Pxy.transpose()).transpose();

            // correct state vector
            final RealVector correctedState = predictedState.add(k.operate(innov));

            // correct covariance matrix
            final RealMatrix correctedCovariance = 
                predictedCovariance.subtract(k.multiply(Pyy).multiplyTransposed(k));
            double beta_j = getMultivariateGaussianLikelihood(meas, predictedMeasurement, Pyy);
            
            // Fill up array of measurement array
            beta_list[i] = beta_j;

        }

        // Compute new weights of updated GMM components
        RealVector alpha_vect = new ArrayRealVector(gmm.getWeights());
        RealVector beta_vect = new ArrayRealVector(beta_list);
        double[] updated_alpha_list = new double[L];
        double[] alphaBetaProduct = alpha_vect.ebeMultiply(beta_vect).toArray();
        double denominator = Arrays.stream(alphaBetaProduct).sum();

        double sumWeights = 0.;
        for (int i=0; i<L; i++) {
            updated_alpha_list[i] = alpha_vect.getEntry(i) * alpha_vect.getEntry(i) / denominator;
            sumWeights = sumWeights + updated_alpha_list[i];
        }

        // Updated GMM by a measurement
        return new GaussianMixtureModel(updated_alpha_list, meansUpdated, PUpdated);
    }

    /**
     * Computes cross covariance matrix.
     * @param predictedSigmaPoints
     * @param predictedState
     * @param predictedMeasurements
     * @param predictedMeasurement
     * @param wc
     * @return
     */
    private static RealMatrix computeCrossCovarianceMatrix(RealVector[] predictedSigmaPoints, 
                                                           RealVector predictedState,
                                                           RealVector[] predictedMeasurements, 
                                                           RealVector predictedMeasurement,
                                                           RealVector wc) {
        // Initialize the cross covariance matrix
        RealMatrix crossCovMatrix = 
            MatrixUtils.createRealMatrix(predictedState.getDimension(),
                                         predictedMeasurement.getDimension());
        // Compute the cross covariance matrix
        for (int i=0; i<=2*predictedState.getDimension(); i++) {
            final RealVector stateDiff = predictedSigmaPoints[i].subtract(predictedState);
            final RealVector measDiff  = predictedMeasurements[i].subtract(predictedMeasurement);
            crossCovMatrix = 
                crossCovMatrix.add(outer(stateDiff, measDiff).scalarMultiply(wc.getEntry(i)));
        }
        return crossCovMatrix;
    }

    /**
     * Predict the predicted states for the given sigma points.
     * @param sigmaPoints current sigma points
     * @param index the index corresponding to the satellite one is dealing with
     * @return predicted state for the given sigma point
     */
    private static List<SpacecraftState> predictStates(final RealVector[] sigmaPoints, 
                                                final PropagatorBuilder propBuilder, 
                                                final ObservedMeasurement meas) {

        // Loop on sigma points to create the propagator parallelizer
        final List<Propagator> propagators = new ArrayList<>(sigmaPoints.length);
        for (int k = 0; k < sigmaPoints.length; ++k) {
            // Current sigma point
            final double[] currentPoint = sigmaPoints[k].copy().toArray();
            // Create the corresponding orbit propagator
            final Propagator currentPropagator = createPropagator(currentPoint, propBuilder);
            // Add it to the list of propagators
            propagators.add(currentPropagator);
        }

        // Create the propagator parallelizer and predict states
        // (the shift is done to start a little bit before the previous measurement epoch)
        final PropagatorsParallelizer parallelizer = new PropagatorsParallelizer(propagators, interpolators -> { });
        final AbsoluteDate previousDate = propBuilder.getInitialOrbitDate();
        final List<SpacecraftState> states = parallelizer.propagate(previousDate.shiftedBy(-1.0e-3), meas.getDate());

        // Return
        return states;
    }

    private static RealVector[] getPredictedMeasurement(final RealVector[] predictedSigmaPoints) {

        // Initialize arrays of predicted states and measurements
        final RealVector[] predictedMeasurements = new RealVector[predictedSigmaPoints.length];

        // J2000 reference frame
        for (int i=0; i<predictedSigmaPoints.length; i++) {
            double[] pv = predictedSigmaPoints[i].toArray();
            Vector3D sigmaPoint =  new Vector3D(pv[0], pv[1], pv[2]);
            double[] angles = new double[]{sigmaPoint.getAlpha(), sigmaPoint.getDelta()};
            predictedMeasurements[i] = new ArrayRealVector(angles);
        }
        return predictedMeasurements;
    }

        /**
     * Create a propagator for the given sigma point.
     * @param point input sigma point
     * @param index the index corresponding to the satellite one is dealing with
     * @return the corresponding orbit propagator
     */
    private static Propagator createPropagator(final double[] point, final PropagatorBuilder propBuilder) {
        // Create a new instance of the current propagator builder
        final PropagatorBuilder copy = propBuilder.copy();
        // Convert the given sigma point to an orbit
        final Orbit orbit = OrbitType.CARTESIAN.mapArrayToOrbit(point, null, null, copy.getInitialOrbitDate(),
                                                      copy.getMu(), copy.getFrame());
        copy.resetOrbit(orbit);
        // Create the propagator
        final Propagator propagator = copy.buildPropagator(copy.getSelectedNormalizedParameters());
        return propagator;
    }


    /** Computes the outer product of two vectors.
     * @param a first vector
     * @param b second vector
     * @return the outer product of a and b
     */
    private static RealMatrix outer(final RealVector a, final RealVector b) {

        // Initialize matrix
        final RealMatrix outMatrix = MatrixUtils.createRealMatrix(a.getDimension(), b.getDimension());

        // Fill matrix
        for (int row = 0; row < outMatrix.getRowDimension(); row++) {
            for (int col = 0; col < outMatrix.getColumnDimension(); col++) {
                outMatrix.setEntry(row, col, a.getEntry(row) * b.getEntry(col));
            }
        }

        // Return
        return outMatrix;

    }


    
    /**
     * 
     * @param meas
     * @param ybar
     * @param pyy
     * @return
     */
    public static double getMultivariateGaussianLikelihood(List<ObservedMeasurement<?>> meas, 
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
        double exp = - 0.5 * (MatrixUtils.inverse(pyy).preMultiply(diffT).multiply(diff).getEntry(0, 0));

        System.out.println(FastMath.pow(2 * FastMath.PI, -0.5*k));
        System.out.println(FastMath.sqrt(lud.getDeterminant()));
        System.out.println(FastMath.exp(exp));

        double mgl = FastMath.pow(2 * FastMath.PI, -0.5*k) 
                        * 1./FastMath.sqrt(lud.getDeterminant())
                        * FastMath.exp(exp);
        System.out.println(mgl);
        return mgl;
    }    
}
