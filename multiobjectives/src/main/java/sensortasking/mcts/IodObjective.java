package sensortasking.mcts;

import java.io.FileWriter;
import java.io.IOException;
import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.DoubleStream;

import org.hipparchus.analysis.solvers.LaguerreSolver;
import org.hipparchus.complex.Complex;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.ArrayRealVector;
import org.hipparchus.linear.CholeskyDecomposition;
import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.linear.RealVector;
import org.hipparchus.special.Gamma;
import org.hipparchus.stat.regression.SimpleRegression;
import org.hipparchus.util.FastMath;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.PVCoordinates;

import tools.GaussianMixtureModel;
import tools.MatrixTools;


@SuppressWarnings("rawtypes")
public class IodObjective implements Objective{

    /** Tracklet consisting of angles [0], [1] in [rad] and their respective derivatives [2], [3] 
     * in [rad/sec]*/
    double[] tracklet;

    /** Sensor. */
    Sensor sensor;

    /** Upper constraint on semi-major axis in [m]. */
    double a_max;

    /** Lower constraint on semi-major axis in [m]. */
    double a_min;

    /** Upper constraint on eccentricity. */
    double e_max;

    /** Lower constraint on eccentricity. */
    double e_min = 0.;

    /** Earth gravitational constant. */
    double mu = 398600441500000.0;

    /** Measurement noise. */
    double[] measNoise;

    /** Earth radius in m. */
    final double Re = 6378.1370*1000.;            // m

    /** Inertial position of sensor. */
    Vector3D q;
    /* Vector3D q = new Vector3D(FastMath.cos(FastMath.toRadians(30.)), 
                                FastMath.toRadians(0.), 
                                FastMath.sin(FastMath.toRadians(30.)))
                                .scalarMultiply(Re); */
    
    /** Earth angular velocity. */
    //Vector3D omega = new Vector3D(0., 0., 7.2921158553e-5);
    Vector3D omega = new Vector3D(0., 0., Constants.WGS84_EARTH_ANGULAR_VELOCITY);

    /** Inertial velocity of sensor. */
    Vector3D dq;

    /** Look up table of solution of standard deviation. Reference to DeMars Table 1.*/
    final double[] lookUp = new double[]{Double.NaN, 
                                        0.3467, 
                                        0.2903, 
                                        0.2466, 
                                        0.2001, 
                                        0.1531, 
                                        0.1225, 
                                        0.1026, 
                                        0.0884, 
                                        0.0778, 
                                        0.0696, 
                                        0.0629, 
                                        0.0575, 
                                        0.0529, 
                                        0.0490, 
                                        0.0456};


    public IodObjective(double[] tracklet, Sensor sensor, double a_max, double a_min, 
                        double e_max, double[] meas_noise, AbsoluteDate date){
        this.tracklet = tracklet;
        this.measNoise = meas_noise;
        this.sensor = sensor;
        this.q = Sensor.getSensorPosEci(date, sensor.getPosition());
        this.dq = omega.crossProduct(q);

        // Constraining AR by semi-major axis and eccentricity
        this.a_max = a_max;
        this.a_min = a_min;
        this.e_max = e_max;
    }

    /**
     * This function computes the Constrained Admissible Region (CAR) produced by a 4D optical  
     * measurement set, containing angles and angle-rates, in particular topocentric right  
     * ascension and declination. The method is based on DeMars and Jah (2013).
     */
    public Map<Double, double[]> car_drho_limits(AbsoluteDate epoch, double[] rho_vect, boolean print) {

        // Extract angles and derivatives from tracklet
        double ra = tracklet[0];
        double dec = tracklet[1];
        double dra = tracklet[2];
        double ddec = tracklet[3];

        // Unit vectors (DeMars between Eq 1-2)
        Vector3D u_rho = new Vector3D(FastMath.cos(ra) * FastMath.cos(dec), 
                                      FastMath.sin(ra) * FastMath.cos(dec), 
                                      FastMath.sin(dec));
        Vector3D u_ra = new Vector3D(-FastMath.sin(ra) * FastMath.cos(dec),
                                     FastMath.cos(ra) * FastMath.cos(dec),
                                     0.);
        Vector3D u_dec = new Vector3D(-FastMath.cos(ra) * FastMath.sin(dec), 
                                      -FastMath.sin(ra) * FastMath.sin(dec),
                                      FastMath.cos(dec));
        // Semi-Major Axis Constraint
        // Compute coefficients (DeMars Eq 2 setup)
        double w0 = q.dotProduct(q);
        double w1 = 2. * (dq.dotProduct(u_rho));
        double w2 = FastMath.pow(dra, 2) * FastMath.pow(FastMath.cos(dec), 2) 
                        + FastMath.pow(ddec, 2);
        double w3 = 2. * dra * (dq.dotProduct(u_ra)) + 2. * ddec * dq.dotProduct(u_dec);
        double w4 = dq.dotProduct(dq);
        double w5 = 2. * (q.dotProduct(u_rho));

        // Compute energy limits (DeMars Eq 5)
        double E_max = -mu/(2. * a_max);
        if (a_min == 0.) {
            a_min = 1E-10;
        }
        double E_min = -mu /(2. * a_min);

        // Eccentricity Constraint
        // Angular Momentum Components (DeMars Eq 6 setup)
        Vector3D h1 = q.crossProduct(u_rho);
        Vector3D h2 = u_rho.crossProduct(u_ra.scalarMultiply(dra).add(u_dec.scalarMultiply(ddec)));
        Vector3D h3 = u_rho.crossProduct(dq)
                        .add(q.crossProduct(u_ra.scalarMultiply(dra)
                                                .add(u_dec.scalarMultiply(ddec))));
        Vector3D h4 = q.crossProduct(dq);

        // Compute coefficients
        double c0 = h1.dotProduct(h1);
        double c1 = 2. * h1.dotProduct(h2);
        double c2 = 2. * h1.dotProduct(h3);
        double c3 = 2. * h1.dotProduct(h4);
        double c4 = h2.dotProduct(h2);
        double c5 = 2. * h2.dotProduct(h3);
        double c6 = 2. * h2.dotProduct(h4) + h3.dotProduct(h3);
        double c7 = 2. * h3.dotProduct(h4);
        double c8 = h4.dotProduct(h4);

        // Loop over range values
        List<Double> rho_output = new ArrayList<Double>();
        List<Double> drho_output = new ArrayList<Double>();
        List<Double> rho_a_all = new ArrayList<Double>();
        List<Double> rho_e_all = new ArrayList<Double>();
        List<Double> drho_a_all = new ArrayList<Double>();
        List<Double> drho_e_all = new ArrayList<Double>();
        Map<Double, double[]> drho_dict = new LinkedHashMap<Double, double[]>();

        for (int ii= 0; ii<rho_vect.length; ii++) {

            // Current range value
            double rho = rho_vect[ii];

            // Compute F for current range (DeMars Eq 3 setup)
            double F = w2 * FastMath.pow(rho, 2) + w3 * rho + w4 
                        - 2*mu/FastMath.sqrt(FastMath.pow(rho, 2) + w5*rho + w0);

            // Compute values of drho for SMA limits (DeMars Eq 4)
            // Max/Min values of the radical in DeMars Eq 4
            double rad_max = FastMath.pow(w1/2., 2) - F + 2.*E_max;
            double rad_min = FastMath.pow(w1/2., 2) - F + 2.*E_min;

            List<Double> drho_a = new ArrayList<Double>();
            if (rad_max >= 0.) {
                rad_max = FastMath.sqrt(rad_max);
                double drho1 = -(w1/2) + rad_max;
                double drho2 = -(w1/2) - rad_max;

                drho_a.add(drho1);
                drho_a.add(drho2);
            }
            if (rad_min >= 0.) {
                rad_min = FastMath.sqrt(rad_min);
                double drho1 = -(w1/2) + rad_min;
                double drho2 = -(w1/2) - rad_min;

                drho_a.add(drho1);
                drho_a.add(drho2);
            }

            // Eccentricity Constraints
            // Compute P and U for current range (DeMars Eq 6)
            double P = c1*rho*rho + c2*rho + c3;
            double U = 
                c4*FastMath.pow(rho, 4) + c5*FastMath.pow(rho, 3) + c6*rho*rho + c7*rho + c8;

            // Compute coefficients (DeMars Eq 8)
            double a0_max = F * U + mu*mu*(1 - e_max*e_max);
            double a0_min = F * U + mu*mu*(1 - e_min*e_min);
            double a1 = F * P + w1 * U;
            double a2 = U + c0 * F + w1 * P;
            double a3 = P + c0 * w1;
            double a4 = c0;

            double[] coef = new double[]{a0_max, a1, a2, a3, a4};

            // Initial guess as the median drho_a for solver
            double guess = 0.;
            if (drho_a.size() != 0) {
                guess = (drho_a.get(0) + drho_a.get(1)) / 2.;
            }

            // Solve quadric function over drho to constrain AR by eccentricity (DeMars Eq 8)
            LaguerreSolver solver = new LaguerreSolver();
            Complex[] r = solver.solveAllComplex(coef, guess);
            List<Double> drho_ecc = new ArrayList<Double>();

            for (Complex sol : r) {
                if(sol.getImaginaryPart()==0.) {
                    drho_ecc.add(sol.getRealPart());
                }
            }

            // Set up output
            // Build arrays of rho values corresponding to limits in SMA and ECC
            for (Double d : drho_a) {
                drho_a_all.add(Double.valueOf(d));
            }
            for (Double d : drho_ecc) {
                drho_e_all.add(Double.valueOf(d));
            }
            for (int i=0; i<drho_a.size(); i++) {
                rho_a_all.add(Double.valueOf(rho));
            }
            for (int i=0; i<drho_ecc.size(); i++) {
                rho_e_all.add(Double.valueOf(rho));
            }

            // If the eccentricity and semi-major axis limits have returned values
            // for drho, determine which form the boundaries of the CAR
            if (!drho_a.isEmpty() && !drho_ecc.isEmpty()) {

                if (drho_ecc.size() == 2) {

                    if (drho_a.size() == 2) {
                        rho_output.add(Double.valueOf(rho));
                        rho_output.add(Double.valueOf(rho));
                        double[] drho_vect = new double[drho_a.size() + drho_ecc.size()];
                        for (int i=0; i<drho_a.size(); i++) {
                            drho_vect[i] = drho_a.get(i);
                            drho_vect[i+drho_a.size()] = drho_ecc.get(i);
                        }
                        Arrays.sort(drho_vect);
                        drho_output.add(drho_vect[1]);
                        drho_output.add(drho_vect[2]);
                        drho_dict.put(rho, new double[]{drho_vect[1], drho_vect[2]});

                    } else if (drho_a.size() == 4) {
                        Collections.sort(drho_a);
                        Collections.sort(drho_ecc);

                        // Positive side
                        double[] drho_vect1 = new double[2];
                        double max_drho_ecc = drho_ecc.get(drho_ecc.size() - 1);
                        if (drho_a.get(2) < max_drho_ecc) {
                            rho_output.add(Double.valueOf(rho));
                            rho_output.add(Double.valueOf(rho));

                            if (drho_a.get(3) < max_drho_ecc) {
                                for (int i=0; i<drho_vect1.length; i++) {
                                    drho_vect1[i] = drho_a.get(i+2);
                                    drho_output.add(drho_a.get(i+2));
                                }
                            } else {
                                drho_vect1[0] = drho_a.get(2);
                                drho_vect1[1] = max_drho_ecc;
                                drho_output.add(drho_vect1[0]);
                                drho_output.add(drho_vect1[1]);
                            }
                        }
                        // Negative side
                        double[] drho_vect2 = new double[2];
                        if (drho_a.get(1) > drho_ecc.get(0)) {
                            rho_output.add(Double.valueOf(rho));
                            rho_output.add(Double.valueOf(rho));

                            if (drho_a.get(0) > drho_ecc.get(0)) {
                                for (int i=0; i<drho_vect2.length; i++) {
                                    drho_vect2[i] = drho_a.get(i);
                                    drho_output.add(drho_vect2[i]);
                                }
                            } else {
                                drho_vect2[0] = drho_a.get(1);
                                drho_vect2[1] = drho_ecc.get(0);
                                drho_output.add(drho_vect2[0]);
                                drho_output.add(drho_vect2[1]);
                            }
                        }
                        double[] drho_vects = 
                            Arrays.copyOf(drho_vect1, drho_vect1.length + drho_vect2.length);
                        System.arraycopy(drho_vect2, 0, drho_vects, drho_vect1.length, drho_vect2.length);
                        drho_dict.put(rho, drho_vects);
                    }

                } else if (drho_ecc.size() == 4) {

                    if (drho_a.size() == 2) {
                        double[] drho_vect = new double[drho_a.size() + drho_ecc.size()];
                        for (int i=0; i<4; i++) {
                            rho_output.add(Double.valueOf(rho));
                            drho_vect[i+2] = drho_ecc.get(i);
                        }
                        drho_vect[0] = drho_a.get(0);
                        drho_vect[1] = drho_a.get(1);
                        Arrays.sort(drho_vect);
                        double[] drho_vect_1_5 = new double[4];
                        for (int i=1; i<5; i++) {
                            drho_output.add(drho_vect[i]);
                            drho_vect_1_5[i-1] = drho_vect[i];
                        }
                        drho_dict.put(rho, drho_vect_1_5);
                    }

                    if (drho_a.size() == 4) {
                        Collections.sort(drho_a);
                        Collections.sort(drho_ecc);

                        // Positive side
                        double[] drho_vect1 = new double[2];
                        double max_drho_ecc = drho_ecc.get(drho_ecc.size() - 1);
                        if (drho_a.get(2) < max_drho_ecc) {
                            rho_output.add(Double.valueOf(rho));
                            rho_output.add(Double.valueOf(rho));
                        }

                        if (drho_a.get(3) < max_drho_ecc) {
                            for (int i=0; i<drho_vect1.length; i++) {
                                drho_vect1[i] = drho_a.get(i+2);
                                drho_output.add(drho_vect1[i]);
                            }
                        } else {
                            drho_vect1[0] = drho_a.get(2);
                            drho_vect1[1] = max_drho_ecc;
                        }

                        // Negative side
                        double[] drho_vect2 = new double[2];
                        if (drho_a.get(1) > drho_ecc.get(0)) {
                            rho_output.add(Double.valueOf(rho));
                            rho_output.add(Double.valueOf(rho));
                        }

                        if (drho_a.get(0) > drho_ecc.get(0)) {
                            for (int i=0; i<drho_vect2.length; i++) {
                                drho_vect2[i] = drho_a.get(i);
                                drho_output.add(drho_vect2[i]);
                            }
                        } else {
                            drho_vect2[0] = drho_a.get(1);
                            drho_vect2[1] = drho_ecc.get(0);
                            drho_output.add(drho_vect2[0]);
                            drho_output.add(drho_vect2[1]);
                        }
                        double[] drho_vects = 
                            Arrays.copyOf(drho_vect1, drho_vect1.length + drho_vect2.length);
                        System.arraycopy(drho_vect2, 0, drho_vects, drho_vect1.length, drho_vect2.length);
                        drho_dict.put(rho, drho_vects);
                    }
                }
            }
        }
        if (print) {
            write_arhoAall_drhoAall_rhoEall_drhoEall("arhoAall_drhoAall_rhoEall_drhoEall.csv", 
                                                     rho_a_all, drho_a_all, rho_e_all, drho_e_all); 
            write_rhoOutput_drhoOutput("rho_output_drho_output.csv", rho_output, drho_output);
        }

        return drho_dict;
    }
                
    private void write_rhoOutput_drhoOutput(String filename, List<Double> rho_output, List<Double> drho_output) {
        try (FileWriter writer = new FileWriter(filename)) {
            // Write header
            writer.append("rho_output,drho_output\n");
            
            // Determine max length for iteration
            int maxLength = Math.max(rho_output.size(), drho_output.size());
            
            // Write data row by row
            for (int i = 0; i < maxLength; i++) {
                writer.append(i < rho_output.size() ? String.valueOf(rho_output.get(i)) : "");
                writer.append(",");
                writer.append(i < drho_output.size() ? String.valueOf(drho_output.get(i)) : "");
                writer.append("\n");
            }
            
            System.out.println("CSV file saved successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
            
    private void write_arhoAall_drhoAall_rhoEall_drhoEall(String filename, List<Double> rho_a_all,
        List<Double> drho_a_all, List<Double> rho_e_all, List<Double> drho_e_all) {
        try (FileWriter writer = new FileWriter(filename)) {
            // Write header
            writer.append("rho_a_all,drho_a_all,rho_e_all,drho_e_all\n");
            
            // Determine max length for iteration
            int maxLength = Math.max(rho_a_all.size(), Math.max(drho_a_all.size(), Math.max(rho_e_all.size(), drho_e_all.size())));
            
            // Write data row by row
            for (int i = 0; i < maxLength; i++) {
                writer.append(i < rho_a_all.size() ? String.valueOf(rho_a_all.get(i)) : "");
                writer.append(",");
                writer.append(i < drho_a_all.size() ? String.valueOf(drho_a_all.get(i)) : "");
                writer.append(",");
                writer.append(i < rho_e_all.size() ? String.valueOf(rho_e_all.get(i)) : "");
                writer.append(",");
                writer.append(i < drho_e_all.size() ? String.valueOf(drho_e_all.get(i)) : "");
                writer.append("\n");
            }
            
            System.out.println("CSV file saved successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    /**
     * This function computes a Gaussian Mixture Model (GMM) to approximate a 
     * uniform distribution representing the Constrained Admissible Region (CAR)
     * produced by a 4D optical measurement set, containing angles and 
     * angle-rates, in particular topocentric right ascension and declination.
    
     * The method is based on DeMars and Jah

     * @param epoch
     * @param rho_vect
     */
    public GaussianMixtureModel optical_car_gmm(AbsoluteDate epoch, double[] rho_vect, 
                                                double sigma_rho_desired, 
                                                double sigma_drho_desired){

        // Compute CAR boundary
        Map<Double, double[]> drho_dict = car_drho_limits(epoch, rho_vect, true);
        List<Map.Entry<Double, double[]>> drho_dict_list = new ArrayList<>(drho_dict.entrySet());

        // Compute range marginal PDF quantities
        double a_rho = Collections.min(drho_dict.keySet()).doubleValue();
        double b_rho = Collections.max(drho_dict.keySet()).doubleValue();
        double[] LandSigmaOut = car_sigma_library(a_rho, b_rho, sigma_rho_desired);
        int L_rho  = (int)LandSigmaOut[0];

        // Compute means and covariances for GMM components (DeMars Eq 22)
        double[] m_rho = new double[L_rho];
        for (int i=0; i<L_rho; i++) {
            m_rho[i] = a_rho + (b_rho - a_rho)/(L_rho+1.)*(i+1.);
        }

        double P_rho = LandSigmaOut[1] * LandSigmaOut[1];

        // Compute weights of GMM components (DeMars Eq 23)
        // Evaluate range marginal PDF at each range value
        Object[] rho_uniqueO = drho_dict.keySet().toArray();
        double[] rho_unique = new double[rho_uniqueO.length];
        double[] p_vect = new double[rho_uniqueO.length];
        double delta_rho = 
            ((Double)rho_uniqueO[1]).doubleValue() - ((Double)rho_uniqueO[0]).doubleValue();
        for (int i=0; i<rho_unique.length; i++) {
            rho_unique[i] = ((Double)rho_uniqueO[i]).doubleValue();
            Map.Entry<Double, double[]> drho_dict_entry = drho_dict_list.get(i);
            double[] drho_vect = drho_dict_entry.getValue();
            double a_drho = Arrays.stream(drho_vect).min().orElseThrow(null);
            double b_drho = Arrays.stream(drho_vect).max().orElseThrow(null);               
            p_vect[i] = (b_drho - a_drho)*delta_rho;            // aprox. marginal PDF of rho
        }

        // Normalise marginal PDF using total probability mass of rho_unique
        double[] p_vect_normed = computeMarginalPDF(p_vect, rho_unique);

        // Compute H-matrix
        int M = p_vect_normed.length;
        double[][] Hmatrix = new double[M][L_rho];
        double sigj = FastMath.sqrt(P_rho);
        for (int i=0; i<M; i++) {
            for (int j=0; j<L_rho; j++) {
                double rhoi = rho_unique[i];
                double mj = m_rho[j];
                Hmatrix[i][j] = (1./(FastMath.sqrt(2.*FastMath.PI)*sigj)) 
                            * FastMath.exp(-(rhoi-mj) * (rhoi-mj)/(2*P_rho));       // Gausian PDF
            }
        }
        RealMatrix H = new Array2DRowRealMatrix(Hmatrix);

        // Set up Moore-Penrose Pseudoinverse --> normal equation solution matrix
        RealMatrix LSprojection = 
            MatrixUtils.inverse(H.transpose().multiply(H)).multiplyTransposed(H);

        // Project p_vect to best-fit solutions for the Gaussian weights
        double[] w_rho = LSprojection.operate(p_vect_normed);

        // Check normalisation
        double sum = Arrays.stream(w_rho).sum();
        if (FastMath.abs(sum - 1.) > 0.1) {
            throw new IllegalStateException("Error: CAR GMM range weights not normalised!" 
                                                + " Sum: " + sum);
        }

        // Compute PDF sum
        double[] g_approx = new double[M];
        for (int i=0; i<M; i++) {
            double gi = 0.;
            double rhoi = rho_unique[i];
            for (int j=0; j<L_rho; j++) {
                double wj = w_rho[j];
                double mj = m_rho[j];
                gi += wj * (1./FastMath.sqrt(2.*FastMath.PI*P_rho)) 
                         * FastMath.exp(-(rhoi-mj) * (rhoi-mj)/(2*P_rho));            
            }
            g_approx[i] = gi;
        }

        write_RhoUnique_pVect_gApprox_toCsV("RhoUnique_pVect_gApprox.csv", 
                                            rho_unique, 
                                            p_vect_normed,
                                            g_approx);

        // Compute range-rate marginal PDF quantities and store in GMM
        // Get drho limits for m_rho
        Map<Double, double[]> drho_dict2 = car_drho_limits(epoch, m_rho, false);
        List<Map.Entry<Double, double[]>> drho_dict2_list = new ArrayList<>(drho_dict2.entrySet());
        double sig_drho_max = 0.;

        List<Double> w = new ArrayList<Double>();
        List<double[]> m = new ArrayList<double[]>();
        List<double[][]> P = new ArrayList<double[][]>();

        for (int i =0; i<L_rho; i++) {

            // Get values from range PDF
            double wi = w_rho[i];
            double mi = m_rho[i];
            
            // Get values from Range-Rate PDF
            Map.Entry<Double, double[]> drho_dict2_entry = drho_dict2_list.get(i);
            double[] drho_vect = drho_dict2_entry.getValue();

            for (int k=0; k<(int)drho_vect.length/2; k++) {
                double[] drho_k = new double[]{drho_vect[2*k], drho_vect[2*k+2-1]};
                double a_drho = Arrays.stream(drho_k).min().orElseThrow(null);
                double b_drho = Arrays.stream(drho_k).max().orElseThrow(null);   
                double[] LDrhoandSigmaOutDrho = 
                    car_sigma_library(a_drho, b_drho, sigma_drho_desired);
                int L_drho = (int)LDrhoandSigmaOutDrho[0];
                double sig_drho = LDrhoandSigmaOutDrho[1];

                if (sig_drho > sig_drho_max) {
                    sig_drho_max = sig_drho;
                }

                // Weights, means, covar for this rho
                double wj = 1./L_drho;
                double Pj = sig_drho * sig_drho;

                for (int j=0; j<L_drho; j++) {
                    double mj = a_drho + (b_drho-a_drho)/(L_drho + 1.) * (j+1.);
                    w.add(wi*wj);
                    m.add(new double[]{mi, mj});
                    double[][] P_entry = new double[2][2];
                    P_entry[0] = new double[]{P_rho, 0.};
                    P_entry[1] = new double[]{0., Pj};
                    P.add(P_entry);
                }
            }
        }

        // Turn w into a one-dim array
        double[] w_array = w.stream().mapToDouble(Double::doubleValue).toArray();

        // Turn m into a multidimensional array
        double[][] m_array = new double[m.size()][];
        for (int i = 0; i < m.size(); i++) {
            m_array[i] = m.get(i);
        }
        writemrhoREmdrho("mrhoREmdrho.csv", m_array);
        
        return new GaussianMixtureModel(epoch, w_array, m_array, P);
    }
        
    private void writemrhoREmdrho(String filename, double[][] m_array) {
        // Extract rho and drho
        double[] mrho_RE = new double[m_array.length];
        double[] mdrho = new double[m_array.length];
        for (int i=0; i<m_array.length; i++) {
            mrho_RE[i] = m_array[i][0];
            mdrho[i] = m_array[i][1];

        }
        try (FileWriter writer = new FileWriter(filename)) {
            // Write header
            writer.append("mrho_RE,mdrho\n");
            
            // Determine max length for iteration
            int maxLength = m_array.length;
            
            // Write data row by row
            for (int i = 0; i < maxLength; i++) {
                writer.append(i < m_array.length ? String.valueOf(m_array[i][0]) : "");
                writer.append(",");
                writer.append(i < m_array.length ? String.valueOf(m_array[i][1]) : "");
                writer.append("\n");
            }
            
            System.out.println("CSV file saved successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public GaussianMixtureModel car_gmm_to_eci(GaussianMixtureModel gmm, double[] meas_noise) {

        // Break out GM
        double[][] m0 = gmm.getMeans();
        List<double[][]> P0 = gmm.getP();

        // Get sigmas for meas_types
        double[] var_vect = new double[meas_noise.length];
        for (int i=0; i<var_vect.length; i++) {
            var_vect[i] = meas_noise[i] * meas_noise[i];
            if (var_vect[i]< 1.e-10) {
                var_vect[i] = 1.e-9;
            }
        }

        // Output 
        double[][] m_list = new double[m0.length][m0[0].length];
        List<double[][]> P_list = new ArrayList<double[][]>();

        // For each GM component use unscented transform to put in ECI
        int L = gmm.getWeights().length;
        for (int j=0; j<L; j++) {
            double[] mj = 
                DoubleStream.concat(Arrays.stream(m0[j]), Arrays.stream(tracklet)).toArray();
            double[] diagonal = new double[P0.get(j).length];

            for (int i=0; i<diagonal.length; i++) {
                diagonal[i] = P0.get(j)[i][i];  // Extract diagonal element
            }
            double[] concartPj = 
                DoubleStream.concat(Arrays.stream(diagonal), Arrays.stream(var_vect)).toArray();
            RealMatrix Pj = new DiagonalMatrix(concartPj);
            Map.Entry<RealVector, RealMatrix> mP = unscented_transform(mj, Pj);
            m_list[j] = mP.getKey().toArray();
            P_list.add(mP.getValue().getData());
        }
        GaussianMixtureModel gmmEci = new GaussianMixtureModel(gmm.getEpoch(), gmm.getWeights(), m_list, P_list);

        // Check if weights are normalised
        double sumWeights = 0.;
        double[] weights = gmmEci.getWeights();
        for (int i=0; i<weights.length; i++) {
            sumWeights = sumWeights + weights[i];
        }
        if (FastMath.abs(sumWeights - 1.) > 0.1) {
            throw new Error("Weights in GmmECI are not normalised");
        }
    
        return gmmEci;
    }
    
    /**
     * Transformation from range/range rate space to ECI.
     * 
     * @param mj
     * @param pj
     * @return
     */
    private Map.Entry<RealVector, RealMatrix> unscented_transform(double[] mj, RealMatrix pj) {

        // Number of states
        int L = mj.length;

        // Value of p-norm distribution
        double pnorm = 2.;

        // Sigma point distribution parameter
        double alpha = 1.;

        // Prior information about the distribution
        double kurt = Gamma.gamma(5./pnorm) * Gamma.gamma(1./pnorm) 
                        / FastMath.pow(Gamma.gamma(3./pnorm), 2);
        double beta = kurt - 1.;
        double kappa = kurt - (double)L;

        // Compute sigma point weights
        double lam = alpha * alpha * (L+kappa) - L;
        double gam = FastMath.sqrt(L + lam);
        RealVector ones = new ArrayRealVector(new double[2*L]);
        ones.set(1.);
        RealVector Wm =  ones.mapMultiply(1./(2. * (L + lam)));
        RealVector Wc = Wm.copy();
        double firstWm = lam/(L + lam);
        RealVector Wm_append = new ArrayRealVector(new double[]{firstWm}).append(Wm);
        RealVector Wc_append = 
            new ArrayRealVector(new double[]{firstWm + (1 - alpha * alpha + beta)}).append(Wc);
        RealMatrix diagWc = new DiagonalMatrix(Wc_append.toArray());

        //Compute chi - baseline sigma points
        RealMatrix sqP = new CholeskyDecomposition(pj).getL();
        double[][] Xrep = new double[mj.length][L];
        for (int i=0; i<L; i++) {
            Arrays.fill(Xrep[i], mj[i]);
        }
        // Positive and negative deviations
        RealMatrix posDev = new Array2DRowRealMatrix(Xrep).add(sqP.scalarMultiply(gam));
        RealMatrix negDev = new Array2DRowRealMatrix(Xrep).subtract(sqP.scalarMultiply(gam));

        // Transform mj into a column vector
        double[][] m1 = new Array2DRowRealMatrix(mj).getData();
        double[][] sigmaPoints = 
            MatrixTools.concatenateColumns(posDev.getData(), negDev.getData());
        double[][] chi = MatrixTools.concatenateColumns(m1,sigmaPoints);
        double[][] chi_diff_matrix = MatrixTools.subtractEachColumnByVec(chi, mj);
        RealMatrix chi_diff = new Array2DRowRealMatrix(chi_diff_matrix);

        // Compute sigma points
        double[][] Y_matrix = ut_car_to_eci(chi);
        RealMatrix Y = new Array2DRowRealMatrix(Y_matrix);

        // Compute mean and covar
        RealVector m2 = Y.operate(Wm_append);
        double[][] Y_diff_matrix = MatrixTools.subtractEachColumnByVec(Y_matrix, m2.toArray());
        RealMatrix Y_diff = new Array2DRowRealMatrix(Y_diff_matrix);
        RealMatrix diagWc_YdiffT = diagWc.multiply(Y_diff.transpose());
        RealMatrix P2 = Y_diff.multiply(diagWc_YdiffT);
        RealMatrix Pcross = chi_diff.multiply(diagWc_YdiffT);
        
        return new AbstractMap.SimpleEntry<>(m2, P2);                
    }
    
    /**
     * Function for use with unscented_transform.
     * Converts sigma point matrix to inertial cartesian coordinates
     * 
     * @param chi
     */
    private double[][] ut_car_to_eci(double[][] chi) {
        int L = chi[0].length;
        double[][] Y = new double[chi.length][L];

        for (int ind=0; ind<L; ind++) {

            // Break out chi
            double rho = chi[0][ind];
            double drho = chi[1][ind];
            double ra = chi[2][ind];
            double dec = chi[3][ind];
            double dra = chi[4][ind];
            double ddec = chi[5][ind];

            // Unit vectors
            double[] u_rho = new double[]{FastMath.cos(ra) * FastMath.cos(dec), 
                                            FastMath.sin(ra) * FastMath.cos(dec), 
                                            FastMath.sin(dec)};
            RealVector u_rho_vect = new ArrayRealVector(u_rho);

            double[] u_ra = new double[]{-FastMath.sin(ra) * FastMath.cos(dec),
                                            FastMath.cos(ra) * FastMath.cos(dec),
                                            0.};
            RealVector u_ra_vect = new ArrayRealVector(u_ra);

            double[] u_dec = new double[]{-FastMath.cos(ra) * FastMath.sin(dec),
                                            -FastMath.sin(ra) * FastMath.sin(dec),
                                            FastMath.cos(dec)};
            RealVector u_dec_vect = new ArrayRealVector(u_dec);

            // Range and Range-Rate vectors
            RealVector rho_vect = u_rho_vect.mapMultiply(rho);
            RealVector drho_vect = u_rho_vect.mapMultiply(drho)
                                    .add(u_ra_vect.mapMultiply(rho * dra))
                                    .add(u_dec_vect.mapMultiply(rho * ddec));
    
            // Compute pos/vel in ECI and add to output
            RealVector r_vect = new ArrayRealVector(q.toArray()).add(rho_vect);
            RealVector v_vect = new ArrayRealVector(dq.toArray()).add(drho_vect);
            for (int i=0; i<r_vect.getDimension(); i++) {
                Y[i][ind] = r_vect.getEntry(i);
                Y[i+r_vect.getDimension()][ind] = v_vect.getEntry(i);
            }                                      
        }
        return Y;
    }
            
    private void write_RhoUnique_pVect_gApprox_toCsV(String filename, double[] rho_unique, 
                                        double[] p_vect, double[] g_approx) {
                                                        
        try (FileWriter writer = new FileWriter(filename)) {
            // Write header
            writer.append("rho_unique,p_vect,g_approx\n");
            
            // Determine max length for iteration
            int maxLength = Math.max(rho_unique.length, Math.max(p_vect.length, g_approx.length));
            
            // Write data row by row
            for (int i = 0; i < maxLength; i++) {
                writer.append(i < rho_unique.length ? String.valueOf(rho_unique[i]) : "");
                writer.append(",");
                writer.append(i < p_vect.length ? String.valueOf(p_vect[i]) : "");
                writer.append(",");
                writer.append(i < g_approx.length ? String.valueOf(g_approx[i]) : "");
                writer.append("\n");
            }
            
            System.out.println("CSV file saved successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
        
    }
        
    /**
     * This function returns the sigma value required to approximate a uniform
     * distribution with a GMM with "L" homoscedastic, evenly spaced, and
     * evenly weighted components. Library based on standard uniform distribution
     * (a = 0, b = 1, p = 1/(b-a)).  Will return result for minimum number of
     * components required to achieve desired std or lower, up to max of 15
     * components.
     * 
     * @param a
     * @param b
     * @param sigma_rho_desired
     * @return
     */
    public double[] car_sigma_library(double a, double b, double sigma_rho_desired) {

        double[] LandSigmaOut = new double[2];
        for (int L=1; L<this.lookUp.length; L++) {
            double sigma_out = (b-a) * lookUp[L];
            LandSigmaOut[0] = L;
            LandSigmaOut[1] = sigma_out;
            if (sigma_out < sigma_rho_desired) {
                break;
            }
        }
        return LandSigmaOut;
    }

    /**
     * Normalises the unnormalised approximated marginal PDF by the total probability mass of rho_unique.
     * This is necessary so that the integral of the marginal PDF is 1.
     * 
     * @param p_vect            Approximation of the marginal probability mass for each rho_unique
     * @param rho_unique        Random variable
     * @return                  Normalised PDF of rho_unique.
     */
    public static double[] computeMarginalPDF(double[] p_vect, double[] rho_unique) {
        double norm_fact = trapezoidalIntegration(p_vect, rho_unique);
        
        double[] p_marginal_rho = new double[p_vect.length];
        for (int i = 0; i < p_vect.length; i++) {
            p_marginal_rho[i] = p_vect[i] / norm_fact;  // Normalize to get the PDF
        }
        
        return p_marginal_rho;
    }

    /**
     * Computes the numerical integral of a function using the trapezoidal rule.
     * This method approximates the integral of the given function values over
     * a set of discrete points.
     *
     * <p>The trapezoidal rule estimates the integral by dividing the area under
     * the curve into trapezoids and summing their areas. It is commonly used for
     * numerical integration when the function is only known at discrete points.
     * 
     * @param y                 The function values at each discrete point (corresponding to f(x)).
     * @param x                 The discrete points (must be sorted in ascending order).
     * @return                  The approximate integral of the function over the given range.
     * 
     * @throws IllegalArgumentException if the input arrays have different lengths
     *                                  or contain fewer than two points.
     */
    public static double trapezoidalIntegration(double[] y, double[] x) {
        if (y.length != x.length || y.length < 2) {
            throw new IllegalArgumentException("Arrays must have the same length" 
                                                + " and contain at least two points.");
        }
    
        double integral = 0.0;
        for (int i = 0; i < y.length - 1; i++) {
            double dx = x[i + 1] - x[i];                    // Non-uniform spacing between x values
            double avgHeight = (y[i + 1] + y[i]) / 2.0;     // Trapezoidal rule
            integral += dx * avgHeight;                     // Area of the trapezoid
        }
    
        return integral;
    }

    /**
     * Compute attributable from list of optical measurements.
     * 
     * @param angles                Optical measurements.
     * @return                      Array of angles (ra/dec), angle rates (raDot/decDot) and time  
     *                              elapsed from epoch of first measurements to mid position 
     *                              inside tracklet.
     */
    public static double[] linearRegressionMeasurements(List<AngularDirection> angles) {

        // Prepare linear regression
        SimpleRegression regRa = new SimpleRegression();
        SimpleRegression regDec = new SimpleRegression();

        // Feed data
        for (int i=0; i<angles.size(); i++) {
            double elapsed = angles.get(i).getDate().durationFrom(angles.get(0).getDate());
            regRa.addData(elapsed, angles.get(i).getAngle1());
            regDec.addData(elapsed, angles.get(i).getAngle2());
        }

        // Retrieve slopes
        double raDot = regRa.getSlope();
        double decDot = regDec.getSlope();

        // Interpolate mid angles
        double timeMid = 
            0.5 *angles.get(angles.size() - 1).getDate().durationFrom(angles.get(0).getDate());
        double raMid = regRa.predict(timeMid);
        double decMid = regDec.predict(timeMid);

        // Return attributable
        return new double[]{raMid, decMid, raDot, decDot, timeMid};
    }

    @Override
    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setMicroAction'");
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getExecusionDuration'");
    }

    @Override
    public List propagateOutcome() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'propagateOutcome'");
    }

    public GaussianMixtureModel car_gmm_eci_to_topoInertial(GaussianMixtureModel gmmEci, 
                                                            Frame topoInertial,
                                                            AbsoluteDate date) {
        // change means
        double[][] mEci = gmmEci.getMeans();
        double[][] mTopo = new double[mEci.length][mEci[0].length];

        Transform t = FramesFactory.getEME2000().getTransformTo(topoInertial, date);
        for (int i=0; i<mEci.length; i++) {
            PVCoordinates pvEci = 
                new PVCoordinates(new Vector3D(mEci[i][0], mEci[i][1], mEci[i][2]), 
                                  new Vector3D(mEci[i][3], mEci[i][4], mEci[i][5]));
            PVCoordinates pvTopo = t.transformPVCoordinates(pvEci);
            mTopo[i] = new double[]{pvTopo.getPosition().getX(), 
                                    pvTopo.getPosition().getY(), 
                                    pvTopo.getPosition().getZ(),
                                    pvTopo.getVelocity().getX(),
                                    pvTopo.getVelocity().getY(),
                                    pvTopo.getVelocity().getZ()};
        }
        // Frame transformation has no effect on weights and covariance
        return new GaussianMixtureModel(gmmEci.getEpoch(), 
                                        gmmEci.getWeights(), 
                                        mTopo, 
                                        gmmEci.getP());
    }   
}