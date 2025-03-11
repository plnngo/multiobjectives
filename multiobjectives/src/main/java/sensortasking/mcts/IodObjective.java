package sensortasking.mcts;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;

import org.hipparchus.analysis.solvers.LaguerreSolver;
import org.hipparchus.complex.Complex;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;


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
    double mu = Constants.WGS84_EARTH_MU;

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


    public IodObjective(double[] tracklet, Sensor sensor, double a_max, double a_min, double e_max){
        this.tracklet = tracklet;
        this.sensor = sensor;

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
    public Map<Double, double[]> car_drho_limits(AbsoluteDate epoch, double[] rho_vect) {

        // Extract angles and derivatives from tracklet
        double ra = tracklet[0];
        double dec = tracklet[1];
        double dra = tracklet[2];
        double ddec = tracklet[3];

        //Inertial position and velocity of sensor
        Vector3D q = this.sensor.getSensorPosEci(epoch);
        Vector3D omega = new Vector3D(0., 0., Constants.WGS84_EARTH_ANGULAR_VELOCITY);
        Vector3D dq = omega.crossProduct(q);

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
        Map<Double, double[]> drho_dict = new HashMap<Double, double[]>();

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
            double guess = (drho_a.get(0) + drho_a.get(1)) / 2.;

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
        return drho_dict;
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
    public void optical_car_gmm(AbsoluteDate epoch, double[] rho_vect, double sigma_rho_desired){

        // Compute CAR boundary
        Map<Double, double[]> drho_dict = car_drho_limits(epoch, rho_vect);
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

        // Compute weights of GMM components (DeMars Eq 23)
        // Evaluate range marginal PDF at each range value
        Object[] rho_uniqueO = drho_dict.keySet().toArray();
        double[] rho_unique = new double[rho_uniqueO.length];
        double[] p_vect = new double[rho_uniqueO.length];
        for (int i=0; i<rho_unique.length; i++) {
            rho_unique[i] = ((Double)rho_uniqueO[i]).doubleValue();
            Map.Entry<Double, double[]> drho_dict_entry = drho_dict_list.get(i);
                double[] drho_vect = drho_dict_entry.getValue();
                double a_drho = Arrays.stream(drho_vect).min().orElseThrow(null);
                double b_drho = Arrays.stream(drho_vect).max().orElseThrow(null);
            if (i>0) {
                double delta_rho = rho_unique[i] - rho_unique[i-1];
                p_vect[i] = (b_drho - a_drho)*delta_rho;
            }

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
            if (sigma_out < sigma_rho_desired) {
                LandSigmaOut[0] = L;
                LandSigmaOut[1] = sigma_out;
                break;
            }
        }
        return LandSigmaOut;
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
    
}
