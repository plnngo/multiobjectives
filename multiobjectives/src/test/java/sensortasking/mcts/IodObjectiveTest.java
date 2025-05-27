package sensortasking.mcts;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.util.FastMath;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.estimation.measurements.AngularRaDec;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.frames.Frame;

import tools.GaussianMixtureModel;

public class IodObjectiveTest {

    @Before
     public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }

    @Test
    public void test_optical_car_gmm() {

        // Measurement vector
/*         double ra = FastMath.toRadians(10.);
        double dec = FastMath.toRadians(-2.);
        double dra = FastMath.toRadians(15.)/3600.;
        double ddec = FastMath.toRadians(3.)/3600.;
        double[] tracklet = new double[]{ra, dec, dra, ddec}; */
        double[] trackletTime = generateTestAttribute();

        // Date reference frame
        AbsoluteDate date = 
            new AbsoluteDate("2025-03-24T22:05:10.000Z", TimeScalesFactory.getUTC())
                .shiftedBy(trackletTime[trackletTime.length - 1]);

        double[] trackletEci = new double[]{trackletTime[0], trackletTime[1], 
                                         trackletTime[2], trackletTime[3]};

        // CAR limits
/*         double a_max = 50000.*1000.;            // m
        double a_min = 0.;            // m
        double e_max = 0.4; */
        double a_max = 45000.*1000.;            // m
        double a_min = 40000.*1000;            // m
        double e_max = 0.02;

        // Range interval
        double[] rho_vect = new double[10000];
        int count = 0;
        for (int i=0; i<rho_vect.length; i++) {
            rho_vect[i] = count;
            count = count + 5000;
        }

        // Desired maximum standard deviation in range 
        double sigma_rho_desired = 500.;       // m
        double sigma_drho_desired = 100.;       // m/s
        //double sigma_drho_desired = 10.;       // m/s

        // Measurement noise
        double arcsec2rad = 1./3600. * FastMath.PI/180.;
        double[] meas_noise = new double[]{0.4 * arcsec2rad,
                                           0.4 * arcsec2rad,
                                           0.07 * arcsec2rad,
                                           0.07 * arcsec2rad};

        // Set up sensor
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0);              // in [m]
        double readout = 7.;
        double exposure = 8.;
        double settling = 10.;
        double cutOff = FastMath.toRadians(5.);
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(2.), FastMath.toRadians(2.));
        double slewVel = FastMath.toRadians(1.)/1.;     // 1 deg per second
        Sensor sensor = new Sensor("TDRS Station", fov, pos, exposure, readout, slewVel, settling, cutOff);
        //Frame topoInertial = sensor.getTopoInertialFrame(date);

        IodObjective obj = 
            new IodObjective(trackletEci, sensor, a_max, a_min, e_max, meas_noise, date);
        //obj.car_drho_limits(date, rho_vect, true);
        GaussianMixtureModel gmm = obj.optical_car_gmm(date, rho_vect, sigma_rho_desired, sigma_drho_desired);
        double sum = 0.;
        for (int i=0; i<gmm.getWeights().length;i++) {
            sum = sum + gmm.getWeights()[i];
        }
        GaussianMixtureModel gmmEci = obj.car_gmm_to_eci(gmm, meas_noise);

        // Transform GMM from ECI to topo inertial frame
        //GaussianMixtureModel gmmTopoInertial  = obj.car_gmm_eci_to_topoInertial(gmmEci, topoInertial, date);

        AbsoluteDate measEpoch = new AbsoluteDate(2025, 3, 24, 22, 10, 1.62, TimeScalesFactory.getUTC());

        // Generate measurement in topocentric inertial frame at epoch date
        double[] simMeas = simulateMeasurement(measEpoch, FramesFactory.getEME2000());
        //double[] simMeas = new double[]{-3.0649966351518683, -0.013068743616550393};

        // Station
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topo = new TopocentricFrame(earth, pos, "Topocentric");
        GroundStation gs = new GroundStation(topo);
        double sigma2 = FastMath.pow(0.4*arcsec2rad, 2);
        AngularRaDec angles = new AngularRaDec(gs, FramesFactory.getEME2000(), measEpoch,
                                               simMeas, new double[]{sigma2, sigma2}, 
                                               new double[]{1., 1.}, 
                                               new ObservableSatellite(0));
        
        System.out.println(FastMath.toDegrees(simMeas[0])+360);
        System.out.println(FastMath.toDegrees(simMeas[1]));

        List<ObservedMeasurement<?>> meas = new ArrayList<ObservedMeasurement<?>>();
        meas.add(angles);
/*         RealVector ybar = new ArrayRealVector(new double[]{-3.06499664,-0.01306874});
        double[][] entries_pyy = new double[2][2];
        entries_pyy[0] = new double[]{9.73403669e-09,-1.05352176e-09};
        entries_pyy[1] = new double[]{-1.05352176e-09, 1.03473434e-09};
        RealMatrix pyy = new Array2DRowRealMatrix(entries_pyy);
        GaussianMixtureModel.getMultivariateGaussianLikelihood(meas, ybar, pyy); */
/*         GaussianMixtureModel gmmUpdated = 
            GaussianMixtureModel.gmm_ukf(gmmTopoInertial, topoInertial, meas); */
        GaussianMixtureModel gmmUpdated = 
            GaussianMixtureModel.gmm_ukf_own(gmmEci, FramesFactory.getEME2000(), meas);
        for (int i=0; i<gmmUpdated.getWeights().length; i++) {
            System.out.print(gmmUpdated.getWeights()[i] + ", ");
        }
        //writeCovariance("PrintCovs.csv", gmmUpdated.getP());
        //writeMeans("PrintMeans.csv", gmmUpdated.getMeans());
        GaussianMixtureModel gmmF = GaussianMixtureModel.mergeAndPrune(gmmUpdated);

    }

    private void writeMeans(String filename, double[][] means) {
        try (FileWriter writer = new FileWriter(filename)) {
                        
            // Write data row by row
            for (int i = 0; i < means.length; i++) {
                for (int j=0; j<means[0].length; j++) {
                    writer.append(String.valueOf(means[i][j]));
                    if (j!=means[0].length-1) {
                        writer.append(",");
                    } else {
                        writer.append("\n"); 
                    }
                }
            }
                
        System.out.println("CSV file saved successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        } 
    }

    public static void writeCovariance(String filename, List<double[][]> p) {
        try (FileWriter writer = new FileWriter(filename)) {
            for (double[][] matrix : p) {
                for (double[] row : matrix) {
                    // Convert row to comma-separated string
                    String rowString = convertRowToCSV(row);
                    writer.write(rowString + "\n");
                }
                writer.write("\n"); // Separate matrices with a blank line
            }
            System.out.println("CSV file written successfully: " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static String convertRowToCSV(double[] row) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < row.length; i++) {
            sb.append(row[i]);
            if (i < row.length - 1) {
                sb.append(","); // Separate values with commas
            }
        }
        return sb.toString();
    }

    private double[] simulateMeasurement(AbsoluteDate date, Frame frame) {

        // Extract object's osculating measurements
        TLE obj = new TLE("1 22724U 93048B   25080.65854828  .00000127  00000-0  00000-0 0  9992",
                          "2 22724  14.3105   5.9714 0015950 216.1428 237.0470  0.99970286116059");
        TLEPropagator sgp4 = TLEPropagator.selectExtrapolator(obj);
        SpacecraftState state = sgp4.getInitialState();

        // Use Keplerian propagator to extract angular position
        KeplerianPropagator kepProp = new KeplerianPropagator(state.getOrbit());
        PVCoordinates pv = kepProp.propagate(date).getPVCoordinates(frame);
        return new double[]{pv.getPosition().getAlpha(), pv.getPosition().getDelta()};
    }
    
    /**
     * Test attribute derived from stripe scan search algorithm corresponding to object 22724.
     * Attribute consits of ra, dec, raDot, decDot and the time elapsed from epoch of first 
     * measurements to mid position inside tracklet.
     * 
     * @return              Attributable corresponding to object 22724 that was detected in 
     *                      stripe scanning mode.
     */
    public double[] generateTestAttribute() {
        Frame eci = FramesFactory.getEME2000();
        double geoDistance = Constants.WGS84_EARTH_EQUATORIAL_RADIUS + 35786 * 1e3;  // in m

        // Set up measurements
        double[] dir1 = new double[]{FastMath.toRadians(192.7055590353602),
                                     FastMath.toRadians(-1.7132729865117795)};
        AngularDirection angle1 = new AngularDirection(eci, dir1, AngleType.RADEC, geoDistance);
        angle1.setDate(new AbsoluteDate("2025-03-24T22:05:10.000Z", TimeScalesFactory.getUTC()));
        double[] dir2 = new double[]{FastMath.toRadians(192.78658089085548),
                                     FastMath.toRadians(-1.7337668239151958)};
        AngularDirection angle2 = new AngularDirection(eci, dir2, AngleType.RADEC, geoDistance);
        angle2.setDate(new AbsoluteDate("2025-03-24T22:05:30.000Z", TimeScalesFactory.getUTC()));
        double[] dir3 = new double[]{FastMath.toRadians(192.82709253672132),
                                     FastMath.toRadians(-1.744012461398498)};
        AngularDirection angle3 = new AngularDirection(eci, dir3, AngleType.RADEC, geoDistance);
        angle3.setDate(new AbsoluteDate("2025-03-24T22:05:40.000Z", TimeScalesFactory.getUTC()));
        double[] dir4 = new double[]{FastMath.toRadians(192.9081172772018),
                                     FastMath.toRadians(-1.7645011481818031)};
        AngularDirection angle4 = new AngularDirection(eci, dir4, AngleType.RADEC, geoDistance);
        angle4.setDate(new AbsoluteDate("2025-03-24T22:06:00.000Z", TimeScalesFactory.getUTC()));
        double[] dir5 = new double[]{FastMath.toRadians(192.9486303768182),
                                     FastMath.toRadians(-1.7747441871835372)};
        AngularDirection angle5 = new AngularDirection(eci, dir5, AngleType.RADEC, geoDistance);
        angle5.setDate(new AbsoluteDate("2025-03-24T22:06:10.000Z", TimeScalesFactory.getUTC()));

        // Create tracklet
        List<AngularDirection> tracklet = new ArrayList<AngularDirection>();
        tracklet.add(angle1);
        tracklet.add(angle2);
        tracklet.add(angle3);
        tracklet.add(angle4);
        tracklet.add(angle5);

        // Perform linear regression
        double[] attr = IodObjective.linearRegressionMeasurements(tracklet);

        return attr;
    }
}
