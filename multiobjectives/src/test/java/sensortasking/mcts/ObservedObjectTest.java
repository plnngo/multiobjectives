package sensortasking.mcts;

import java.io.File;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.AbsolutePVCoordinates;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

public class ObservedObjectTest {
    @Before
    public void init() {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }

    /**
     * Test {@link ObservedObject#spacecraftStateToStateVector(SpacecraftState)} using the  
     * reference coordinates from Vallado's Example 4-1 in "Fundamentals of Astrodynamics and 
     * Applications". 
     */
    @Test
    public void testSpacecraftStateToStateVector(){

        AbsoluteDate epoch = 
            new AbsoluteDate(1994, 5, 14, 13, 11, 20.59856, TimeScalesFactory.getUTC());
        Vector3D pos = new Vector3D(5036.736529 * 1e3, -10806.660797 * 1e3, -4534.633784 * 1e3);
        Vector3D vel = new Vector3D(2.6843855 * 1e3, -5.7595920 * 1e3, -2.4168093* 1e3);
        TimeStampedPVCoordinates timePosVel = 
            new TimeStampedPVCoordinates(epoch, new PVCoordinates(pos, vel));
        AbsolutePVCoordinates pv = 
            new AbsolutePVCoordinates(FramesFactory.getGCRF(), timePosVel);
        SpacecraftState scState = new SpacecraftState(pv);

        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        GeodeticPoint station = new GeodeticPoint(FastMath.toRadians(39.007),     // Geodetic latitude
                                                  FastMath.toRadians(-104.883),     // Longitude
                                         2194.56);                         // Altitude in [m]
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "Topocentric Horizon");

        StateVector stateVec = ObservedObject.spacecraftStateToStateVector(scState, topoHorizon);
        Vector3D posTopoHorizon = stateVec.getPositionVector();

        // Due to different angle definition, azimuth needs to get converted according to Vallado
        double actualAzOrekit = FastMath.toDegrees(posTopoHorizon.getAlpha());
        double actualAzVallado = 180 - (actualAzOrekit + 90.);

        // Compare range, azimuth and elevation with Vallado
        Assert.assertEquals(210.8777747, actualAzVallado, 1e-3);
        Assert.assertEquals(-5.9409535, FastMath.toDegrees(posTopoHorizon.getDelta()), 1e-3);
        Assert.assertEquals(11710812, posTopoHorizon.getNorm(), 1e2);
    }

    /**
     * Test {@link ObservedObject#stateCovToCartesianCov(org.orekit.orbits.Orbit, StateCovariance, Frame)}
     * using test data from "Covariance Transformations for Satellite Flight Dynamics Operations"
     * by David A. Vallado and the test class related to 
     * {@link org.orekit.propagation.StateCovariance#changeCovarianceFrame(org.orekit.orbits.Orbit, Frame)}.
     * The input data in the orekit test function is given wrt GCRF. However, 
     * {@link ObservedObject#stateCovToCartesianCov(org.orekit.orbits.Orbit, StateCovariance, Frame)} 
     * is usually called on state and covariances defined in TEME. For this reason the initial 
     * input parameters are first converted into TEME. Furthermore, the orekit test function only 
     * provides the transformed covariance matrix in ITRF. Hence the resulting covariance is then 
     * transformed into ITRF and compared with the results of orekit. In the last step, the 
     * transformation into the topocentric horizon frame (ENU) needs to be manually verified. The 
     * output covariance in ITRF {@code covITRF} is converted to ENU using the appropiate rotation
     * matrix {@code rotEcefToEnu}. The result is compared with the outcome of 
     * {@link ObservedObject#stateCovToCartesianCov(org.orekit.orbits.Orbit, StateCovariance, Frame)}
     * which computes the state covariance directly into ENU from an input defined in TEME.
     * 
     * @see org.orekit.propagation.StateCovariance
     */
    @Test
    public void testStateCovToCartesianCov() {

        // Create frames
        Frame eci = FramesFactory.getGCRF();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, false);
        Frame teme = FramesFactory.getTEME();

        // Location at United States Air Force Academy according to Vallado Example 4-1
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),   // Geodetic latitude
                                              FastMath.toRadians(-104.883),   // Longitude
                                              2194.56);              // in [m]

        // Set up topocentric horizon frame
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, pos, "United States Air Force Academy");

        // Set up input data from Vallado
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        PVCoordinates pvEci = 
            new PVCoordinates(new Vector3D(-605792.21660, -5870229.51108, 3493053.19896),
                              new Vector3D(-1568.25429, -3702.34891, -6479.48395));

        // Transform from ECI to TEME
        Transform eciToTeme = eci.getTransformTo(teme, date);
        PVCoordinates pvTeme = eciToTeme.transformPVCoordinates(pvEci);

        // Set covariance matrix
        double[][] covArray = new double[6][6];
        covArray[0] = 
            new double[]{0.9999945079552018, 0.009999542228922726, 0.009997709967860641, 
                         9.994507954236584E-5, 9.999542225752099E-5, 9.997710067646445E-5};
        covArray[1] =
            new double[]{0.009999542228922725, 1.0000045790384262, 0.010002745854447466, 
                         9.999542227959272E-5, 1.0004579035257533E-4, 1.0002746172251207E-4};
        covArray[2] =
            new double[]{0.00999770996786064, 0.010002745854447466, 1.0000009130063703, 
                         9.997709871242466E-5, 1.0002745537610997E-4, 1.000091301050587E-4};
        covArray[3] =
            new double[]{9.994507954236582E-5, 9.999542227959272E-5, 9.997709871242466E-5, 
                         9.99450795327065E-7, 9.999542224788645E-7, 9.997709971028265E-7};
        covArray[4] =
            new double[]{9.999542225752099E-5, 1.0004579035257532E-4, 1.0002745537610997E-4, 
                         9.999542224788643E-7, 1.0004579032088274E-6, 1.0002745855414732E-6};
        covArray[5] = 
            new double[]{9.997710067646443E-5, 1.0002746172251205E-4, 1.0000913010505871E-4, 
                         9.99770997102827E-7, 1.0002745855414737E-6, 1.000091301464106E-6};
        RealMatrix covMatrix = MatrixUtils.createRealMatrix(covArray);
        StateCovariance covTeme = new StateCovariance(covMatrix, date, teme, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        CartesianCovariance covEcef = ObservedObject.stateCovToCartesianCov(orbit, covTeme, ecef);
        RealMatrix covITRF = covEcef.getCovarianceMatrix();

        // Verify that transformation from eci to ecef worked correctly
        double[][] covItrfOrekit = new double[6][6];
        covItrfOrekit[0] = 
            new double[]{0.9934000575390601, 0.007512499898238329, 0.00583126745930062,
                         3.454839551824719E-5, 2.685122990788755E-6, 5.831267711564615E-5};
        covItrfOrekit[1] =
            new double[]{0.007512499898238334, 1.0065990293024842, 0.01288431024807918, 
                         1.4852735913336836E-4, 1.654424728247383E-4, 1.288431070222388E-4};
        covItrfOrekit[2] =
            new double[]{0.005831267459300618, 0.012884310248079177, 1.0000009131584502,
                         5.925221082546215E-5, 1.284178752507414E-4, 1.00009131657014E-4};
        covItrfOrekit[3] =
            new double[]{3.454839551824716E-5, 1.4852735913336836E-4, 5.9252210825462165E-5,
                         3.5631474107176443E-7, 7.608348837390666E-7, 5.925221338121884E-7};
        covItrfOrekit[4] =
            new double[]{2.6851229907887606E-6, 1.654424728247383E-4, 1.284178752507414E-4,
                         7.608348837390664E-7, 1.6542289254103502E-6, 1.2841787977379298E-6};
        covItrfOrekit[5] = 
            new double[]{5.831267711564615E-5, 1.288431070222388E-4, 1.0000913165701402E-4,
                         5.925221338121886E-7, 1.2841787977379302E-6, 1.0000913172951267E-6};

        for (int row=0; row<covItrfOrekit.length; row++) {
            double[] actualRowVec = covITRF.getRow(row);
            for (int col=0; col<covItrfOrekit[row].length; col++){
                Assert.assertEquals(covItrfOrekit[row][col], actualRowVec[col], 1e-6);
            }
        }

        // Rotational matrix from ECEF to ENU (Topocentric Horizon Frame)
        double[][] rot = new double[6][6];
        double cosLon = FastMath.cos(pos.getLongitude());
        double sinLon = FastMath.sin(pos.getLongitude());
        double cosLat = FastMath.cos(pos.getLatitude());
        double sinLat = FastMath.sin(pos.getLatitude());
        
        rot[0] = new double[]{-sinLon, cosLon, 0., 0., 0., 0.};
        rot[1] = new double[]{-cosLon*sinLat, -sinLon*sinLat, cosLat, 0., 0., 0.};
        rot[2] = new double[]{cosLon*cosLat, sinLon*cosLat, sinLat, 0., 0., 0.};
        rot[3] = new double[]{0., 0., 0., -sinLon, cosLon, 0.};
        rot[4] = new double[]{0., 0., 0., -cosLon*sinLat, -sinLon*sinLat, cosLat};
        rot[5] = new double[]{0., 0., 0., cosLon*cosLat, sinLon*cosLat, sinLat};
        
        RealMatrix rotEcefToEnu = MatrixUtils.createRealMatrix(rot);
        RealMatrix covExpected = rotEcefToEnu.multiply(covITRF.multiplyTransposed(rotEcefToEnu));

        CartesianCovariance covTopo = 
            ObservedObject.stateCovToCartesianCov(orbit, covTeme, topoHorizon);
        RealMatrix covActual = covTopo.getCovarianceMatrix();

        // Verify transformation to ENU (Topocentric Horizon frame)
        for (int row=0; row<covExpected.getRowDimension(); row++) {
            double[] rowVecExpected = covExpected.getRow(row);
            double[] rowVecActual = covActual.getRow(row);
            for (int col=0; col<covExpected.getColumnDimension(); col++) {
                Assert.assertEquals(rowVecExpected[col], rowVecActual[col], 1e-14);
            }
        }
    }
}
