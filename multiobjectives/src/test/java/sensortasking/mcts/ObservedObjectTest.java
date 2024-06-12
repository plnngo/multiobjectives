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
import org.orekit.orbits.PositionAngle;
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
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, false);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               earthFrame);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, pos, "United States Air Force Academy");

        // Set up input data from Vallado
        AbsoluteDate date = new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        PVCoordinates pvEci = 
            new PVCoordinates(new Vector3D(-605792.21660, -5870229.51108, 3493053.19896),
                              new Vector3D(-1568.25429, -3702.34891, -6479.48395));

        // Transform from ECI to TEME
        Transform eciToTeme = eci.getTransformTo(teme, date);
        PVCoordinates pvTeme = eciToTeme.transformPVCoordinates(pvEci);

        // Generate test object 
        //SpacecraftState scState = new SpacecraftState(new AbsolutePVCoordinates(teme, date, pvTeme));

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
        StateCovariance covTeme = new StateCovariance(covMatrix, date, teme, OrbitType.CARTESIAN, PositionAngle.MEAN);

        CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        CartesianCovariance covEcef = ObservedObject.stateCovToCartesianCov(orbit, covTeme, ecef);
        RealMatrix covITRF = covEcef.getCovarianceMatrix();

        // Rotational matrix from ECEF to ENU (Topocentric Horizon Frame)
        double[][] rot = new double[6][6];
        rot[0] = new double[]{-FastMath.sin(pos.getLongitude()), FastMath.cos(pos.getLongitude()), 0., 0., 0.};
        
        RealMatrix rotEcefToEnu = MatrixUtils.createRealMatrix(rot);


        CartesianCovariance covTopo = 
            ObservedObject.stateCovToCartesianCov(orbit, covTeme, topoHorizon);
        RealMatrix covTopoMatrix = covTopo.getCovarianceMatrix();

        System.out.println("Covariance in ITRF");
        for (int row=0; row<covITRF.getRowDimension(); row++) {
                double[] rowVector = covITRF.getRow(row);
                for (int col=0; col<covITRF.getColumnDimension(); col++) {
                        System.out.print(rowVector[col] + " ");
                }
                System.out.println();
        }
        System.out.println("Covariance in Topocentric horizon frame");
        for (int row=0; row<covTopoMatrix.getRowDimension(); row++) {
                double[] rowVector = covTopoMatrix.getRow(row);
                for (int col=0; col<covTopoMatrix.getColumnDimension(); col++) {
                        System.out.print(rowVector[col] + " ");
                }
                System.out.println();
        }
    }
}
