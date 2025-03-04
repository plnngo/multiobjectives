package sensortasking.mcts;

import java.io.File;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
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
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class AngularDirectionTest {
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
     * Test {@link AngularDirection#getEnclosedAngle(AngularDirection)} by comparing the output 
     * with an online tool: 
     * https://www.emathhelp.net/en/calculators/linear-algebra/angle-between-two-vectors-calculator/
     */
    @Test
    public void testGetEnclosedAngles() {

        // Inputs
        Frame eci = FramesFactory.getGCRF();
        Vector3D u = new Vector3D(5., -2., 3);
        Vector3D v = new Vector3D(-4, 5, 7);
        Vector3D w = u.negate();
        AngularDirection uDir = 
            new AngularDirection(eci, new double[]{u.getAlpha(), u.getDelta()}, 
                                 AngleType.RADEC, u.getNorm());
        AngularDirection vDir = 
            new AngularDirection(eci, new double[]{v.getAlpha(), v.getDelta()}, 
                                 AngleType.RADEC, v.getNorm());
        AngularDirection wDir = 
            new AngularDirection(eci, new double[]{w.getAlpha(), w.getDelta()}, 
                                 AngleType.RADEC, w.getNorm());

        // Results
        double actualUV = uDir.getEnclosedAngle(vDir);
        double expectedUV = 1.725307134097968; 
        double actualUW = uDir.getEnclosedAngle(wDir);
        double expectedUW = FastMath.PI;
        double actualVW = vDir.getEnclosedAngle(wDir);
        double expectedVW = 1.416285519491826;

        // Compare
        Assert.assertEquals(expectedUV, actualUV, 1e-14);
        Assert.assertEquals(expectedUW, actualUW, 1e-14);
        Assert.assertEquals(expectedVW, actualVW, 1e-14);
    }

    /**
     * Test {@link AngularDirection#transformReference(Frame, AbsoluteDate, AngleType)} using the  
     * reference coordinates from Vallado et al. "Implementation Issues Surrounding the New IAU 
     * Reference Systems for Astrodynamics". 
     */
    @Test
    public void testTransformReference1() {

        // Frames
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame eci = FramesFactory.getGCRF();
        AbsoluteDate date = new AbsoluteDate(2004, 4, 6, 7, 51, 28.386009, 
                                            TimeScalesFactory.getUTC());

        // Input
        Vector3D posEcef = new Vector3D(-1033.4793830*1e3, 7901.2952754*1e3, 6380.3565958*1e3);
        AngularDirection lonlat = 
            new AngularDirection(ecef, new double[]{posEcef.getAlpha(), posEcef.getDelta()}, 
                                 AngleType.LONLAT, posEcef.getNorm());
        Vector3D posEci = new Vector3D(5102.5089530*1e3, 6123.0113955*1e3, 6378.1369371*1e3);
        AngularDirection radec = 
            new AngularDirection(eci, new double[]{posEci.getAlpha(), posEci.getDelta()}, 
                                 AngleType.RADEC, posEci.getNorm());

        // Transform
        AngularDirection actual = radec.transformReference(ecef, date, AngleType.LONLAT);
        Assert.assertEquals(lonlat.getFrame(), actual.getFrame());
        Assert.assertEquals(lonlat.getAngleType(), actual.getAngleType());
        Assert.assertEquals(lonlat.getAngle1(), actual.getAngle1(), 1e-7);
        Assert.assertEquals(lonlat.getAngle2(), actual.getAngle2(), 1e-7);

        actual = lonlat.transformReference(eci, date, AngleType.RADEC);
        Assert.assertEquals(radec.getFrame(), actual.getFrame());
        Assert.assertEquals(radec.getAngleType(), actual.getAngleType());
        Assert.assertEquals(radec.getAngle1(), actual.getAngle1(), 1e-7);
        Assert.assertEquals(radec.getAngle2(), actual.getAngle2(), 1e-7);
    }

    /**
     * Test {@link AngularDirection#transformReference(Frame, AbsoluteDate, AngleType)} using the  
     * reference angular position from Vallado's Example 4-1 in "Fundamentals of Astrodynamics and 
     * Applications". 
     */
    @Test
    public void testTransformReference2() {

        // Frames
        Frame eci = FramesFactory.getGCRF();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),     // Geodetic latitude
                                              FastMath.toRadians(-104.883),     // Longitude
                                     2194.56);                         // Altitude in [m]
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, pos, "Topocentric Horizon");

        // Input
        AbsoluteDate date = 
            new AbsoluteDate(1994, 5, 14, 13, 11, 20.59856, TimeScalesFactory.getUTC());
        double[] radecAngles = 
            new double[]{FastMath.toRadians(294.9891458), FastMath.toRadians(-20.8234944)};
        double range = 4437725220.273 * 1e3;
        AngularDirection radec = new AngularDirection(eci, radecAngles, AngleType.RADEC, range);

        // Expected data wrt Vallado's definition of topocentric horizon frame
        double[] expectedVallado = new double[]{FastMath.toRadians(210.8250667), 
                                                FastMath.toRadians(23.8595052)};
        double[] expectedOrekit = new double[]{-(expectedVallado[0]-2*FastMath.PI) + FastMath.PI/2,
                                               expectedVallado[1]};
        AngularDirection actualOrekit = radec.transformReference(topoHorizon, date, AngleType.AZEL);

        // Compare
        double tolerance = 1e-5;
        Assert.assertEquals(expectedOrekit[0], actualOrekit.getAngle1(), tolerance);
        Assert.assertEquals(expectedOrekit[1], actualOrekit.getAngle2(), tolerance);
    }

    /**
     * Test {@link AngularDirection#transformReference(Frame, AbsoluteDate, AngleType)} using the  
     * reference angular position from Vallado's Example 4-1 in "Fundamentals of Astrodynamics and 
     * Applications". The goal is transform an angular direction from geocentric inertial frame
     * into topocentric inertial frame.
     */
    @Test
    public void testTransformReference3() {
        AbsoluteDate date = 
            new AbsoluteDate(1994, 5, 14, 13, 11, 20.59856, TimeScalesFactory.getUTC());
        double geoDist = Constants.WGS84_EARTH_EQUATORIAL_RADIUS + 35786 * 1e3;
        Vector3D posGeoCentric = new Vector3D(geoDist, new Vector3D(FastMath.toRadians(33), 0.));
        Assert.assertEquals(35361820.72, posGeoCentric.getX(), 0.01);
        Assert.assertEquals(22964234.89, posGeoCentric.getY(), 0.01);
        Assert.assertEquals(0., posGeoCentric.getZ(), 1e-16);

        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),     // Geodetic latitude
                                              FastMath.toRadians(-104.883),     // Longitude
                                              2194.56);                         // Altitude in [m]
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, pos, "Topocentric Horizon");
        Frame eci = FramesFactory.getGCRF();
        Transform topoToEci = topoHorizon.getTransformTo(eci, date);
        Vector3D siteGeoCentric = topoToEci.transformPosition(Vector3D.ZERO);
        Assert.assertEquals(4068213.205, siteGeoCentric.getX(), 1e3);
        Assert.assertEquals(-2842429.051, siteGeoCentric.getY(), 1e3);
        Assert.assertEquals(3996349.956, siteGeoCentric.getZ(), 1e3);

        Transform eciToTopoInertial = new Transform(date, siteGeoCentric.negate());
        Frame topoInertial = new Frame(eci, eciToTopoInertial, "Topocentric", true);

        Vector3D posTopo = posGeoCentric.subtract(siteGeoCentric);
        AngularDirection dirGeoCentric = 
            new AngularDirection(eci, 
                                 new double[]{posGeoCentric.getAlpha(), posGeoCentric.getDelta()}, 
                                 AngleType.RADEC, posGeoCentric.getNorm());
        AngularDirection dirTopoInertial = 
            dirGeoCentric.transformReference(topoInertial, date, AngleType.RADEC);
        Assert.assertEquals(posTopo.getAlpha(), dirTopoInertial.getAngle1(), 1e-16);
        Assert.assertEquals(posTopo.getDelta(), dirTopoInertial.getAngle2(), 1e-16);
    }

    /**
     * Test {@link AngularDirection#computeAngularDistMoon(AbsoluteDate, Frame, AngularDirection)}
     * using the reference angular position of Moon in Vallado's Example 5-3 in "Fundamentals of 
     * Astrodynamics and Applications".
     */
    @Test
    public void testComputeAngularDistMoon() {

        // Input
        AbsoluteDate date = new AbsoluteDate(1994, 4, 28, 0, 0, 0, TimeScalesFactory.getUTC());
        Frame eci = FramesFactory.getGCRF();

        // Test 1
        double[] moonAngles = 
            new double[]{FastMath.toRadians(246.691103), FastMath.toRadians(-20.477702)};
        double moonRange = 362144.6075 * 1e3;
        AngularDirection pos = new AngularDirection(eci, moonAngles, AngleType.RADEC, moonRange);
        double actual = AngularDirection.computeAngularDistMoon(date, eci, pos);
        Assert.assertEquals(0., FastMath.toDegrees(actual), 0.1);

        // Test 2
        moonAngles = new double[]{moonAngles[0] - FastMath.PI, -moonAngles[1]};
        pos = new AngularDirection(eci, moonAngles, AngleType.RADEC, moonRange);
        actual = AngularDirection.computeAngularDistMoon(date, eci, pos);
        Assert.assertEquals(180., FastMath.toDegrees(actual), 0.1);
    }



}
