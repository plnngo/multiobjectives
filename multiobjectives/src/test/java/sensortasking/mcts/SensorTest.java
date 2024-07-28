package sensortasking.mcts;

import java.io.File;
import java.util.InputMismatchException;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.junit.Assert;
import org.junit.Before;
import org.junit.Test;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.StaticTransform;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

public class SensorTest {

    /** Test sensor according to Vallado Example 4-1. */
    Sensor sensor;

    /** Epoch related to frame transformation. */
    AbsoluteDate date;

    /** Test sensor according to Vallado Example 7-1. */
    //Sensor sensor2;

    /** Epoch related to frame transformation. */
    //AbsoluteDate date2;

    @Before
    public void init() {

        // Load orekit data
        File orekitData = new File("C:/Users/plnngo/Documents/Programs/orekit/orekit-data");
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Settings in terms of duration according to Frueh
        double exposureT = 8.;      //in [s]
        double readoutT = 7.;       //in [s]
        double slewT = 9.;          //in [s]
        double settlingT = readoutT;//in [s]

        // Settings of sensor     
        Fov fov = new Fov(Fov.Type.RECTANGULAR, FastMath.toRadians(0.59), FastMath.toRadians(0.73));
        double slewVel = fov.getHeight()/slewT;
        double cutOff = FastMath.toRadians(5.);

        // Location at United States Air Force Academy according to Vallado Example 4-1
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(39.007),   // Geodetic latitude
                                              FastMath.toRadians(-104.883),   // Longitude
                                              2194.56);              // in [m]

        sensor = new Sensor("United States Air Force Academy", fov, pos, exposureT, readoutT,
                                 slewVel, cutOff);
        date = new AbsoluteDate(1994, 5, 14, 13, 11, 20.59856, 
                                            TimeScalesFactory.getUTC());

        /* // Another sensor configured according to https://naif.jpl.nasa.gov/pub/naif/toolkit_docs/MATLAB/mice/cspice_sxform.html 
        GeodeticPoint pos2 = new GeodeticPoint(FastMath.toRadians(34.05),   // Geodetic latitude
                                              FastMath.toRadians(118.25),   // Longitude
                                              0.);              // in [m]
        sensor2 = new Sensor("Another sensor", fov, pos2, exposureT, readoutT, slewVel, cutOff);
        date2 = new AbsoluteDate(1990, 2, 1, 0, 0, 0., TimeScalesFactory.getUTC()); 

        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        OneAxisEllipsoid oblateEarth = 
            new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 Constants.WGS84_EARTH_FLATTENING,
                                 ecef);
        date2 = new AbsoluteDate(2004, 4, 6, 7, 51, 28.386009, 
                                            TimeScalesFactory.getUTC());
        Vector3D posItrf = new Vector3D(-1033.4793830*1e3, 7901.2952754*1e3, 6380.3565958*1e3);
        GeodeticPoint pos2 = oblateEarth.transform(posItrf, ecef, date);
        sensor2 = new Sensor("Another sensor", fov, pos2, exposureT, readoutT, slewVel, cutOff);*/
    }

    /**
     * Test {@link Sensor#getTopoInertialFrame(AbsoluteDate)}. To test the correctness of the
     * frame transformation, the position of the sensor in the sensor frame is converted into ECEF
     * and crossed checked with the expected value given by Vallado's Example 4-1 in
     * "Fundamentals of Astrodynamics and Applications".
     */
    @Test
    public void testGetTopoInertialFrame(){
        Frame actual = sensor.getTopoInertialFrame(date);
        Vector3D centerTopo = Vector3D.ZERO;

        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        StaticTransform topo2ecef = actual.getTransformTo(ecef, date);
        Vector3D actualCenterEcef = topo2ecef.transformPosition(centerTopo);

        Vector3D expectedCenterEcef = new Vector3D(-1275.123419*1e3, 
                                                   -4797.994704*1e3, 
                                                   3994.302210*1e3);

        System.out.println(sensor.getSensorPosEcef());
        double tolerance = 1e-3;
        Assert.assertEquals(expectedCenterEcef.getX(), actualCenterEcef.getX(), tolerance);
        Assert.assertEquals(expectedCenterEcef.getY(), actualCenterEcef.getY(), tolerance);
        Assert.assertEquals(expectedCenterEcef.getZ(), actualCenterEcef.getZ(), tolerance);
    }

    /**
     * Test {@link Sensor#computeRepositionT(AngularDirection, AngularDirection)}. The pointing 
     * driections are defined in an inertial frame and the computed angle between the two directions
     * is compared with the results using orekit functions.
     */
    @Test
    public void testComputeRepositionT1(){

        // Input
        Frame gcrf = FramesFactory.getGCRF();
        double[] radecOrigin = new double[]{FastMath.toRadians(90.), FastMath.toRadians(60.)};
        double[] radecDest = new double[]{FastMath.toRadians(45.), FastMath.toRadians(15.)};

        AngularDirection origin = new AngularDirection(gcrf, radecOrigin, AngleType.RADEC);
        AngularDirection dest = new AngularDirection(gcrf, radecDest, AngleType.RADEC);

        double actual = 
            sensor.computeRepositionT(origin, dest, sensor.isSlewVelInclSensorSettle());

        // Orekit computation
        Vector3D originVec = new Vector3D(radecOrigin[0], radecOrigin[1]);
        Vector3D destVec = new Vector3D(radecDest[0], radecDest[1]);
        double angle = Vector3D.angle(originVec, destVec);
        double expectedOrekit = angle/this.sensor.getSlewVel();

        Assert.assertEquals(expectedOrekit, actual, 1e-12);
    }

    /**
     * Test {@link Sensor#computeRepositionT(AngularDirection, AngularDirection)}. The pointing 
     * driections are defined in the topocentric horizon frame and the computed angle between the 
     * two directions is compared with the results using orekit functions.
     */
    @Test
    public void testComputeRepositionT2(){

        // Build up topocentric horizon frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        OneAxisEllipsoid oblateEarth = 
            new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 Constants.WGS84_EARTH_FLATTENING,
                                 ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(oblateEarth, sensor.getPosition(), 
                                                    "Topocentric Horizon");

        // Input
        double[] azelOrigin = new double[]{FastMath.toRadians(45.), FastMath.toRadians(15.)};
        double[] azelDest = new double[]{FastMath.toRadians(-160.), FastMath.toRadians(10.)};
        AngularDirection origin = new AngularDirection(topoHorizon, azelOrigin, AngleType.AZEL);
        AngularDirection dest = new AngularDirection(topoHorizon, azelDest, AngleType.AZEL);
        double actual = 
            sensor.computeRepositionT(origin, dest, sensor.isSlewVelInclSensorSettle());

        // Orekit computation
        Vector3D originVec = new Vector3D(azelOrigin[0], azelOrigin[1]);
        Vector3D destVec = new Vector3D(azelDest[0], azelDest[1]);
        double angle = Vector3D.angle(originVec, destVec);
        double expectedOrekit = angle/this.sensor.getSlewVel();

        Assert.assertEquals(expectedOrekit, actual, 1e-12);
    }

    /**
     * Test {@link Sensor#computeRepositionT(AngularDirection, AngularDirection)}. The pointing 
     * driections are defined in two different frames which is why it is expected that the tested 
     * function throws an {@link InputMismatchException}. 
     */
    @Test(expected = InputMismatchException.class)
    public void testComputeRepositionTErrorFrameMissMatch(){

        // Build up frames
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame eci = FramesFactory.getEME2000();

         // Input
        double[] latlonOrigin = new double[]{FastMath.toRadians(0.), FastMath.toRadians(5.)};
        double[] radecDest = new double[]{FastMath.toRadians(-180.), FastMath.toRadians(10.)};
        AngularDirection origin = new AngularDirection(ecef, latlonOrigin, AngleType.LONLAT);
        AngularDirection dest = new AngularDirection(eci, radecDest, AngleType.RADEC);

        sensor.computeRepositionT(origin, dest, sensor.isSlewVelInclSensorSettle());
    }

    /**
     * Test {@link Sensor#getSensorPosEci(AbsoluteDate)} using the reference coordinates from 
     * Vallado et al. "Implementation Issues Surrounding the New IAU Reference Systems for 
     * Astrodynamics". 
     */
    @Test
    public void testGetSensorPosEci(){

        // Expected position in ECI
        Vector3D expected = new Vector3D(5102.5089530*1e3, 6123.0113955*1e3, 6378.1369371*1e3);

        // Set up ECEF frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        OneAxisEllipsoid oblateEarth = 
            new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 Constants.WGS84_EARTH_FLATTENING,
                                 ecef);
        AbsoluteDate date = new AbsoluteDate(2004, 4, 6, 7, 51, 28.386009, 
                                            TimeScalesFactory.getUTC());
        Vector3D posEcef = new Vector3D(-1033.4793830*1e3, 7901.2952754*1e3, 6380.3565958*1e3);
        GeodeticPoint pos = oblateEarth.transform(posEcef, ecef, date);

        Sensor sensor2 = new Sensor("Another sensor", sensor.getFov(), pos, sensor.getExposureT(), 
                                    sensor.getReadoutT(), sensor.getSlewVel(), 
                                    sensor.getElevCutOff());
        
        Vector3D actual = sensor2.getSensorPosEci(date);

        double tolerance = 1.;
        Assert.assertEquals(expected.getX(), actual.getX(), tolerance);
        Assert.assertEquals(expected.getY(), actual.getY(), tolerance);
        Assert.assertEquals(expected.getZ(), actual.getZ(), tolerance);

        checkEcef2GeodeticCoordinates();        
    }

    /**
     * In order to guarantee the correct functionality of 
     * {@link Sensor#getSensorPosEci(AbsoluteDate)} it is necessary to ensure that the 
     * transformation of a Cartesian position in ECEF into geodetic coordinates is performed
     * correcrtly. The values used for testing were extracted from Vallado's Example 4-1 in
     * "Fundamentals of Astrodynamics and Applications".
     */
    private void checkEcef2GeodeticCoordinates() {

        // Position of the sensor according to Vallado Example 4-1
        Vector3D posEcef = new Vector3D(-1275.123419*1e3, -4797.994704*1e3, 3994.302210*1e3);

        // Set up ECEF frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        OneAxisEllipsoid oblateEarth = 
            new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 Constants.WGS84_EARTH_FLATTENING,
                                 ecef);

        GeodeticPoint actual = oblateEarth.transform(posEcef, ecef, date);
        GeodeticPoint expected = sensor.getPosition();
        Assert.assertEquals(expected.getAltitude(), actual.getAltitude(), 1e-2);
        Assert.assertEquals(FastMath.toDegrees(expected.getLatitude()), 
                            FastMath.toDegrees(actual.getLatitude()), 1e-3);
        Assert.assertEquals(FastMath.toDegrees(expected.getLongitude()), 
                            FastMath.toDegrees(actual.getLongitude()), 1e-3);
    }

    /**
     * Test {@link Sensor#getSensorPosEcef(AbsoluteDate)} using values from Vallado's Example 4-1.
     * As addition test, the result is also compared with the values given by orekit using
     * {@link org.orekit.bodies.OneAxisEllipsoid#transform(GeodeticPoint)}. 
     */
    @Test
    public void testGetSensorPosEcef(){

        // Position of the sensor according to Vallado Example 4-1
        Vector3D expected = new Vector3D(-1275.123419*1e3, -4797.994704*1e3, 3994.302210*1e3);
        Vector3D actual = sensor.getSensorPosEcef();

        // Set up ECEF frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        OneAxisEllipsoid oblateEarth = 
            new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 Constants.WGS84_EARTH_FLATTENING,
                                 ecef);

        Vector3D expectedOrekit = oblateEarth.transform(sensor.getPosition());

        double tolerance = 1e-3;
        Assert.assertEquals(expected.getX(), actual.getX(), tolerance);
        Assert.assertEquals(expected.getY(), actual.getY(), tolerance);
        Assert.assertEquals(expected.getZ(), actual.getZ(), tolerance);

        // Check accordance with orekit implememtaion
        Assert.assertEquals(expected.getX(), expectedOrekit.getX(), tolerance);
        Assert.assertEquals(expected.getY(), expectedOrekit.getY(), tolerance);
        Assert.assertEquals(expected.getZ(), expectedOrekit.getZ(), tolerance);
    }

    @Test
    public void testMapSpacecraftStateToFieldOfRegard(){
        TLE test = new TLE("1 25338U 98030A   24102.60546611  .00000200  00000-0  10064-3 0  9995", 
                           "2 25338  98.5757 131.1050 0010951 153.9296 206.2438 14.26531185347696");

        TLEPropagator propagator = TLEPropagator.selectExtrapolator(test);
        //AbsoluteDate date = new AbsoluteDate(2024, 4, 12, 2, 29, 35, TimeScalesFactory.getUTC());
        AbsoluteDate date = new AbsoluteDate(2024, 4, 12, 2, 29, 30, TimeScalesFactory.getUTC());
        SpacecraftState finalState = propagator.propagate(date);       // state in TEME
        AngularDirection actualAzEl = this.sensor.mapSpacecraftStateToFieldOfRegard(finalState, date);
        
        // Expected data extracted from https://www.n2yo.com/passes/?s=25338&a=1#
        // Definition of horizon frame: +x --> north; +y --> East
        double azimuth = 0.5*FastMath.PI - actualAzEl.getAngle1();
        if (azimuth<0.) {
            azimuth += 2*FastMath.PI;
        }
        Assert.assertEquals(259.99, FastMath.toDegrees(azimuth), 
                            FastMath.toDegrees(this.sensor.getFov().getWidth()));
        Assert.assertEquals(42.33, FastMath.toDegrees(actualAzEl.getAngle2()), 
                            FastMath.toDegrees(this.sensor.getFov().getHeight()));
    }
}
