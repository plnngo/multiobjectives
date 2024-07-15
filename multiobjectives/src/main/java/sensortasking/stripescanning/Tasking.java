package sensortasking.stripescanning;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.InputMismatchException;
import java.util.List;
import java.util.Objects;

import org.apache.commons.lang3.ArrayUtils;
import org.hipparchus.geometry.euclidean.threed.Line;
import org.hipparchus.geometry.euclidean.threed.Plane;
import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

import lombok.Getter;
import sensortasking.mcts.AngleType;
import sensortasking.mcts.AngularDirection;
import sensortasking.mcts.Sensor;

@Getter
public class Tasking {

    /** Observing sensor. */
    private Sensor sensor;

    /** Scanning stripes. */
    Stripe[] scanStripes;

    /** Schedule. */
    List<Slot> schedule;

    /** Start epoch of experiment or observation. */
    private AbsoluteDate start;

    /** End epoch of experiment or observation. */
    private AbsoluteDate end;

    /** Number of exposures of one field. */
    final int numExpos;

    /** Minimal modified solar phase angle (MSPA) in [rad] to ensure good visibility. The MSPA is 
     * formed by the space object and the sun from the perspective of the observer*/
    final static double minSolarPhase = FastMath.toRadians(40.);    // TODO: change back to value according to siminski

    /** Minimal angular distance between Moon and observation stripe. */
    double minMoonDist = FastMath.toRadians(20.);   // TODO: change back to 90 deg

    /** Sun's maximal elevation angle according to nautical twilight condition. */
    double nauticalTwilight = FastMath.toRadians(-12);

    /** Tasked schedule of the previous night. */
    Tasking prevNight;

    /** Marker indicating that re-observation stripe corresponding to first fixed stripe could not 
     * be found. */
    boolean firstReObsNotFound = false;

    /** Geostationary distance to Earth's center in [m]*/
    //final static double geoRadius = 42164*1e3;
    final static double geoRadius = 42241*1e3;

    // TODO think of tolerance 
    final static double tolerance = 1e-5;

    final double intvLaplace = FastMath.toRadians(7.5);

    /**Maxmimal multiple j related to the position of the re-observation stripe. */
    //final int maxIterations = 5;
    final int maxIterations = 30;

    /** Geocentric Celestial Reference Frame. */
    final Frame eci = FramesFactory.getGCRF();

    /** Sphere around Earth with radius same as of a GEO. */
    final OneAxisEllipsoid geoSphere = new OneAxisEllipsoid(geoRadius, 0., eci);

    double cycleT;

    /**
     * Simple constructor.
     * 
     * @param sensor            Observing sensor.
     * @param startExperiment   Start epoch of the experiment, optimally at the start of the night.
     * @param endExperiment     End epoch of the experiment, optimally at the end of the night.
     * @param numExpos          Number of exposures of one field.
     */
    public Tasking(Sensor sensor, AbsoluteDate startExperiment, AbsoluteDate endExperiment, 
                   int numExpos){

        this.sensor = sensor;
        this.start = startExperiment;
        this.end = endExperiment;
        this.numExpos = numExpos;
    }
    
    public List<Slot> createSchedule() throws Exception {

        // Substract daytime (no observation possible) from experiment interval
        boolean duringNauticalTwilight = false;
        AbsoluteDate startObs = this.start;

        // Set up topocentric horizon reference frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                            Constants.WGS84_EARTH_FLATTENING,
                                            ecef);
        TopocentricFrame topoHorizon = 
            new TopocentricFrame(earth, sensor.getPosition(), sensor.getName());
        do {
            CelestialBody sun = CelestialBodyFactory.getSun();
            PVCoordinates sunPos = sun.getPVCoordinates(startObs, topoHorizon);
            double sunElev = sunPos.getPosition().getDelta();
            duringNauticalTwilight = sunElev < nauticalTwilight;
            if(!duringNauticalTwilight) {
                startObs = startObs.shiftedBy(1.);
            } else{
                duringNauticalTwilight = true;
                this.start = startObs;
            }
        } while(!duringNauticalTwilight);

        Stripe[] scanStripes = computeScanStripes();
        Stripe[] scanStripesCopy = new Stripe[]{scanStripes[0], scanStripes[2]};
        this.scanStripes = scanStripesCopy;
        computeObsSlots(this.scanStripes);
        return getSchedule();
    }

    /**
     * Compute two fixed and two re-observation stripes.
     * 
     * @param duration
     * @param scanStripes
     * 
     * @return                  Set of fixed and re-observation stripes. First half contains fixed,
     *                          second half the related re-obs stripes.
     */
    public Stripe[] computeScanStripes() {

        // Set up Earth
        OneAxisEllipsoid oblateEarth = 
            new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                 Constants.WGS84_EARTH_FLATTENING,
                                 FramesFactory.getGCRF());                 

        int numDecFields = computeNumberDeclinationFieldsLeakproof();
        //int numDecFields = computeNumberDeclinationFields();


        Stripe[] fixedRa = computeFixedRAs(oblateEarth, numDecFields);
        Stripe[] reobsRa = computeReObsRAs(oblateEarth, fixedRa, numDecFields);
        //Stripe[] reobsRa = computeReObsRAsSameDec(oblateEarth, fixedRa, numDecFields);


        // Scan stripes shall only contain non-null stripes
        List<Stripe> nonNullReObsStripes = new ArrayList<Stripe>();
        for(int i=0; i<reobsRa.length; i++) {
            if(!Objects.isNull(reobsRa[i])) {
                nonNullReObsStripes.add(reobsRa[i]);
            } else if(i==0) {

                // Marker to identify which re-observation stripe could not be found
                this.firstReObsNotFound = true;
            }
        }
        Stripe[] allStripes = 
            ArrayUtils.addAll(fixedRa, nonNullReObsStripes.toArray(new Stripe[0]));
        this.scanStripes = allStripes;

        return allStripes;
    }

    protected Stripe[] computeReObsRAs(OneAxisEllipsoid oblateEarth, Stripe[] fixedRa, 
                                     int numDecFields) {

        // Create geosynchronous sphere in eci
        Frame eci = FramesFactory.getGCRF();
        OneAxisEllipsoid geoSphere = new OneAxisEllipsoid(geoRadius, 0., eci);

        // multiple of stripe duration 
        int j = 1;  
        
        Stripe[] reObs = new Stripe[2];
        
        // Compute next time ready for an observation of new stripe
        
       
/*         double shiftT = fixedRa[0].getStripeT(this.numExpos)
                                + reposLast2FirstField + this.sensor.getSettlingT(); */
        //shiftT = 75.*60.;
        //double shiftAngle = (1./43200.) * FastMath.PI * shiftT;      //in [rad]

        // TODO: think of implementation of galactic plane
        //CelestialBody sun = CelestialBodyFactory.getSun();

        double[] reObsRa = new double[]{fixedRa[0].getFirstPosField().getAngle1(), 
                                        fixedRa[1].getFirstPosField().getAngle1()};
        for (int i=0; i<fixedRa.length; i++) {
            
            double reposLast2FirstField = 
                this.sensor.computeRepositionT(fixedRa[i].getPosField(fixedRa[i].getNumDecFields()-1), 
                                            fixedRa[i].getFirstPosField());
            // derived with pythagoras (factor 1.5 shall compensate for longer paths)
            double bufferForRepos = 1.5 * reposLast2FirstField * FastMath.sqrt(2);      
            double shiftT = fixedRa[i].getStripeT(this.numExpos) 
                            + bufferForRepos
                            //+ reposLast2FirstField 
                            + this.sensor.getSettlingT();

            this.cycleT = shiftT;

            double shiftAngle = (1./43200.) * FastMath.PI * (shiftT);

            double reObsShift;
            AngularDirection midStripePos;
            AbsoluteDate startReObs;
            boolean visible = false;
            do {
                startReObs = this.start.shiftedBy(j * shiftT);

                // Compute station's position in ECI
                Vector3D stationEci = this.sensor.getSensorPosEci(startReObs);

                // Shift right ascension
                reObsShift = reObsRa[i] + j*shiftAngle; 
                if(reObsShift>2*FastMath.PI) {
                    reObsShift -= 2*FastMath.PI;
                }

                // Create Laplace plane in topocentric inertial frame
                Plane laplace = createLaplacePlane(eci, startReObs);
                //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);
                midStripePos = setStripeMidPoint(reObsShift, stationEci, 
                                                 geoSphere, laplace, startReObs);

                try {
                    visible = checkVisibility(reObsShift, stationEci, numDecFields, j, geoSphere, laplace, startReObs);
                    
                } catch (IllegalArgumentException e) {
                    // TODO: no re-obs stripe found
                    reObs[i] = null;
                }          

                /* midStripePos = setStripeMidPoint(reObsShift, stationEci, 
                                                 geoSphere, laplace, startReObs); // TODO: check date
                //System.out.println("Mid strip position computed: " + FastMath.toDegrees(midStripePos.getAngle1()) + ", " + FastMath.toDegrees(midStripePos.getAngle2()));
                
                angularDist = 
                    AngularDirection.computeAngularDistMoon(this.start, eci, midStripePos);
                
                // Compute Earth shadow boundaries
                double[] earthShadow = getEarthShadowBoundariesRAs(sun, oblateEarth, startReObs);

                // Transform angular range from [-pi, pi] to [0,2pi]
                for(int ii=0; ii<earthShadow.length; ii++) {
                    if(earthShadow[ii] < 0) {
                        earthShadow[ii] += 2*FastMath.PI;
                    }
                } */
                //inEarthShadow = checkInEarthShadow(midStripePos, startReObs);
                j++;
            } while(j < maxIterations  && !visible);

            if(visible){
                //reObsRa[i] = reObsShift;
 /*                
                // Check solar phase angle
                boolean optimalSolarPhaseAngle = checkSolarPhaseCondition(startReObs, midStripePos);
                if(!optimalSolarPhaseAngle) {
                    // TODO: no re-obs stripe found
                    reObs[i] = null;
                } */
                
                // Set up re-obs stripe
                reObs[i] = Stripe.setStripe(numDecFields, eci, sensor, midStripePos, j-1);
            }
            // Reset for next re-obs stripe
            j = 1; 
        }
        return reObs;
    }

    protected Stripe[] computeReObsRAsSameDec(OneAxisEllipsoid oblateEarth, Stripe[] fixedRa, 
                                              int numDecFields) {

        // Create geosynchronous sphere in eci
        Frame eci = FramesFactory.getGCRF();
        OneAxisEllipsoid geoSphere = new OneAxisEllipsoid(geoRadius, 0., eci);

        // multiple of stripe duration 
        int j = 1;  
        
        Stripe[] reObs = new Stripe[2];
        
        // Compute next time ready for an observation of new stripe
        double reposLast2FirstField = 
            this.sensor.computeRepositionT(fixedRa[0].getPosField(fixedRa[0].getNumDecFields()-1), 
                                           fixedRa[1].getFirstPosField());      

        double shiftT = fixedRa[0].getStripeT(this.numExpos)
                                + reposLast2FirstField + this.sensor.getSettlingT();
        //shiftT = 75.*60.;
        double shiftAngle = (1./43200.) * FastMath.PI * shiftT;      //in [rad]

        // TODO: think of implementation of galactic plane
        //CelestialBody sun = CelestialBodyFactory.getSun();

        double[] reObsRa = new double[]{fixedRa[0].getFirstPosField().getAngle1(), 
                                        fixedRa[1].getFirstPosField().getAngle1()};
        for (int i=0; i<fixedRa.length; i++) {
            double reObsShift;
            AngularDirection midStripePos;
            AbsoluteDate startReObs;
            boolean visible = false;
            do {
                startReObs = this.start.shiftedBy(j * shiftT);

                // Compute station's position in ECI
                Vector3D stationEci = this.sensor.getSensorPosEci(startReObs);

                // Shift right ascension
                reObsShift = reObsRa[i] + j*shiftAngle; 
                if(reObsShift>2*FastMath.PI) {
                    reObsShift -= 2*FastMath.PI;
                }

                // Create Laplace plane in topocentric inertial frame
                Plane laplace = createLaplacePlane(eci, startReObs);
                //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);
                double[] midAngles = 
                    new double[]{reObsShift, fixedRa[i].getPosField(numDecFields/2).getAngle2()};
                midStripePos = new AngularDirection(eci, midAngles, AngleType.RADEC);

                try {
                    visible = checkVisibility(reObsShift, stationEci, numDecFields, j, geoSphere, laplace, startReObs);
                    
                } catch (IllegalArgumentException e) {
                    // TODO: no re-obs stripe found
                    reObs[i] = null;
                }          

                
                j++;
            } while(j < maxIterations  && !visible);

            if(visible){                
                // Set up re-obs stripe
                reObs[i] = Stripe.setStripe(numDecFields, eci, sensor, midStripePos, j-1);
            }
            // Reset for next re-obs stripe
            j = 1; 
        }
        return reObs;
    }

    /**
     * Check if satellite is within Earth's penumbra under the assumption that the shadow spreads
     * out cylindrically behind the Earth.
     * 
     * @param toCheck           Angular position of the satellite.
     * @param date              Date to get the position of the Sun.
     * 
     * @return                  True, if satellite is in Earth's shadow, false if not.
     */
    protected static boolean checkInEarthShadow(AngularDirection toCheck, AbsoluteDate date) {

        Frame eci = FramesFactory.getGCRF();

        // Set up boundary condition parameters
        double rGeo = 42164 * 1e3;                      // GEO radius in [m] 
        double halfCone = FastMath.asin(Constants.WGS84_EARTH_EQUATORIAL_RADIUS/rGeo);
        //double halfCone = 0.15176;

        // Compute direction of Earth shadow's center
        CelestialBody sun = CelestialBodyFactory.getSun();
        PVCoordinates shadow = sun.getPVCoordinates(date, eci).negate().normalize();
        double[] shadowAngles = 
            new double[]{shadow.getPosition().getAlpha(), shadow.getPosition().getDelta()};
        AngularDirection shadowDir = new AngularDirection(eci, shadowAngles, AngleType.RADEC);

        if(toCheck.getEnclosedAngle(shadowDir) < halfCone) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Compute two fixed observation stripes.
     * 
     * @param oblateEarth       Modelling of Earth.
     * @param numDecFields      Number of declination fields within the stripe.
     * 
     * @return                  Two fixed stripes.
     * @throws IllegalArgumentException If observability and visibility conditions are violated. 
     *                                  No observation stripe to be found.
     */
    protected Stripe[] computeFixedRAs(OneAxisEllipsoid oblateEarth, int numDecFields)
                                     throws IllegalArgumentException {

        Frame eci = FramesFactory.getGCRF();
        Vector3D stationEci = this.sensor.getSensorPosEci(this.start);

        // Get all bodies and their position that affect visibility condition
        CelestialBody sun = CelestialBodyFactory.getSun();

        // TODO: think of implementation of galactic plane
        // Compute position of the first fixed stripes
        double[] earthShadow = getEarthShadowBoundariesRAs(sun, oblateEarth, this.start);
        double fixedRaSmall = earthShadow[0];
        double fixedRaLarge = earthShadow[1];

        // Create Laplace plane in geocentric inertial frame
        Plane laplace = createLaplacePlane(eci, this.start);
        //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);

        // Create geosynchronous sphere in eci
        OneAxisEllipsoid geoSphere = new OneAxisEllipsoid(geoRadius, 0., eci);

        double sensorSize =  sensor.getFov().getWidth();
                                           
        // Deal with smaller earth shadow boundary        
        final int j = 0;      // marker for fixed stripes
        boolean visible = checkVisibility(fixedRaSmall, stationEci, numDecFields, j, geoSphere, laplace, this.start);          
        while(!visible) {

            // Shift stripe by one FOV away from earth shadow boundary
            // Check in which angular quadrant the fixed Ra are
            if(fixedRaSmall>0. && fixedRaSmall<FastMath.toRadians(90.) && fixedRaLarge>FastMath.toRadians(270.)) {
                fixedRaSmall += sensorSize;
            } else {
                fixedRaSmall -= sensorSize;
            }
            visible = checkVisibility(fixedRaSmall, stationEci, numDecFields, j, geoSphere, laplace, this.start);
        } 

        AngularDirection midStripeSmall = setStripeMidPoint(fixedRaSmall, stationEci, 
                                                            geoSphere, laplace, this.start);
        Stripe fixedSmallRa = Stripe.setStripe(numDecFields, eci, sensor, midStripeSmall, 0);

        // Deal with larger earth shadow boundary
        visible = checkVisibility(fixedRaLarge, stationEci, numDecFields, j, geoSphere, laplace, this.start);    
        while(!visible) {

            // Shift stripe by one FOV away from earth shadow boundary
            // Check in which angular quadrant the fixed Ra are
            if(fixedRaSmall>0. && fixedRaSmall<FastMath.toRadians(90.) && fixedRaLarge>FastMath.toRadians(270.)) {
                fixedRaLarge -= sensorSize;
            } else {
                fixedRaLarge += sensorSize;
            }
            visible = checkVisibility(fixedRaLarge, stationEci, numDecFields, j, geoSphere, laplace, this.start);
        } 
        AngularDirection midStripeLarge = setStripeMidPoint(fixedRaLarge, stationEci, 
                                                            geoSphere, laplace, this.start);
        Stripe fixedLargeRa = Stripe.setStripe(numDecFields, eci, sensor, midStripeLarge, j);
        System.out.println(FastMath.toDegrees(fixedLargeRa.getPosField(fixedLargeRa.getNumDecFields()-1).getAngle2()));

        // Set fixed observation stripes
        Stripe[] out = new Stripe[]{fixedSmallRa, fixedLargeRa};

        return out;
    }
/* 
    private Frame setupTopocentricInertialFrame(AbsoluteDate date, GeodeticPoint sensorPos) {

        // Set up ECI frame
        Frame eci = FramesFactory.getGCRF();

        // Transform station into ECI frame
        Transform ecef2eci = ecef.getTransformTo(eci, date);
        Vector3D stationEcef = oblateEarth.transform(sensorPos);
        Vector3D stationEci = ecef2eci.transformPosition(stationEcef);     // TODO: check transformation!!!

        // Set up topocentric inertial frame
        Transform geo2topoEci = new Transform(date, stationEci);
        Frame topoInertial = new Frame(eci, geo2topoEci, "Topocentric-Inertial", true);
        return topoInertial;
    } */

    /**
     * 
     * @param ra
     * @param stationEci
     * @param numDecFields
     * @param j
     * @param geoSphere
     * @param laplace
     * @param date
     * @return
     * @throws IllegalArgumentException
     */
    private boolean checkVisibility(double ra, Vector3D stationEci, int numDecFields, int j,
                                    OneAxisEllipsoid geoSphere, Plane laplace, AbsoluteDate date) 
                                    throws IllegalArgumentException {
                                    
        Frame eci = FramesFactory.getGCRF();
        AngularDirection midStripe = 
            setStripeMidPoint(ra, stationEci, geoSphere, laplace, date);

        Stripe stripe = Stripe.setStripe(numDecFields, eci, sensor, midStripe, j);

        // Distance to Moon
        double angularDistFirst =  AngularDirection.computeAngularDistMoon(date, eci, stripe.getFirstPosField());
        double angularDistLast =  AngularDirection.computeAngularDistMoon(date, eci, stripe.getPosField(numDecFields-1));

        // Check solar phase angle
        boolean optimalSolarPhaseAngleFirst = checkSolarPhaseCondition(date, stripe.getFirstPosField());
        boolean optimalSolarPhaseAngleLast = checkSolarPhaseCondition(date, stripe.getPosField(numDecFields-1));

        // Check if in Earth's shadow (not necessary for fixed stripes)
        if(j!=0) {
            boolean inEarthShadowFirst = checkInEarthShadow(stripe.getFirstPosField(), date);
            boolean inEarthShadowLast = checkInEarthShadow(stripe.getPosField(numDecFields-1), date);
            if (inEarthShadowFirst || inEarthShadowLast) {
                return false;
            }
        }

        if(!optimalSolarPhaseAngleFirst && !optimalSolarPhaseAngleLast) {
            throw new IllegalArgumentException("Solar Phase Angle condition violated " 
                                                + "and no fixed stripe has been found");
        }

        if (angularDistFirst < minMoonDist || angularDistLast < minMoonDist) {
            return false;
        } else {
            return true;
        }
    }

    /**
     * Compute the number of declination field of a Laplace scanning stripe. The mid field of the 
     * stripe shall be centered at the Laplace plane and the number of fields above the plane is 
     * equal to the number of fields below it.
     * 
     * @return                  Number of declination fields (always odd).
     */
    public int computeNumberDeclinationFields() {

        double heightFov = this.sensor.getFov().getHeight();

        // Mid field is placed at Laplace plane, i.e. it cuts the plane into two equal halfs
        double rest = intvLaplace - 0.5*heightFov;
        double numDecFields = 1 + 2 * FastMath.ceil(rest/heightFov);

        return (int)numDecFields;
    }

    /**
     * Compute the number of declination field of a Laplace scanning stripe. The mid field of the 
     * stripe shall be centered at the Laplace plane and the number of fields above the plane is 
     * equal to the number of fields below it. The size of the stripe is adapted to the width of 
     * the FOV to guarantee leakproofness within the stripe.
     * 
     * @return                  Number of declination fields (always odd).
     */
    public int computeNumberDeclinationFieldsLeakproof() {

        Sensor sensor = this.sensor;
        double heightFov = sensor.getFov().getHeight();
        double widthFov = sensor.getFov().getWidth();

        // Compute leakproof stripe scanning duration
        double rotRateGeo = 86400/(2*FastMath.PI);      // in [s/rad]
        double leakproofStripeT = widthFov * rotRateGeo;

        // Compute time to reposition after one declination field scan. Assume no overlap.
        AngularDirection origin = new AngularDirection(eci, new double[]{0., 0.}, AngleType.RADEC);
        AngularDirection dest = 
            new AngularDirection(eci, new double[]{0., heightFov}, AngleType.RADEC);
        double reposT = sensor.computeRepositionT(origin, dest);

        // Assigne time durations as defined in Carolin Fruehs paper
        double t1 = 0.;
        double t2 = sensor.getReadoutT();
        if(reposT>sensor.getReadoutT()) {
            t1 = reposT;
        } else {
            t1 = sensor.getReadoutT();
        }

        int numDecFields = 
            (int)((leakproofStripeT + t1 - t2)
                /(numExpos*sensor.getExposureT() + (numExpos -1)*sensor.getReadoutT() + t1));

        // Check if t2 is assigned to read out or repositioning time (depending which is longer)
        AngularDirection destEndStripe = 
            new AngularDirection(eci, new double[]{0., heightFov * numDecFields}, AngleType.RADEC);
        double reposTBeginEndStripe = sensor.computeRepositionT(origin, destEndStripe);
        if(t2<reposTBeginEndStripe) {
            t2 = reposTBeginEndStripe;
            numDecFields = 
                (int)((leakproofStripeT + t1 - t2)
                    /(numExpos*sensor.getExposureT() + (numExpos -1)*sensor.getReadoutT() + t1));
        }
        
        return numDecFields;
    }

    /**
     * Check if the angle formed by the space object and the sun from the perspective of the 
     * frame's origin does not exceed the minimal visibility requirement in terms of the solar 
     * phase angle. This check does not guarantee visibility yet because it is still necessary 
     * to check if sensor is pointing towards Earth shadow.
     * 
     * @param date              Date related to frame transformation.
     * @param pos               Angular pointing direction.
     * 
     * @return                  true when min. modified solar phase angle is exceeded, else false.
     */
    public static boolean checkSolarPhaseCondition(AbsoluteDate date, AngularDirection pos) {
                                             
        AngularDirection sunDir = getSunDir(pos.getFrame(), date);
        double solarPhaseModified = sunDir.getEnclosedAngle(pos);
        return solarPhaseModified > minSolarPhase;
    }

    /**
     * Compute the mid stripe position with respect to an inertial frame (GCRF).
     * The mid stripe position shall lie within the Laplace plane.
     *      
     * @param ra                Right ascension of observation stripe.
     * @param stationEci        Ground station with respect to GCRF.
     * @param geoSphere         Sphere around the Earth with the same radius as a geostationary 
     *                          object.
     * @param laplace           Laplace plane in GCRF.
     * 
     * @return                  Angular position of stripe's mid point.
     */
    protected static AngularDirection setStripeMidPoint(double ra, Vector3D stationEci, 
                                                        OneAxisEllipsoid geoSphere, Plane laplace,
                                                        AbsoluteDate date) {

        Frame eci = FramesFactory.getGCRF();

        // Shift ra to [-pi,pi] interval to be able to build up the plane
        if(ra>FastMath.PI) {
            ra -= 2*FastMath.PI;
        } 
        Plane raPlane = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, 
                                    new Vector3D(ra, 0.), tolerance);
                    
        Line intersectRaLaplace = raPlane.intersection(laplace);

        GeodeticPoint midStripeGeo = 
            geoSphere.getIntersectionPoint(intersectRaLaplace, stationEci, eci, date); // TODO: check date
        if(FastMath.abs(midStripeGeo.getLongitude() - ra) > 1e-15) {
            midStripeGeo = geoSphere.getIntersectionPoint(intersectRaLaplace, 
                                                             stationEci.negate(), 
                                                             eci, date); // TODO: check date
        }
        Vector3D midStripeGeoCart = geoSphere.transform(midStripeGeo); // TODO: check if frame is ECI

        double[] angles = new double[]{midStripeGeoCart.getAlpha(), midStripeGeoCart.getDelta()};
                                        
        return new AngularDirection(eci, angles, AngleType.RADEC);
    }

    /**
     * Get right ascension of Earth shadow boundaries that do not necessarily lie on the Laplace 
     * plane with respect to an inertial reference frame GCRF.
     * 
     * @param sun               Sun as a celestrial body.
     * @param earth             Earth whose shape is defined with respect to GCRF.
     * @param date              Date related to any frame transformation and the computation of the
     *                          shadow boudaries positions.
     * 
     * @return                  Right ascension (between 0 and 2PI) position of Earth shadow 
     *                          boundaries in GCRF in ascending order.
     */
    public double[] getEarthShadowBoundariesRAs(CelestialBody sun, OneAxisEllipsoid earth,
                                                AbsoluteDate date) {

        Frame eci = FramesFactory.getGCRF();

        // Check if body shape is defined wrt GCRF
        if(eci.equals(earth.getBodyFrame())) {
            earth = new OneAxisEllipsoid(earth.getEquatorialRadius(), earth.getFlattening(), eci);
        }

        AngularDirection[] boundaries = getShadowBoundaryAngles(sun, earth, date);
        System.out.println("Earth shadow 1: " + FastMath.toDegrees(boundaries[0].getAngle1()) + ", " + FastMath.toDegrees(boundaries[0].getAngle2()));
        System.out.println("Earth shadow 2: " + FastMath.toDegrees(boundaries[1].getAngle1()) + ", " + FastMath.toDegrees(boundaries[1].getAngle2()));

        double[] boundaryRAs = new double[]{boundaries[0].getAngle1(), boundaries[1].getAngle1()};

        for(int i=0; i<boundaries.length; i++) {

            // Transform into [0, 2Pi] interval
            if(boundaryRAs[i] < 0.) {
                boundaryRAs[i] += 2*FastMath.PI;
            }
        }
        // Store right ascensions in increasing way of order
        if(boundaryRAs[1] < boundaryRAs[0]) {
            double raMax = boundaryRAs[0];
            boundaryRAs[0] = boundaryRAs[1];
            boundaryRAs[1] = raMax;
        }
        return boundaryRAs;
    }

    /**
     * Pointing direction towards Earth shadow boundaries that do not necessarily lie on the 
     * Laplace plane.
     * 
     * @param sun
     * @param earth
     * @param date
     * @return
     */
    protected static AngularDirection[] getShadowBoundaryAngles(CelestialBody sun, 
                                                                OneAxisEllipsoid earth, 
                                                                AbsoluteDate date) {

        Frame eci = FramesFactory.getGCRF();

        // Plane perpendicular to sun vector
        Vector3D sunPosEci = sun.getPVCoordinates(date, eci).getPosition();

        //Transform sunNormal plane into inertial frame
        Vector3D sunNormEci = sunPosEci.normalize();

        Plane dayNightPlane  = new Plane(Vector3D.ZERO, sunPosEci, tolerance);

        Plane laplace = createLaplacePlane(eci, date);
        //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);
        Line dayNight = dayNightPlane.intersection(laplace);

        AngularDirection[] earthShadowBoundaries = new AngularDirection[2];
    // TODO: what if sun vector aligned with I axis --> use J axis then
        Vector3D[] closePoints = new Vector3D[]{Vector3D.PLUS_I, Vector3D.MINUS_I};     
        for(int i=0; i<closePoints.length; i++) {
            GeodeticPoint boundary= 
                earth.getIntersectionPoint(dayNight, closePoints[i], eci, date);

            Vector3D boundaryOnEarthEci = earth.transform(boundary); 

            // Get boundary position in geostationary distance
            double shadowLength = FastMath.sqrt(FastMath.pow(geoRadius, 2) 
                                                + FastMath.pow(boundaryOnEarthEci.getNorm(), 2));
            Vector3D boundaryInGeoEci = boundaryOnEarthEci.subtract(shadowLength, sunNormEci);

            double[] angles = new double[]{boundaryInGeoEci.getAlpha(), 
                                           boundaryInGeoEci.getDelta()};
            earthShadowBoundaries[i] = new AngularDirection(eci, angles, AngleType.RADEC);
        }
        return earthShadowBoundaries;
    }

    /**
     * 
     * @param scanStripes       Scanning stripes with fixed right ascension.
     * 
     * @throws Exception        If number of scanning stripes is not 4.
     */
    protected void computeObsSlots(Stripe[] scanStripes) throws Exception {

        // Compute maximum possible repositioning time between stripes
        double maxReposNextStripeT = computeMaxReposNextStripeT(scanStripes);

        // Compute cycle duration
        double stripeT = scanStripes[0].getStripeT(numExpos);
        //double cycleT = stripeT + maxReposNextStripeT;

        this.schedule = new ArrayList<Slot>();

        // Set up topocentric horizon reference frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = 
            new TopocentricFrame(earth, sensor.getPosition(), sensor.getName());

        // Divide observation campaigne into time slots
        for (AbsoluteDate startSlot = start; 
             startSlot.compareTo(end) <= 0;    //Last time slot with less optimal visible condition
             startSlot = startSlot.shiftedBy(cycleT))  {

            AbsoluteDate endSlot = startSlot.shiftedBy(stripeT);

            // Check for nautical twilight condition
            AngularDirection sun = getSunDir(topoHorizon, startSlot);
            if(sun.getAngle2()<nauticalTwilight) {
                this.schedule.add(new Slot(startSlot, endSlot));
            }
        }

        // Find first observable slot
        boolean foundFirstObservableSlot = false;
        while (!foundFirstObservableSlot) {
            for (Stripe stripe : scanStripes) {
                if (StripeType.FIXED.equals(stripe.getFunctionality())) {
                    AbsoluteDate startSlot = this.schedule.get(0).getStart();

                    double ra = stripe.getFirstPosField().getAngle1();
                    Vector3D stationEci = this.sensor.getSensorPosEci(startSlot);

                    // Create geosynchronous sphere in eci
                    Frame eci = FramesFactory.getGCRF();
                    OneAxisEllipsoid geoSphere = new OneAxisEllipsoid(geoRadius, 0., eci);

                    // Create Laplace plane in topocentric inertial frame
                    Plane laplace = createLaplacePlane(eci, startSlot);
                    //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);

                    if (checkObservability(stripe, startSlot, this.sensor, numExpos) 
                        && checkLonCondition(stripe, startSlot, this.prevNight)
                        && checkVisibility(ra, stationEci, stripe.getNumDecFields(), stripe.getJ(), geoSphere, laplace, startSlot)) {

                        // Found observable stripe for the first time slot
                        foundFirstObservableSlot = true;
                        this.schedule.get(0).setStripe(stripe, numExpos);
                        break;      
                    } else {

                        // First time slot is not observable
                        this.schedule.remove(0);
                    }
                } else {
                    continue;
                }
            }
        }

        // Fill up schedule with stripes
        // TODO: How to handle more or less than four scanning stripes
/*         if (scanStripes.length != 4 && scanStripes.length != 3) {
            throw new IllegalArgumentException("Procedure for a number of scanning stripe " +
                                               "different from 3 or 4 needs yet to be implemented");
        } */

        // Find re-obs stripe corresponding to first observable time slot
        int indexCurrent = 0;
        Stripe firstScan = this.schedule.get(0).getStripe();
        int indexFixedStripe = Arrays.asList(scanStripes).indexOf(firstScan);
        addReObsSlot(indexFixedStripe, indexCurrent);   

        // Fill up rest of time slots
        indexCurrent++;

        // Number of fixed stripes that are NOT observable
        int stripeNotObservable = 0;
        while (indexCurrent<this.schedule.size()) {
            if (Objects.isNull(this.schedule.get(indexCurrent).getStripe())) {

                if (scanStripes.length==2) {
                    indexFixedStripe = 0;
                }

                else if (indexFixedStripe == 0) {
                    indexFixedStripe = 1;
                } else {
                    indexFixedStripe = 0;
                }
                // Check observability of fixed stripe in the given time interval
                Stripe obsStripe = scanStripes[indexFixedStripe];
                AbsoluteDate startSlot = this.schedule.get(indexCurrent).getStart();

                double ra = obsStripe.getFirstPosField().getAngle1();
                Vector3D stationEci = this.sensor.getSensorPosEci(startSlot);

                // Create Laplace plane in topocentric inertial frame
                Plane laplace = createLaplacePlane(eci, startSlot);
                //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);

                if(stripeNotObservable >= 2) {
                    // all fixed stripes are not observable
                    System.out.println("At " + startSlot + ", none of fixed stripes is observable. " 
                                        + "Aim for re-obs stripes instead...");
                    if(scanStripes.length == 3) {
                        obsStripe = scanStripes[scanStripes.length-1];
                    } else {
                        obsStripe = scanStripes[indexFixedStripe+2];  
                    }
                       
                    ra = obsStripe.getFirstPosField().getAngle1();
                }

                if (stripeNotObservable >= scanStripes.length) {
                    // all observation stripes are not observable, need to remove obs slot
                    this.schedule.remove(indexCurrent);
                }


                if (checkObservability(obsStripe, startSlot, this.sensor, numExpos) 
                    && checkLonCondition(obsStripe, startSlot, this.prevNight)
                    && checkVisibility(ra, stationEci, obsStripe.getNumDecFields(), obsStripe.getJ(), geoSphere, laplace, startSlot)) {
                    this.schedule.get(indexCurrent).setStripe(obsStripe, numExpos);

                    // Find re-obs stripe corresponding to given fixed stripe
                    addReObsSlot(indexFixedStripe, indexCurrent);
                    stripeNotObservable = 0;
                } else {
                    // pre-assigned fixed stripe is not visible/observable in the requested time interval
                    // try to continue to observe previous fixed stripe
                    stripeNotObservable++;
                    continue;
                }
            } 
            indexCurrent++;
        }
    }

    /**
     * Compute all possible reposition times between the scanning stripes and return the longest
     * one.
     *  
     * @param stripes           Non-null scanning stripes.
     * 
     * @return                  Maximal repositioning time of corresponding stripes.
     */
    protected double computeMaxReposNextStripeT(Stripe[] stripes) {

        double maxReposNextStripeT = 0.;

        for(int i=0; i<stripes.length; i++) {
            Stripe current = stripes[i];
            AngularDirection lastFieldCurrent = current.getPosField(current.getNumDecFields()-1);
                        
            for(int j = 0; j<stripes.length; j++) {
                if(i == j) {
                    continue;
                } else {
                    Stripe next = stripes[j];
                    AngularDirection firstFieldNext = next.getFirstPosField();
                    double reposT = sensor.computeRepositionT(lastFieldCurrent, firstFieldNext);
                    if(maxReposNextStripeT<reposT) {
                        maxReposNextStripeT = reposT;
                    }
                }
            }
        } 
        return maxReposNextStripeT;
    }

    /**
     * Schedule re-observation of the underlying fixed stripe. If observation can be 
     * performed within observation campaign interval, this function returns true and adjusts
     * the schedule accordingly, otherwise this function returns false.
     * 
     * @param indexFixedStripe  Fixed observation stripe.
     * @param indexCurrent      Index within schedule.
     * 
     * @return                  True, if r-observation has been scheduled, false otherwise.
     */
    private boolean addReObsSlot(int indexFixedStripe, int indexCurrent) {
        int indexReObsStripe;

        // Find related re-observation stripe
        if(scanStripes.length == 2 && (!firstReObsNotFound  && indexFixedStripe == 0)){
            indexReObsStripe = indexFixedStripe + 1;
        }
        else if(scanStripes.length == 4 || (!firstReObsNotFound  && indexFixedStripe == 0)) {
            indexReObsStripe = indexFixedStripe + 2;
        } else if(scanStripes.length == 3 && firstReObsNotFound && indexFixedStripe == 1) {
            // Re-observation stripe exists
            indexReObsStripe = indexFixedStripe + 1;
        } else {
            // Re-observation stripe does not exist
            return false;
        }

        Stripe reObs = scanStripes[indexReObsStripe];
        int reObsJ = reObs.getJ();
        if(indexCurrent + reObsJ < this.schedule.size()) {
            double ra = reObs.getFirstPosField().getAngle1();
            AbsoluteDate startSlot = this.schedule.get(indexCurrent + reObsJ).getStart();
            Vector3D station = this.sensor.getSensorPosEci(startSlot);
            int numDecFields =  reObs.getNumDecFields();

            // Create geosynchronous sphere in eci
            Frame eci = FramesFactory.getGCRF();
            OneAxisEllipsoid geoSphere = new OneAxisEllipsoid(geoRadius, 0., eci);

            // Create Laplace plane in topocentric inertial frame
            Plane laplace = createLaplacePlane(eci, startSlot);
            //Plane laplace = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);

                if(checkVisibility(ra, station, numDecFields, reObsJ, geoSphere, laplace, startSlot)){

                    this.schedule.get(indexCurrent + reObsJ).setStripe(reObs, numExpos);
                    return true;
                } else {
                    // Re-obs not possible because visibility is limited
                    return false;
                } 
        } else {

            // Re-obs outside observation campaigne interval
            return false;
        }
    }


    private boolean checkLonCondition(Stripe stripe, AbsoluteDate date, 
                                      Tasking prevNight) {

        if(Objects.isNull(prevNight)) {
            return true;
        }
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        AngularDirection posFirstField = stripe.getFirstPosField();
                // TODO: check if conversion is true by using values in Vallado p.173
        AngularDirection lonlatFirstPos = 
            posFirstField.transformReference(ecef, date, AngleType.LONLAT);
        double lonCurrent = lonlatFirstPos.getAngle1();

        Stripe[] scanPrev = prevNight.getScanStripes();
        for(Stripe prevScanStripe : scanPrev) {
            if(prevScanStripe.getFirstPosField().getAngle1() 
                == stripe.getFirstPosField().getAngle1()) {

                // Check if same geostationary longitude has been observed previous night
                for(Slot slot : prevNight.getSchedule()) {
                    AbsoluteDate startPre = slot.getStart();
                    Stripe stripePrev = slot.getStripe();
                    AngularDirection lonlatFirstPosPrev = 
                        stripePrev.getFirstPosField()
                                  .transformReference(ecef, startPre, AngleType.LONLAT);
                    double lonPrev = lonlatFirstPosPrev.getAngle1();

                    //TODO: think of a way to define a threshold
                    if(lonCurrent == lonPrev) {
                        return false;
                    }
                }
            }
        }
        return true;
    }

    /**
     * Check if stripe has risen over the horizon and is observable with respect to the sensor
     * (considering its cut off angle).
     * 
     * @param stripe            Observation stripe.
     * @param date              Epoch related to the transformation into horizon topocentric frame.
     * @param sensor            Observer.
     * @param int               Number of exposures.
     * 
     * @return                  True, if stripe is observable by the sensor, false otherwise.
     */
    protected static boolean checkObservability(Stripe stripe, AbsoluteDate date, Sensor sensor, 
                                             int numExpos){

        // Get lowest and highest field
        AngularDirection posFirstField = stripe.getFirstPosField();
        AngularDirection posLastField = stripe.getPosField(stripe.getNumDecFields()-1);

        // Set up topocentric az/el frame as destination frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = 
            new TopocentricFrame(earth, sensor.getPosition(), sensor.getName());

        // Convert pointing direction of first stripe into az/el horizon topocentric frame
        AngularDirection posFirstFieldTopoHorizon = 
            posFirstField.transformReference(topoHorizon, date, AngleType.AZEL);
        //System.out.println("Stripe Topo 1st field: " + FastMath.toDegrees(posFirstFieldTopoHorizon.getAngle1()) + " and " + FastMath.toDegrees(posFirstFieldTopoHorizon.getAngle2()));
        
        // Compute time when end of stripe is reached (incl. exposure of last field, too)
        double stripeT = stripe.getStripeT(numExpos);
        AbsoluteDate endOfStripe = date.shiftedBy(stripeT);

        //TODO: date might needs to get shifted so that it corresponds to the epoch when the sensor reaches the last field
        AngularDirection posLastFieldTopoHorizon = 
            posLastField.transformReference(topoHorizon, endOfStripe, AngleType.AZEL);
        if(posFirstFieldTopoHorizon.getAngle2() < sensor.getElevCutOff()) {

            return false;
        } else {

            // Compute direction of Sun wrt observer
            AngularDirection sunFirstField = getSunDir(topoHorizon, date);
            //System.out.println("Sun Topo 1st field: " + FastMath.toDegrees(sunFirstField.getAngle1()) + " and " +  FastMath.toDegrees(sunFirstField.getAngle2()));
            AngularDirection sunLastField = getSunDir(topoHorizon, endOfStripe);
            
            // Compute modified solar phase angle
            double solarPhaseModFirstField = sunFirstField.getEnclosedAngle(posFirstFieldTopoHorizon);
            double solarPhaseModLastField = sunLastField.getEnclosedAngle(posLastFieldTopoHorizon);

            if(solarPhaseModFirstField<minSolarPhase || solarPhaseModLastField<minSolarPhase) {
                return false;
            } 
        }
        return true;
    }

    /**
     * Compute Sun's angular position in requested destination frame.
     * 
     * @param frame             Destination frame.
     * @param date              Date related to frame transformation.
     * 
     * @return                  Sun's angular position in new frame.
     * @throws InputMismatchException
     */
    protected static AngularDirection getSunDir(Frame frame, AbsoluteDate date) 
        throws InputMismatchException {

        // Convert Sun's position
        CelestialBody sun = CelestialBodyFactory.getSun();
        Vector3D sunPos = sun.getPVCoordinates(date, frame).getPosition();
        double alpha = sunPos.getAlpha();     // between (-PI; PI)
        double delta = sunPos.getDelta();     // between (-PI/2; PI/2)

        // Output
        if(frame.isPseudoInertial()) {
            return new AngularDirection(frame, new double[]{alpha, delta}, AngleType.RADEC);
        } else if(frame instanceof TopocentricFrame){
            return new AngularDirection(frame, new double[]{alpha, delta}, AngleType.AZEL);
        } else if(frame.equals(FramesFactory.getITRF(IERSConventions.IERS_2010, true))){
            return new AngularDirection(frame, new double[]{alpha, delta}, AngleType.LONLAT);
        } else {
            throw new InputMismatchException("Not clear in which frame sun direction is computed");
        } 
    }

    /**
     * Create Laplace frame in the frame of interest.
     * 
     * @param dest              Frame of interest.
     * @param date              Date related to frame transformation.
     * 
     * @return                  Laplace plane in the destination frame.
     */
    public static Plane createLaplacePlane(Frame dest, AbsoluteDate date) {

        // TODO think of tolerance for laplace plane
        double tolerance = 1e-5;

        // Create ecliptic frame
        Frame ecliptic = FramesFactory.getEcliptic(IERSConventions.IERS_2010);
        Vector3D zEcliptic = new Vector3D(0, 0, 1);

        // Convert into Earth centered inertial frame
        Frame gcrf = FramesFactory.getGCRF();
        Transform ecliptic2gcrf = ecliptic.getTransformTo(gcrf, date);
        Vector3D zGcrf = new Vector3D(0, 0, 1);
        Vector3D zEclipticInGcrf = 
            ecliptic2gcrf.transformVector(zEcliptic);   // ignoring translation

        // Compute Laplace normal in GCRF
        double tiltLaplace = FastMath.toRadians(7.5);
        Vector3D rotAxis = Vector3D.crossProduct(zGcrf, zEclipticInGcrf);
        //System.out.println(rotAxis.normalize());
        // According to this convention, the rotation moves vectors wrt a fixed reference frame
        Rotation rot = new Rotation(rotAxis, tiltLaplace, RotationConvention.VECTOR_OPERATOR);
        Vector3D laplaceNormalInGcrf = rot.applyTo(zGcrf);

        // Convert to destination frame 
        Transform gcrf2dest = gcrf.getTransformTo(dest, date);
        Vector3D earthCenterDestFrame = gcrf2dest.transformPosition(Vector3D.ZERO);
        Plane laplace  = new Plane(earthCenterDestFrame, laplaceNormalInGcrf, tolerance);
        
        // Transform equator plane from itrf into gcrf
/*         Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Transform ecef2dest = ecef.getTransformTo(dest, date);
        Vector3D earthRotAxis = ecef2dest.transformVector(Vector3D.PLUS_K);
        Vector3D earthCenter = ecef2dest.transformPosition(Vector3D.ZERO); */
        Plane equator = new Plane(Vector3D.ZERO, Vector3D.PLUS_K, tolerance);

        return laplace;
        //return equator;
    }
}