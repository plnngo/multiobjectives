package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.events.Action;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.geometry.fov.FieldOfView;
import org.orekit.geometry.fov.PolygonalFieldOfView;
import org.orekit.geometry.fov.PolygonalFieldOfView.DefiningConeType;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngle;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.GroundFieldOfViewDetector;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.propagation.events.handlers.ContinueOnEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import sensortasking.stripescanning.Tasking;

public class TrackingObjective implements Objective{

    /** List of targets of interests with their initial condition. */
    List<ObservedObject> initTargets;

    /** List of targets of interest with latest state update. */
    List<ObservedObject> updatedTargets;

    /** Maximum checking interval of event detector. */
    final double maxcheck  = 60.0;

    /** Convergence threshold in the event time search. */
    final double threshold =  0.001;

    /** Cut off eleveation angle in [rad]. */
    final double elevation = FastMath.toRadians(5.0);

    /** Coordinated Universal Time.  */
    final TimeScale utc = TimeScalesFactory.getUTC();

    /** Updated field of regard passes. */
    //List<LoggedEvent> loggedFORpasses = new ArrayList<LoggedEvent>();

    /** Maximal propagation duration in [sec] to check if satellite is entering field of regard. */
    final double maxPropDuration = 60.*15;

    /** Number of observations during one field of regard pass. */
    final int numObs = 4;

    /** Tasking duration for an observation in [sec]. */
    final double taskDuration = 2 * 60.;

    /** Sun. */
    final CelestialBody sun = CelestialBodyFactory.getSun();

    /** Minimal angular distance between Moon and sensor pointing direction. */
    double minMoonDist = FastMath.toRadians(20.);

    double sensorApartureRadius = FastMath.toRadians(2.);

    final TopocentricFrame stationFrame;


    public TrackingObjective(List<ObservedObject> targets, TopocentricFrame gs) {

        // Initialise list of targets
        for (ObservedObject target : targets) {
            initTargets.add(target);
            updatedTargets.add(target);
        }

        this.stationFrame = gs;

    }
    @Override
    public AngularDirection setMicroAction(AbsoluteDate current) {

        // Set up Earth shape
        OneAxisEllipsoid earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                                      Constants.WGS84_EARTH_FLATTENING,
                                                      FramesFactory.getITRF(IERSConventions.IERS_2010,
                                                                            true));

        // Iterate through list of objects of interest
        for (ObservedObject candidate : updatedTargets) {

            final EventDetector visibility =
                    new ElevationDetector(maxcheck, threshold, stationFrame).
                    withConstantElevation(elevation).
                    withHandler((s, d, increasing) -> {
                        System.out.println(" Visibility on " +
                                           ((ElevationDetector) d).getTopocentricFrame().getName() +
                                           (increasing ? " begins at " : " ends at ") +
                                           s.getDate().toStringWithoutUtcOffset(utc, 3));
                        return increasing ? Action.CONTINUE : Action.STOP;
                    });

            // Set up SGP4 propagator
            TLEPropagator tleProp = TLEPropagator.selectExtrapolator(candidate.getPseudoTle());

            // State
            
            // Covariance
            final RealMatrix covInitMatrix = candidate.getCovariance().getCovarianceMatrix();
            final StateCovariance covInit = 
                new StateCovariance(covInitMatrix, candidate.getEpoch(), candidate.getFrame(), 
                                    OrbitType.CARTESIAN, PositionAngle.MEAN);
            final String stmAdditionalName = "stm";
            final MatricesHarvester harvester = 
                tleProp.setupMatricesComputation(stmAdditionalName, null, null);

            final StateCovarianceMatrixProvider provider = 
                new StateCovarianceMatrixProvider("covariance", stmAdditionalName, harvester,
                                                 OrbitType.CARTESIAN, PositionAngle.MEAN, covInit);

            tleProp.addAdditionalStateProvider(provider);
            // Add event to be detected
            final EventsLogger logger = new EventsLogger();
            tleProp.addEventDetector(logger.monitorDetector(visibility));

            // Propagate over maximal propoagation duration
            tleProp.propagate(current.shiftedBy(maxPropDuration));
            AbsoluteDate[] passInterval = getFORPassDuration(current, logger.getLoggedEvents());
            if (passInterval[0].equals(passInterval[1])){
                // object doess not pass through FOR within the upcoming 15mins
                continue;
            } else {
                // schedule observations for FOR pass 
                double totalObsDuration = numObs * taskDuration;

                // TODO: check that timeDurationBetweenTasks > taskDuration
                double timeDurationBetweenTasks = 
                    (passInterval[1].durationFrom(passInterval[0])-totalObsDuration) / (numObs+1);
                AbsoluteDate taskEpoch = passInterval[0].shiftedBy(timeDurationBetweenTasks);
                //TLEPropagator outerProp = TLEPropagator.selectExtrapolator(candidate.getPseudoTle());
                EventsLogger earthShadowLogger = new EventsLogger();

                // set up eclipse detector
                EclipseDetector eclipseDetector = new EclipseDetector(sun, Constants.SUN_RADIUS, earth)
                                                .withMaxCheck(60.0)
                                                .withThreshold(1.0e-3)
                                                .withHandler(new ContinueOnEvent<>())
                                                .withUmbra();
                tleProp.addEventDetector(earthShadowLogger.monitorDetector(eclipseDetector));

                // organise observation tasks
                for (int i=0; i<numObs; i++) {

                    // Propagate over tasking duration
                    SpacecraftState state = tleProp.propagate(taskEpoch);
                    boolean inEarthShadow = eclipseDetector.g(state)<0.0;
                    if (inEarthShadow) {
                        // Observation cannot be performed because of lack of visibility
                        continue;
                    }
                    // Transform spacecraft state into sensor pointing direction
                    AngularDirection dirAzEl = transformStateToMeasurement(state);
                    
                    boolean goodSolarPhase = Tasking.checkSolarPhaseCondition(taskEpoch, dirAzEl);

                    if (!goodSolarPhase) {
                        // Observation cannot be performed because of lack of visibility
                        continue;
                    }
                    double distMoon = 
                        AngularDirection.computeAngularDistMoon(taskEpoch, stationFrame, dirAzEl);
                    if (distMoon < minMoonDist) {
                        // Observation cannot be performed because of lack of visibility
                        continue;
                    }

                    // If code reaches here, object is observable and visible
                    //DoubleDihedraFieldOfView fov = new DoubleDihedraFieldOfView(Vector3D.ZERO, posTeme, timeDurationBetweenTasks, posTeme, distMoon, i)
                    List<AngularDirection> simulatedMeasurements = new ArrayList<AngularDirection>();
                    Vector3D centerFov = new Vector3D(dirAzEl.getAngle1(), dirAzEl.getAngle2());
                    Vector3D meridian = new Rotation(Vector3D.PLUS_K, sensorApartureRadius, 
                                                     RotationConvention.VECTOR_OPERATOR).applyTo(centerFov);
                    FieldOfView fov =
                        new PolygonalFieldOfView(new Vector3D(dirAzEl.getAngle1(), dirAzEl.getAngle2()),
                                                 DefiningConeType.INSIDE_CONE_TOUCHING_POLYGON_AT_EDGES_MIDDLE,
                                                 meridian, sensorApartureRadius, 4, 0);

                    GroundFieldOfViewDetector fovDetector = new GroundFieldOfViewDetector(stationFrame, fov)
                                                                .withHandler((s, d, increasing) -> {
                                                                    System.out.println(" Visibility on satellite" +
                                                                                       (increasing ? " begins at " : " ends at ") +
                                                                                       s.getDate().toStringWithoutUtcOffset(utc, 3));
                                                                    if (increasing) {
                                                                        // TODO: check when a measurement is generated --> which propagation step size
                                                                        simulatedMeasurements.add(generateMeasurements(s));
                                                                    }
                                                                    return increasing ? Action.CONTINUE : Action.STOP;
                                                                });
                    final EventsLogger logFovPass = new EventsLogger();
                    tleProp.addEventDetector(logFovPass.monitorDetector(fovDetector));

                    // Extract FOV pass
                    tleProp.propagate(taskEpoch.shiftedBy(taskDuration/2.));
                    List<LoggedEvent> fovCrossings = logFovPass.getLoggedEvents();
                    if (fovCrossings.size()!=2) {
                        throw new Error("Satellite does not cross through FOV as expected.");
                    }

                    //ArrayList<AngularDirection> simulatedMeas = generateMeasurements(candidate, );

                }
            }



        }

        // Fake data
        return new AngularDirection(null, 
                            new double[]{FastMath.toRadians(88.), FastMath.toRadians(30.)}, 
                            AngleType.AZEL);
    }

/*     @Override
    public double computeGain() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'computeGain'");
    } */

    private AngularDirection transformStateToMeasurement(SpacecraftState state) {
        Vector3D posTeme = state.getPVCoordinates().getPosition(); 
        AngularDirection dirTeme = 
            new AngularDirection(state.getFrame(), 
                                 new double[]{posTeme.getAlpha(), posTeme.getDelta()}, 
                                 AngleType.RADEC);
        return dirTeme.transformReference(stationFrame, state.getDate(), AngleType.AZEL);
    }
    private AngularDirection generateMeasurements(SpacecraftState state) {
        // TODO: add noise
        return transformStateToMeasurement(state);
    }

    @Override
    public double getUtility() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getUtility'");
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        //return 60.*5.;
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current};
/*         if (this.loggedFORpasses.size() == 2) {
            // object is going to enter and exit FOR within upcoming 15min
            interval = new AbsoluteDate[]{this.loggedFORpasses.get(0).getDate(), 
                                          this.loggedFORpasses.get(1).getDate()};
        }
        for (LoggedEvent event : this.loggedFORpasses) {
            if (event.isIncreasing()) {
                // object is going to enter FOR within upcoming 15min
                interval = new AbsoluteDate[]{this.loggedFORpasses.get(0).getDate(),
                                              current.shiftedBy(maxPropDuration)};
            } else {
                // object is going to leave within upcoming 15min
                interval[1] = this.loggedFORpasses.get(0).getDate();
            }
        } */
        return interval;
    }

    /**
     * Compute the time duration of the field of regard pass of the space object logged in 
     * {@code forPass}.
     * 
     * @param current
     * @param forPass
     * @return
     */
    public AbsoluteDate[] getFORPassDuration(AbsoluteDate current, List<LoggedEvent> forPass) {
        
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current};
        if (forPass.size() == 2) {
            // object is going to enter and exit FOR within upcoming 15min
            interval = new AbsoluteDate[]{forPass.get(0).getDate(), 
                                          forPass.get(1).getDate()};
        }
        for (LoggedEvent event : forPass) {
            if (event.isIncreasing()) {
                // object is going to enter FOR within upcoming 15min
                interval = new AbsoluteDate[]{forPass.get(0).getDate(),
                                              current.shiftedBy(maxPropDuration)};
            } else {
                // object is going to leave within upcoming 15min
                interval[1] = forPass.get(0).getDate();
            }
        }
        return interval;
    }

    @Override
    public List<ObservedObject> propagateOutcome() {
        // Fake data
        StateVector state = new StateVector();
        state.setX(150.);
        state.setY(250.);
        state.setZ(350.);

        CartesianCovariance cov = new CartesianCovariance(null);
        for (int pos=0; pos<3; pos++) {
            cov.setCovarianceMatrixEntry(pos, pos, 15.);
            cov.setCovarianceMatrixEntry(pos+3, pos+3, 150.);
        }
        
        ObservedObject obj = new ObservedObject(345, state, cov, new AbsoluteDate(), FramesFactory.getTEME());
        List<ObservedObject> out = new ArrayList<ObservedObject>();
        out.add(obj);
        return out;
    }
    
}
