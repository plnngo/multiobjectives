package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.LUDecomposition;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.events.Action;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.estimation.measurements.AngularAzEl;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.estimation.sequential.ConstantProcessNoise;
import org.orekit.estimation.sequential.KalmanEstimator;
import org.orekit.estimation.sequential.KalmanEstimatorBuilder;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.FactoryManagedFrame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.geometry.fov.FieldOfView;
import org.orekit.geometry.fov.PolygonalFieldOfView;
import org.orekit.geometry.fov.PolygonalFieldOfView.DefiningConeType;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.analytical.tle.generation.FixedPointTleGenerationAlgorithm;
import org.orekit.propagation.conversion.TLEPropagatorBuilder;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.propagation.events.GroundFieldOfViewDetector;
import org.orekit.propagation.events.handlers.ContinueOnEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.TimeStampedPVCoordinates;

import sensortasking.stripescanning.Tasking;


public class TrackingObjective implements Objective{

    /** List of targets of interests with their initial condition. */
    List<ObservedObject> initTargets = new ArrayList<ObservedObject>();

    /** List of targets of interest with latest state update. */
    List<ObservedObject> updatedTargets = new ArrayList<ObservedObject>();

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
    final double maxPropDuration = 60.*10;

    /** Number of observations during one field of regard pass. */
    int numObs = 4;

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

        FactoryManagedFrame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Set up Earth shape
        OneAxisEllipsoid earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                                      Constants.WGS84_EARTH_FLATTENING,
                                                      ecef);

        // Iterate through list of objects of interest
        for (ObservedObject candidate : updatedTargets) {

            final EventDetector visibility =
                    new ElevationDetector(maxcheck, threshold, stationFrame).
                    withConstantElevation(elevation).
                    withHandler((s, d, increasing) -> {
                        System.out.println(" Visibility on object " +
                                           candidate.getId() +
                                           (increasing ? " begins at " : " ends at ") +
                                           s.getDate().toStringWithoutUtcOffset(utc, 3));
                        return increasing ? Action.CONTINUE : Action.STOP;  // stop propagation when object leaves FOR
                    }/* new ContinueOnEvent() */);

            // Set up SGP4 propagator
            TLE tle = candidate.getPseudoTle();
            TLEPropagator tleProp = TLEPropagator.selectExtrapolator(tle);

            // Covariance
            final RealMatrix covInitMatrix = candidate.getCovariance().getCovarianceMatrix();
            final StateCovariance covInit = 
                new StateCovariance(covInitMatrix, candidate.getEpoch(), candidate.getFrame(), 
                                    OrbitType.CARTESIAN, PositionAngleType.MEAN);
            final String stmAdditionalName = "stm";
            final MatricesHarvester harvester = 
                tleProp.setupMatricesComputation(stmAdditionalName, null, null);

            final StateCovarianceMatrixProvider provider = 
                new StateCovarianceMatrixProvider("cov", stmAdditionalName, harvester, covInit);

            tleProp.addAdditionalStateProvider(provider);
            // Add event to be detected
            final EventsLogger logger = new EventsLogger();
            tleProp.addEventDetector(logger.monitorDetector(visibility));

            // Propagate over maximal propoagation duration
            SpacecraftState s = tleProp.propagate(current.shiftedBy(maxPropDuration));
            //SpacecraftState s = tleProp.propagate(current);
            System.out.println("Events: " + logger.getLoggedEvents().size());
            StateVector stateVec = ObservedObject.spacecraftStateToStateVector(s, stationFrame);
            System.out.println("Elevation: " + FastMath.toDegrees(stateVec.getPositionVector().getDelta()));
            System.out.println("Azimuth: " + FastMath.toDegrees(stateVec.getPositionVector().getAlpha()));
            boolean objectAlreadyInFOR = visibility.g(tleProp.getInitialState()) > 0. ? true : false;
            AbsoluteDate[] passInterval = 
                getFORPassDuration(current, logger.getLoggedEvents(), objectAlreadyInFOR);
            // System.out.println("G value end " + visibility.g(s));
            // System.out.println("G value beginning " + visibility.g(tleProp.getInitialState()));
             if (passInterval[0].equals(passInterval[1])){
                // object doess not pass through FOR within the request time duration
                continue;
            } else {
                // schedule observations for FOR pass 
                double totalObsDuration = numObs * taskDuration;
                double actualObsDuration = passInterval[1].durationFrom(passInterval[0]);

                // Correct number of observation request so that they fit into observation window
                while(actualObsDuration<totalObsDuration) {
                    numObs--;
                    totalObsDuration = numObs * taskDuration;
                }

                // TODO: check that timeDurationBetweenTasks > taskDuration
                double timeDurationBetweenTasks = 
                    (passInterval[1].durationFrom(passInterval[0])-totalObsDuration) / (numObs+1);
                AbsoluteDate taskEpoch = passInterval[0].shiftedBy(timeDurationBetweenTasks);
                EventsLogger earthShadowLogger = new EventsLogger();

                // set up eclipse detector
                EclipseDetector eclipseDetector = new EclipseDetector(sun, Constants.SUN_RADIUS, earth)
                                                .withMaxCheck(60.0)
                                                .withThreshold(1.0e-3)
                                                .withHandler(new ContinueOnEvent())
                                                .withUmbra();
                tleProp.addEventDetector(earthShadowLogger.monitorDetector(eclipseDetector));

                // organise observation tasks
                for (int i=0; i<numObs; i++) {

                    // Propagate towards beginning of tasking interval
                    SpacecraftState stateBeginTask = tleProp.propagate(taskEpoch);
                    StateCovariance covBeginTask = provider.getStateCovariance(stateBeginTask);

                    boolean inEarthShadow = eclipseDetector.g(stateBeginTask)<0.0;
                    if (inEarthShadow) {
                        // Observation cannot be performed because of lack of visibility
                        continue;
                    }
                    // Transform spacecraft state into sensor pointing direction
                    AngularDirection azElBeginTask = transformStateToMeasurement(stateBeginTask);
                    
                    boolean goodSolarPhase = 
                        Tasking.checkSolarPhaseCondition(taskEpoch, azElBeginTask);

                    if (!goodSolarPhase) {
                        // Observation cannot be performed because of lack of visibility
                        continue;
                    }
                    double distMoon = 
                        AngularDirection.computeAngularDistMoon(taskEpoch, stationFrame, azElBeginTask);
                    if (distMoon < minMoonDist) {
                        // Observation cannot be performed because of lack of visibility
                        continue;
                    }

                   /*  // If code reaches here, object is observable and visible
                    List<AngularDirection> simulatedMeasurements = new ArrayList<AngularDirection>();

                    // TODO: Placed FOV centrally at location of satellite at beginning of tasking interval --> need to relocate to get maximised pass duration 
                    Vector3D centerFov = new Vector3D(azElBeginTask.getAngle1(), azElBeginTask.getAngle2());
                    Vector3D meridian = new Rotation(Vector3D.PLUS_K, sensorApartureRadius, 
                                                     RotationConvention.VECTOR_OPERATOR).applyTo(centerFov);
                    FieldOfView fov =                    
                        new PolygonalFieldOfView(new Vector3D(azElBeginTask.getAngle1(), azElBeginTask.getAngle2()),
                                                 DefiningConeType.INSIDE_CONE_TOUCHING_POLYGON_AT_EDGES_MIDDLE,
                                                 meridian, sensorApartureRadius, 4, 0);

                    GroundFieldOfViewDetector fovDetector = new GroundFieldOfViewDetector(stationFrame, fov)
                                                                .withHandler((c, d, increasing) -> {
                                                                    System.out.println(" Visibility on object " +
                                                                                    candidate.getId() +
                                                                                    (increasing ? " begins at " : " ends at ") +
                                                                                    c.getDate().toStringWithoutUtcOffset(utc, 3));
                                                                    return increasing ? Action.CONTINUE : Action.STOP;  // stop propagation when object leaves FOR
                                                                });
                    

                    // Set up modified TLE using the state at the beginning of the tasking window
                    TLE modTleBeginTask = TLE.stateToTLE(stateBeginTask, candidate.getPseudoTle());
                    StateVector stateVecBeginTask = ObservedObject.spacecraftStateToStateVector(stateBeginTask, candidate.getFrame());
                    CartesianCovariance cartCovBeginTask = ObservedObject.stateCovToCartesianCov(stateBeginTask.getOrbit(), covBeginTask, candidate.getFrame());
                    ObservedObject candidateBeginTask = new ObservedObject(candidate.getId(), stateVecBeginTask, cartCovBeginTask, stationFrame, modTleBeginTask);

                    // Set up SGP4 propagator to propagate over tasking duration
                    TLEPropagator tlePropBeginTask = TLEPropagator.selectExtrapolator(modTleBeginTask);
                    final EventsLogger logFovPass = new EventsLogger();
                    tlePropBeginTask.addEventDetector(logFovPass.monitorDetector(fovDetector));
                    tlePropBeginTask.propagate(taskEpoch.shiftedBy(taskDuration));

                    List<LoggedEvent> fovCrossings = logFovPass.getLoggedEvents();
                    if (fovCrossings.size()!=2) {
                        throw new Error("Satellite does not cross through FOV as expected.");
                    }

                    // Transform measurements into orekit measurements
                    List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
                    for (AngularDirection meas : simulatedMeasurements) {
                        orekitAzElMeas.add(transformAngularAzEl2OrekitMeasurements(meas));
                    }
                    // Perform updated state estimation with measurements
                    ObservedObject[] result = estimateStateWithKalman(orekitAzElMeas, candidateBeginTask);

                    // Evaluate information gain
                    double iG = computeInformationGain(result[1], result[2]); */

                }
            }
        }

        // Fake data
        return new AngularDirection(null, 
                            new double[]{FastMath.toRadians(88.), FastMath.toRadians(30.)}, 
                            AngleType.AZEL);
    }

    private double computeInformationGain(ObservedObject prior, ObservedObject posterior) {
        return computeJensenShannonDivergence(prior, posterior);
    }
    private double computeJensenShannonDivergence(ObservedObject prior, ObservedObject posterior) {
        // Compute mixture distribution
        ObservedObject mixture = null;

        // Compute KL divergence between prior and mixture
        double klPriorMixture = computeKullbackLeiblerDivergence(prior, mixture);

        // Compute KL divergence between posterior and mixture
        double klPosteriorMixture = computeKullbackLeiblerDivergence(posterior, mixture);

        // Compute JS divergence
        double jsPriorPosterior = 0.5 * klPriorMixture + 0.5 * klPosteriorMixture;

        return jsPriorPosterior;
    }
    private double computeKullbackLeiblerDivergence(ObservedObject p, ObservedObject q) {
        
        // Retrieve covariances
        RealMatrix covP = p.getCovariance().getCovarianceMatrix();
        RealMatrix covQ = q.getCovariance().getCovarianceMatrix();

        // Compute determinant
        LUDecomposition decomP = new LUDecomposition(covP);
        LUDecomposition decomQ = new LUDecomposition(covQ);
        double detP = decomP.getDeterminant();
        double detQ = decomQ.getDeterminant();

        double logDetCovQByDetCovP = FastMath.log(detQ/detP);

        // Compute inverse of covQ
        RealMatrix invCovQ = MatrixUtils.inverse(covQ);

        double traceInvCovQCovP = invCovQ.multiply(covP).getTrace();

        // Substract means of probability distributions
        Vector3D posQ = q.getState().getPositionVector();
        Vector3D velQ = q.getState().getVelocityVector();
        double[] meanStateQ = new double[]{posQ.getX(), posQ.getY(), posQ.getZ(), 
                                           velQ.getX(), velQ.getY(), velQ.getZ()};
        Vector3D posP = p.getState().getPositionVector();
        Vector3D velP = p.getState().getVelocityVector();
        double[] meanStateP = new double[]{posP.getX(), posP.getY(), posP.getZ(),
                                           velP.getX(), velP.getY(), velP.getZ()};

        double[] meanQMinusMeanP = new double[6];
        
        for (int i=0; i<meanQMinusMeanP.length; i++) {
            meanQMinusMeanP[i] = meanStateQ[i] - meanStateP[i];
        }

        // Means transposed multiplied by inverse covariance of Q
        double[] meanTMultiplyInvCovQ = new double[]{0.,0.,0.,0.,0.,0.};
        for (int numCol=0; numCol<invCovQ.getColumnDimension(); numCol++) {
            double[] colInvCovQ = invCovQ.getColumn(numCol);
            for (int i=0; i<meanQMinusMeanP.length; i++) {
                meanTMultiplyInvCovQ[numCol] += meanQMinusMeanP[i] * colInvCovQ[i];
            }
        }
        
        // Multiply with mean again
        double meanTMultiplyInvCovQMultiplyMean = 0.;
        for (int i=0; i<meanQMinusMeanP.length; i++) {
            meanTMultiplyInvCovQMultiplyMean += meanTMultiplyInvCovQ[i] * meanQMinusMeanP[i];
        }
        
        double dKL = 0.5 * (logDetCovQByDetCovP + traceInvCovQCovP 
                                + meanTMultiplyInvCovQMultiplyMean - meanQMinusMeanP.length);
        
        return dKL;
    }
    private ObservedObject[] estimateStateWithKalman(Iterable<ObservedMeasurement<?>> azElMeas,
                                                     ObservedObject candidate) {

        // Initialise output
        ObservedObject[] output = new ObservedObject[3];

        // State at beginning of tasking
        output[0] = new ObservedObject(candidate.getId() + 0, candidate.getState(), 
                                       candidate.getCovariance(), candidate.getFrame(), 
                                       candidate.getPseudoTle());

        // Set initial state covariance
        RealMatrix initialP = candidate.getCovariance().getCovarianceMatrix();

        // Set process noise
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8});
        
        // Set propagator
        //LeastSquaresTleGenerationAlgorithm converter = new LeastSquaresTleGenerationAlgorithm();
        TLEPropagatorBuilder propBuilder = 
            new TLEPropagatorBuilder(candidate.getPseudoTle(), PositionAngleType.MEAN, 1., 
                                     new FixedPointTleGenerationAlgorithm());

        // Build Kalman filter
        KalmanEstimator kalman = 
            new KalmanEstimatorBuilder()
                .addPropagationConfiguration(propBuilder, new ConstantProcessNoise(initialP, Q))
                .build();
        Propagator[] estimated = kalman.processMeasurements(azElMeas);

        // Retrieve updated state and covariance
        AbsoluteDate lastMeasEpoch = estimated[estimated.length-1].getInitialState().getDate();
        SpacecraftState stateEndUpdated = estimated[estimated.length-1].getInitialState();
        StateVector stateVecEndUpdate = ObservedObject.spacecraftStateToStateVector(stateEndUpdated, candidate.getFrame());
        Orbit estimatedO = stateEndUpdated.getOrbit();
        RealMatrix estimatedP = kalman.getPhysicalEstimatedCovarianceMatrix();

        // Process covariance to derive Cartesian orbital covariance matrix
        double[][] dCdY = new double[6][6];
        estimatedO.getJacobianWrtParameters(PositionAngleType.MEAN, dCdY);
        RealMatrix jacobian = MatrixUtils.createRealMatrix(dCdY);
        RealMatrix estimatedCartesianP = jacobian.multiply(estimatedP.getSubMatrix(0, 5, 0, 5))
                                                 .multiply(jacobian.transpose());       // TODO: check if sqrt of diagonal has to be taken
        StateCovariance stateCovEndUpdated = 
            new StateCovariance(estimatedCartesianP, lastMeasEpoch, stationFrame,       // TODO: check frame
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovEndUpdated = 
            ObservedObject.stateCovToCartesianCov(estimatedO, stateCovEndUpdated, 
                                                  candidate.getFrame());

        // State at the end of tasking with measurement updates
        TLE pseudoTleMeas = new FixedPointTleGenerationAlgorithm().generate(stateEndUpdated, 
                                                                        candidate.getPseudoTle());
        output[3] = new ObservedObject(candidate.getId() + 3, stateVecEndUpdate, cartCovEndUpdated,
                                       stationFrame, pseudoTleMeas);

        // Propagate from start of window to last measurement epoch without measurement update
        TLEPropagator prop = TLEPropagator.selectExtrapolator(candidate.getPseudoTle());

        // Covariance
        final RealMatrix covInitMatrix = candidate.getCovariance().getCovarianceMatrix();
        final StateCovariance covInit = 
            new StateCovariance(covInitMatrix, candidate.getEpoch(), candidate.getFrame(), 
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = prop.setupMatricesComputation(stmAdditionalName, null, null);

        final StateCovarianceMatrixProvider provider = 
            new StateCovarianceMatrixProvider("cov", stmAdditionalName, harvester, covInit);

        prop.addAdditionalStateProvider(provider);

        final SpacecraftState stateEndNotUpdated = prop.propagate(lastMeasEpoch);
        StateVector stateVecEndNotUpdate = ObservedObject.spacecraftStateToStateVector(stateEndNotUpdated, candidate.getFrame());
        RealMatrix covProp = provider.getStateCovariance(stateEndNotUpdated).getMatrix();
        StateCovariance stateCovEndNotUpdated = 
            new StateCovariance(covProp, lastMeasEpoch, stateEndNotUpdated.getFrame(),       // TODO: check frame
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovNotEndUpdated = 
            ObservedObject.stateCovToCartesianCov(stateEndNotUpdated.getOrbit(), 
                                                  stateCovEndNotUpdated, candidate.getFrame());

        // State at the end of tasking without measurement updates
        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(stateEndNotUpdated, 
                                                                        candidate.getPseudoTle());
        output[2] = new ObservedObject(candidate.getId() + 2, stateVecEndNotUpdate, cartCovNotEndUpdated, 
                                       stationFrame, pseudoTle);

        return output;
    }
    private AngularAzEl transformAngularAzEl2OrekitMeasurements(AngularDirection meas) {
        AngularAzEl azElOrekit = 
            new AngularAzEl(new GroundStation(stationFrame), meas.getDate(), 
                                              new double[]{meas.getAngle1(), meas.getAngle2()}, 
                                              new double[]{1e-3, 1e-3}, 
                                              new double[]{1., 1.},
                                               new ObservableSatellite(0));
        return azElOrekit;
    }

    /**
     * Transform spacecraft state into angular direction in topocentric horizon frame.
     * 
     * @param state         Spacecraft state.
     * @return              Angular pointing direction with respect to topocentric horizon frame.
     */
    private AngularDirection transformStateToMeasurement(SpacecraftState state) {

        Transform toTopo = state.getFrame().getTransformTo(stationFrame, state.getDate());
        TimeStampedPVCoordinates stateTopo = toTopo.transformPVCoordinates(state.getPVCoordinates());
        Vector3D posTopo = stateTopo.getPosition();
        AngularDirection dirTopo = 
            new AngularDirection(stationFrame, 
                                 new double[]{posTopo.getAlpha(), posTopo.getDelta()}, 
                                 AngleType.AZEL);
        return dirTopo;
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
     * @param alreadyInFOR
     * @return
     */
    public AbsoluteDate[] getFORPassDuration(AbsoluteDate current, List<LoggedEvent> forPass, boolean alreadyInFOR) {
        
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current};
        if (forPass.size() == 2) {
            // object is going to enter and exit FOR within upcoming 15min
            interval = new AbsoluteDate[]{forPass.get(0).getDate(), 
                                          forPass.get(1).getDate()};
        } else if (forPass.size() == 0 && alreadyInFOR) {
            interval[1] = current.shiftedBy(maxPropDuration);
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
        
        ObservedObject obj = new ObservedObject(345, state, cov, new AbsoluteDate(), stationFrame);
        List<ObservedObject> out = new ArrayList<ObservedObject>();
        out.add(obj);
        return out;
    }
    
}
