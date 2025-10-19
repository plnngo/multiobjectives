package sensortasking.mcts;

import java.util.AbstractMap;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.ListIterator;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Objects;
import java.util.Random;

import org.hipparchus.geometry.euclidean.threed.Rotation;
import org.hipparchus.geometry.euclidean.threed.RotationConvention;
import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
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
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.FactoryManagedFrame;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.geometry.fov.FieldOfView;
import org.orekit.geometry.fov.PolygonalFieldOfView;
import org.orekit.geometry.fov.PolygonalFieldOfView.DefiningConeType;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.AbstractPropagator;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLEPropagator;
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
import org.orekit.utils.PVCoordinates;
import org.orekit.utils.TimeStampedPVCoordinates;

import benchtest.Filter;
import benchtest.OrbitRangeAngularMeasurementModel;
import benchtest.Satellite;
import lombok.Getter;
import sensortasking.stripescanning.Tasking;
import tools.OptimisingVector;

@SuppressWarnings("rawtypes")
@Getter
public class TrackingObjective extends Objective<Satellite>{

    /** List of targets of interest with latest state update. */
    List<ObservedObject> updatedTargets = new ArrayList<ObservedObject>();

    /** Maximum checking interval of event detector. */
    final double maxcheck  = 60.0;

    /** Convergence threshold in the event time search. */
    final double threshold =  0.001;

    /** Coordinated Universal Time.  */
    final TimeScale utc = TimeScalesFactory.getUTC();

    /** Maximal propagation duration in [sec] to check if satellite is entering field of regard. */
    final double maxPropDuration = 100.;

    /** Number of observations during one field of regard pass. */
    int numObs = 4;

    /** Tasking duration for an observation in [sec]. */
    final double taskDuration = 2 * 60.;

    /** Last object whose state has been updated. */
    long lastUpdated = Long.MIN_VALUE;

    /** Last object#s information gain. */
    double lastUpdatedIG = 0.;

    /** Preparation time duraution in [sec]. */
    static double preparation = 6.;

    /** Allocation time duration in [sec].  */
    static double allocation = 2.;

    /** Sun. */
    final static CelestialBody sun = CelestialBodyFactory.getSun();

    FactoryManagedFrame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

    Frame j2000 = FramesFactory.getEME2000();

    /** Earth. */
    static OneAxisEllipsoid earth = 
        new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                Constants.WGS84_EARTH_FLATTENING,
                                FramesFactory.getITRF(IERSConventions.IERS_2010, true));

    /** Minimal angular distance between Moon and sensor pointing direction. */
    static double minMoonDist = FastMath.toRadians(20.);

    private double sensorApartureRadius;

    TopocentricFrame stationHorizon;

    /** Sensor. */
    Sensor sensor;

    AbsoluteDate endCampaign;

    AbsoluteDate startCampaign;


    public TrackingObjective(List<ObservedObject> targets, Sensor sensor, 
                             AbsoluteDate endCampaign, AbsoluteDate startCampaign) {

        // Initialise list of targets
        for (ObservedObject target : targets) {
            updatedTargets.add(target);
        }
        this.stationHorizon = sensor.getTopoHorizon();
        this.sensor = sensor;
        this.sensorApartureRadius = sensor.getFov().getWidth()/2;       // TODO: currently assumed that aparture is circular
        this.endCampaign = endCampaign;
        this.startCampaign = startCampaign;
    }

    public void setStationHorizonFrame(TopocentricFrame frame){
        this.stationHorizon = frame;
    }

    
//     public AngularDirection setMicroActionMultiMeasurements(AbsoluteDate current) { 

//         // Output
//         double maxIG = Double.NEGATIVE_INFINITY;
//         // TODO: change to radec
//         AngularDirection pointing = new AngularDirection(stationHorizon, new double[]{0., 0.},
//                                                             AngleType.AZEL);
//         ObservedObject target = null;

//         // Iterate through list of objects of interest
//         for (ObservedObject candidate : updatedTargets) {

//             final EventDetector visibility =
//                     new ElevationDetector(maxcheck, threshold, stationHorizon).
//                     withConstantElevation(elevation).
//                     withHandler((s, d, increasing) -> {
//                         System.out.println(" Visibility on object " +
//                                            candidate.getId() +
//                                            (increasing ? " begins at " : " ends at ") +
//                                            s.getDate().toStringWithoutUtcOffset(utc, 3));
//                         return increasing ? Action.CONTINUE : Action.STOP;  // stop propagation when object leaves FOR
//                     }/* new ContinueOnEvent() */);

//             // Set up SGP4 propagator
//             TLE tle = candidate.getPseudoTle();
//             TLEPropagator tleProp = TLEPropagator.selectExtrapolator(tle);

            
//             // Add event to be detected
//             final EventsLogger logger = new EventsLogger();
//             tleProp.addEventDetector(logger.monitorDetector(visibility));

//             // Propagate over maximal propoagation duration
//             // TODO: set targetDate to measurement epoch that we aim for
//             AbsoluteDate targetDate = current.shiftedBy(maxPropDuration);
//             //SpacecraftState s = tleProp.propagate(targetDate);
//             SpacecraftState s = tleProp.propagate(current);
//             //System.out.println("Events: " + logger.getLoggedEvents().size());
//             //StateVector stateVec = ObservedObject.spacecraftStateToStateVector(s, stationFrame);
//             // System.out.println("Elevation: " + FastMath.toDegrees(stateVec.getPositionVector().getDelta()));
//             // System.out.println("Azimuth: " + FastMath.toDegrees(stateVec.getPositionVector().getAlpha()));
//             boolean objectAlreadyInFOR = visibility.g(tleProp.getInitialState()) > 0. ? true : false;
//             AbsoluteDate[] passInterval = 
//                 getFORPassDuration(current, logger.getLoggedEvents(), objectAlreadyInFOR);
//             // System.out.println("G value end " + visibility.g(s));
//             // System.out.println("G value beginning " + visibility.g(tleProp.getInitialState()));
            
//              if (passInterval[0].equals(passInterval[1])){
//                 // object doess not pass through FOR within the request time duration
//                 continue;
//             } else {
//                 // schedule observations for FOR pass 
//                 double totalObsDuration = numObs * taskDuration;
//                 double actualObsDuration = passInterval[1].durationFrom(passInterval[0]);

//                 // Correct number of observation request so that they fit into observation window
//                 while(actualObsDuration<totalObsDuration) {
//                     numObs--;
//                     totalObsDuration = numObs * taskDuration;
//                 }

//                 TLEPropagator tlePropFOR = TLEPropagator.selectExtrapolator(tle);
//                 // TODO: check that timeDurationBetweenTasks > taskDuration
//                 double timeDurationBetweenTasks = 
//                     (passInterval[1].durationFrom(passInterval[0])-totalObsDuration) / (numObs+1);
//                 //AbsoluteDate taskEpoch = passInterval[0].shiftedBy(timeDurationBetweenTasks);
//                 AbsoluteDate taskEpoch = current;

//                 // organise observation tasks
//                 numObs = 1;
                
//                 for (int i=0; i<numObs; i++) {

//                     Entry<SpacecraftState, StateCovariance> stateBeginTask = 
//                         TrackingObjective.propagateStateAndCovariance(candidate, taskEpoch);

//                     //RealMatrix covPropMatrix = stateBeginTask.getValue().getMatrix();
// /*                     for (int row=0; row<covPropMatrix.getRowDimension(); row++) {
//                         double[] rowVec = covPropMatrix.getRow(row);
//                         for(int col=0; col<covPropMatrix.getColumnDimension(); col++) {
//                             System.out.print(rowVec[col] + " ");
//                         }
//                         System.out.println();
//                     } */

//                     if (Objects.isNull(stateBeginTask)) {
//                         // Observation cannot be performed because of lack of visibility
//                         continue;
//                     }

//                     // Transform spacecraft state into sensor pointing direction
//                     AngularDirection azElBeginTask = 
//                         transformStateToPointing(stateBeginTask.getKey(), stationHorizon);
//                     System.out.println("Azimuth sensor: " + FastMath.toDegrees(azElBeginTask.getAngle1()));
//                     System.out.println("Elevation sensor: " + FastMath.toDegrees(azElBeginTask.getAngle2()));

                    
//                     boolean goodSolarPhase = 
//                         Tasking.checkSolarPhaseCondition(taskEpoch, azElBeginTask);

//                     if (!goodSolarPhase) {
//                         // Observation cannot be performed because of lack of visibility
//                         continue;
//                     }
//                     double distMoon = 
//                         AngularDirection.computeAngularDistMoon(taskEpoch, stationHorizon, azElBeginTask);
//                     if (distMoon < minMoonDist) {
//                         // Observation cannot be performed because of lack of visibility
//                         continue;
//                     }

//                     // If code reaches here, object is observable and visible
                    
//                     List<AngularDirection> simulatedMeasurements = new ArrayList<AngularDirection>();

//                     // Set up modified TLE using the state at the beginning of the tasking window
//                     TLE modTleBeginTask = 
//                         new FixedPointTleGenerationAlgorithm().generate(stateBeginTask.getKey(), 
//                                                                         candidate.getPseudoTle());
//                     StateVector stateVecBeginTask = 
//                         ObservedObject.spacecraftStateToStateVector(stateBeginTask.getKey(), 
//                                                                     candidate.getFrame());
//                     CartesianCovariance cartCovBeginTask = 
//                         ObservedObject.stateCovToCartesianCov(stateBeginTask.getKey().getOrbit(), 
//                                                               stateBeginTask.getValue(), 
//                                                               candidate.getFrame());
//                     ObservedObject candidateBeginTask = 
//                         new ObservedObject(candidate.getId(), stateVecBeginTask, cartCovBeginTask, 
//                                            stationHorizon, modTleBeginTask);
//                     simulatedMeasurements = 
//                         generateMeasurements(candidateBeginTask, azElBeginTask, this.sensor.getReadoutT(), 
//                                              this.sensor.getExposureT(), taskEpoch);

//                     // Transform measurements into orekit measurements
//                     List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
//                     for (AngularDirection meas : simulatedMeasurements) {
//                         orekitAzElMeas
//                             .add(transformAngularAzEl2OrekitMeasurements(meas, this.stationHorizon));
//                     }
//                     // Perform updated state estimation with measurements
//                     ObservedObject[] result = estimateStateWithKalman(orekitAzElMeas, candidateBeginTask);

//                     // Evaluate information gain
//                     double iG = computeInformationGain(result[1], result[2]); 
//                     if (iG>maxIG) {
//                         maxIG = iG;
//                         pointing = azElBeginTask;
//                         target = candidate;
                        
//                     }
//                 }
//             }
//         }
//         System.out.println("Information gain: " + maxIG);
//         return pointing;
//     }

    protected List<AngularDirection> generateMeasurements(ObservedObject candidate,
                                                          AngularDirection pointing, 
                                                          double readout,
                                                          double exposure, AbsoluteDate date) {

        List<AngularDirection> simulatedMeasurements = new ArrayList<AngularDirection>();
        // TODO: Placed FOV centrally at location of satellite at beginning of tasking interval --> need to relocate to get maximised pass duration 
        Vector3D centerFov = new Vector3D(pointing.getAngle1(), pointing.getAngle2());
        Vector3D meridian = new Rotation(Vector3D.PLUS_K, sensorApartureRadius, 
                                            RotationConvention.VECTOR_OPERATOR).applyTo(centerFov);
        
        FieldOfView fov =                    
            new PolygonalFieldOfView(new Vector3D(pointing.getAngle1(), pointing.getAngle2()),
                                        DefiningConeType.INSIDE_CONE_TOUCHING_POLYGON_AT_EDGES_MIDDLE,
                                        meridian, sensorApartureRadius, 4, 0);

        GroundFieldOfViewDetector fovDetector = new GroundFieldOfViewDetector(stationHorizon, fov)
                                                    .withHandler(new ContinueOnEvent());

        // Set up SGP4 propagator to propagate over tasking duration
        TLEPropagator tlePropBeginTask = TLEPropagator.selectExtrapolator(candidate.getPseudoTle());
        final EventsLogger logFovPass = new EventsLogger();
        tlePropBeginTask.addEventDetector(logFovPass.monitorDetector(fovDetector));
        AbsoluteDate end = date.shiftedBy(taskDuration);

        // Take measurements during propagation
        double stepT = readout + exposure;    
        SpacecraftState stateProp = tlePropBeginTask.getInitialState();
        for (AbsoluteDate extrapDate = date;
            extrapDate.compareTo(end) <= 0;
            extrapDate = extrapDate.shiftedBy(stepT))  {
            stateProp = tlePropBeginTask.propagate(extrapDate);
            if (fovDetector.g(stateProp) < 0./* logFovPass.getLoggedEvents().size()== 0 */) {
                simulatedMeasurements.add(transformStateToPointing(stateProp, stationHorizon));
            }
        }
        
        /*System.out.println("Azimuth begin: " + FastMath.toDegrees(transformStateToAzEl(tlePropBeginTask.getInitialState()).getAngle1()));
        System.out.println("El begin: " + FastMath.toDegrees(transformStateToAzEl(tlePropBeginTask.getInitialState()).getAngle2()));*/

        System.out.println("Azimuth end: " 
            + FastMath.toDegrees(transformStateToPointing(stateProp, stationHorizon).getAngle1()));
        System.out.println("El end: " 
            + FastMath.toDegrees(transformStateToPointing(stateProp, stationHorizon).getAngle2()));
        System.out.println("Date end actually " + stateProp.getDate());
        System.out.println("Date end " + end.toString());
        System.out.println("Date begin " + tlePropBeginTask.getInitialState().getDate());
      
        List<LoggedEvent> fovCrossings = logFovPass.getLoggedEvents();
        
        boolean objInFovBeginTask = fovDetector.g(tlePropBeginTask.getInitialState()) < 0.;
        boolean objInFovEndProp = fovDetector.g(stateProp) < 0.;
        if (!(objInFovBeginTask && fovCrossings.size()==1 && !objInFovEndProp)) {
            throw new Error("Satellite does not cross through FOV as expected.");
        }
        return simulatedMeasurements;
    }

    protected static Map.Entry<SpacecraftState,StateCovariance> propagateStateAndCovariance(ObservedObject candidate, AbsoluteDate target){

        TLEPropagator tlePropFOR = TLEPropagator.selectExtrapolator(candidate.getPseudoTle());
        EventsLogger earthShadowLogger = new EventsLogger();

        // set up eclipse detector
        EclipseDetector eclipseDetector = new EclipseDetector(sun, Constants.SUN_RADIUS, earth)
                                        .withMaxCheck(60.0)
                                        .withThreshold(1.0e-3)
                                        .withHandler(new ContinueOnEvent())
                                        .withUmbra();
        tlePropFOR.addEventDetector(earthShadowLogger.monitorDetector(eclipseDetector));

        // Covariance
        final RealMatrix covInitMatrix = candidate.getCovariance().getCovarianceMatrix();
        final StateCovariance covInit = 
            new StateCovariance(covInitMatrix, candidate.getEpoch(), candidate.getFrame(), 
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "STM";
        final MatricesHarvester harvester = 
            tlePropFOR.setupMatricesComputation(stmAdditionalName, null, null);

        final StateCovarianceMatrixProvider provider = 
            new StateCovarianceMatrixProvider("cartCov", stmAdditionalName, harvester, covInit);
        tlePropFOR.addAdditionalStateProvider(provider);

        // Propagate
        SpacecraftState s = tlePropFOR.propagate(target);
        StateCovariance covProp = provider.getStateCovariance(s);

        boolean inEarthShadow = eclipseDetector.g(s)<0.0;
        if (inEarthShadow) {
            // Observation cannot be performed because of lack of visibility
            return null;
        }
        return new AbstractMap.SimpleEntry<>(s, covProp);
    }

    protected static double computeInformationGain(ObservedObject prior, ObservedObject posterior) {
        //return computeJensenShannonDivergence(prior, posterior);
        return computeKullbackLeiblerDivergence(prior, posterior);
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
    protected static double computeKullbackLeiblerDivergence(ObservedObject p, ObservedObject q) {
        
        // Retrieve covariances
        RealMatrix covP = p.getCovariance().getCovarianceMatrix();
        RealMatrix covQ = q.getCovariance().getCovarianceMatrix();

        // Compute determinant
        LUDecomposition decomP = new LUDecomposition(covP);
        LUDecomposition decomQ = new LUDecomposition(covQ);
        double detP = decomP.getDeterminant();
        double detQ = decomQ.getDeterminant();

        // Compute traces
        double traceP = covP.getTrace();
        double traceQ = covQ.getTrace();

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
            
        if (Double.isNaN(dKL)) {
            System.out.println("why nan");
        }
        
        return dKL;
    }

    protected static ObservedObject[] estimateStateWithOwnConventionalKalman(AngularDirection meas, RealMatrix R,
                                                          SpacecraftState predicted, 
                                                          MatricesHarvester harvester, 
                                                          ObservedObject candidate,
                                                          Frame topoInertial,
                                                          double[] angleResiduals,
                                                          double[] xhatPreData) {
        Frame j2000 = FramesFactory.getEME2000();
      
        // Initialise Kalman setting
/*         RealMatrix xbar0 = 
            MatrixUtils.createColumnRealMatrix(new double[]{0., 0., 0., 0., 0., 0.});*/ 
        RealMatrix xhatPre = MatrixUtils.createColumnRealMatrix(xhatPreData); 
        ObservedObject[] output = new ObservedObject[2];

        // Process noise
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-8, 1e-8, 1e-8});
        RealMatrix gamma = App.getGammaMatrix(candidate.getEpoch(), predicted.getDate());
        RealMatrix mappedAcc = gamma.multiply(Q).multiplyTransposed(gamma);

        // Prediction
        RealMatrix dYdY0 = harvester.getStateTransitionMatrix(predicted);
        RealMatrix covInit = candidate.getCovariance().getCovarianceMatrix();
        RealMatrix predictedCov = dYdY0.multiply(covInit).multiplyTransposed(dYdY0);
        predictedCov = predictedCov.add(mappedAcc);     // Add process noise to predicted cov
        RealMatrix xbar = dYdY0.multiply(xhatPre);
        Vector3D predictedPos = predicted.getPVCoordinates().getPosition();
        Vector3D predictedVel = predicted.getPVCoordinates().getVelocity();
        double[] dataPredictedState = 
            new double[]{predictedPos.getX(), predictedPos.getY(), predictedPos.getZ(),
                         predictedVel.getX(), predictedVel.getY(), predictedVel.getZ()};
        RealMatrix predictedStateColumnVec = new Array2DRowRealMatrix(dataPredictedState);

        // Measurement
        Transform toTopo = 
            predicted.getFrame().getTransformTo(topoInertial, predicted.getDate());
        PVCoordinates pvTopo = toTopo.transformPVCoordinates(predicted.getPVCoordinates());
        Vector3D posTopo = pvTopo.getPosition();

        RealMatrix H = App.getObservationPartialDerivative(posTopo, false);
        AngularDirection radec = App.predictMeasurement(posTopo, topoInertial); 
       
        // Compute Kalman Gain
        RealMatrix covInMeasSpace = H.multiply(predictedCov).multiplyTransposed(H);
        RealMatrix kalmanGain = predictedCov.multiplyTransposed(H)
                                            .multiply(MatrixUtils.inverse(covInMeasSpace.add(R)));

        // Measurement error
        AngularDirection residuals = meas.substract(radec);
        angleResiduals[0] = residuals.getAngles()[0];

        angleResiduals[1] = residuals.getAngle2();
        double[][] residualsArray = new double[2][1];
        residualsArray[0] = new double[]{residuals.getAngles()[0]};
        residualsArray[1] = new double[]{residuals.getAngle2()};
        RealMatrix residualMatrix = MatrixUtils.createRealMatrix(residualsArray);

        // Correction
        RealMatrix xhat = xbar.add(kalmanGain.multiply(residualMatrix.subtract(H.multiply(xbar))));
        RealMatrix updatedState = predictedStateColumnVec.add(xhat);
        double[] xhatPrePlaceholder = xhat.getColumn(0);
        for(int i=0; i<xhatPrePlaceholder.length; i++){
            xhatPreData[i] = xhatPrePlaceholder[i];
        }

        RealMatrix kRkT = kalmanGain.multiply(R).multiplyTransposed(kalmanGain);
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(6);
        RealMatrix iMinusKgH = identity.subtract(kalmanGain.multiply(H));
        RealMatrix updatedCov = 
            iMinusKgH.multiply(predictedCov).multiplyTransposed(iMinusKgH).add(kRkT);
        
        // Set up output prediction
        StateVector predState = ObservedObject.spacecraftStateToStateVector(predicted, predicted.getFrame());
        StateCovariance stateCov = 
            new StateCovariance(predictedCov, predicted.getDate(), j2000, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN);
        CartesianCovariance predCartCov = 
            ObservedObject.stateCovToCartesianCov(predicted.getOrbit(), stateCov, j2000);
        output[0] = new Satellite(candidate.getId(), predState, predCartCov, 
                                       predicted.getDate(), j2000);
            
        // Set up output updated
        double[] updatedArray = updatedState.getColumn(0);
        Vector3D posUpdated = new Vector3D(updatedArray[0], updatedArray[1], updatedArray[2]);
        Vector3D velUpdated = new Vector3D(updatedArray[3], updatedArray[4], updatedArray[5]);
        PVCoordinates pvUpdated = new PVCoordinates(posUpdated, velUpdated);
        CartesianOrbit updatedOrbit = new CartesianOrbit(pvUpdated, j2000, predicted.getDate(), Constants.WGS84_EARTH_MU);
        SpacecraftState updated = new SpacecraftState(updatedOrbit);
        StateVector corrState = ObservedObject.spacecraftStateToStateVector(updated, j2000);
        StateCovariance updatedStateCov = 
            new StateCovariance(updatedCov, predicted.getDate(), j2000, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN);
        CartesianCovariance corrCartCov = 
            ObservedObject.stateCovToCartesianCov(updatedOrbit, updatedStateCov, j2000);
        output[1] = new Satellite(candidate.getId(), corrState, corrCartCov, 
                                       predicted.getDate(), j2000);
        return output;
    }

   

    /**
     * Implementation of extended Kalman Filter.
     * 
     * @param prop
     * @param epoch             Measurement epoch.
     * @param R
     * @param candidate
     * @param topoInertial
     * @param angleResiduals
     * @return                  ObservedObject[] where prediction is stored in first and correction
     *                          in second field.
     */
    protected static ObservedObject[] estimateStateWithOwnExtendedKalman(AbstractPropagator prop,
                                                                         AbsoluteDate epoch, 
                                                                         RealMatrix R,
                                                                         ObservedObject candidate,
                                                                         double[] angleResiduals,
                                                                         Sensor sensor) {
        
        // Topocentric frame
        Frame topocentric = sensor.getTopoInertialFrame(epoch, sensor.getPosition());

        // Set up covariance matrix provider and add it to the propagator
        final String stmName = "stm";
        final MatricesHarvester harvester = 
            prop.setupMatricesComputation(stmName, null, null);
        SpacecraftState predicted = prop.propagate(epoch);
        AngularDirection meas = 
            TrackingObjective.transformStateToPointing(predicted, topocentric);

        Frame j2000 = FramesFactory.getEME2000();
      
        // Initialise Kalman setting
        RealMatrix xbar0 = 
            MatrixUtils.createColumnRealMatrix(new double[]{0., 0., 0., 0., 0., 0.});
        RealMatrix xhatPre = xbar0; 
        ObservedObject[] output = new ObservedObject[2];

        // Process noise
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{0.,0.,0.});

        RealMatrix gamma = App.getGammaMatrix(candidate.getEpoch(), predicted.getDate());
        RealMatrix mappedAcc = gamma.multiply(Q).multiplyTransposed(gamma);

        // Prediction
        RealMatrix dYdY0 = harvester.getStateTransitionMatrix(predicted);
        RealMatrix covInit = candidate.getCovariance().getCovarianceMatrix();
        RealMatrix predictedCov = dYdY0.multiply(covInit).multiplyTransposed(dYdY0);
        predictedCov = predictedCov.add(mappedAcc);     // Add process noise to predicted cov
        RealMatrix xbar = dYdY0.multiply(xhatPre);
        Vector3D predictedPos = predicted.getPVCoordinates().getPosition();
        Vector3D predictedVel = predicted.getPVCoordinates().getVelocity();
        double[] dataPredictedState = 
            new double[]{predictedPos.getX(), predictedPos.getY(), predictedPos.getZ(),
                         predictedVel.getX(), predictedVel.getY(), predictedVel.getZ()};
        RealMatrix predictedStateColumnVec = new Array2DRowRealMatrix(dataPredictedState);

        // Measurement
        Transform toTopo = 
            predicted.getFrame().getTransformTo(meas.getFrame(), predicted.getDate());
        PVCoordinates pvTopo = toTopo.transformPVCoordinates(predicted.getPVCoordinates());
        Vector3D posTopo = pvTopo.getPosition();

        RealMatrix H = App.getObservationPartialDerivative(posTopo, false);
        AngularDirection radec = App.predictMeasurement(posTopo, meas.getFrame()); 
       
        // Compute Kalman Gain
        RealMatrix covInMeasSpace = H.multiply(predictedCov).multiplyTransposed(H);
        double sigmaRA = FastMath.sqrt(covInMeasSpace.getEntry(0, 0));
        //System.out.println("Predicted uncertainty RA: " + FastMath.toDegrees(sigmaRA));
        double sigmaDEC = FastMath.sqrt(covInMeasSpace.getEntry(1, 1));
        //System.out.println("Predicted uncertainty Dec: " + FastMath.toDegrees(sigmaDEC));

        if (sigmaRA > FastMath.toRadians(1.)) {
            //System.out.println("Uncertainty exceeds FOV in right ascesion dir");    
        }
        if (sigmaDEC > FastMath.toRadians(1.)) {
            //System.out.println("Uncertainty exceeds FOV in declination dir");    
        }
        RealMatrix kalmanGain = predictedCov.multiplyTransposed(H)
                                            .multiply(MatrixUtils.inverse(covInMeasSpace.add(R)));

        // Measurement error
        AngularDirection residuals = meas.substract(radec);
        angleResiduals[0] = residuals.getAngles()[0];

        angleResiduals[1] = residuals.getAngle2();
        double[][] residualsArray = new double[2][1];
        residualsArray[0] = new double[]{residuals.getAngles()[0]};
        residualsArray[1] = new double[]{residuals.getAngle2()};
        RealMatrix residualMatrix = MatrixUtils.createRealMatrix(residualsArray);

        // Correction
        RealMatrix xhat = xbar.add(kalmanGain.multiply(residualMatrix.subtract(H.multiply(xbar))));
        RealMatrix updatedState = predictedStateColumnVec.add(xhat);

        RealMatrix kRkT = kalmanGain.multiply(R).multiplyTransposed(kalmanGain);
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(6);
        RealMatrix iMinusKgH = identity.subtract(kalmanGain.multiply(H));
        RealMatrix updatedCov = 
            iMinusKgH.multiply(predictedCov).multiplyTransposed(iMinusKgH).add(kRkT);

        // Set up output prediction
        StateVector predState = ObservedObject.spacecraftStateToStateVector(predicted, predicted.getFrame());
        StateCovariance stateCov = 
            new StateCovariance(predictedCov, predicted.getDate(), j2000, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN);
        CartesianCovariance predCartCov = 
            ObservedObject.stateCovToCartesianCov(predicted.getOrbit(), stateCov, j2000);
        output[0] = new Satellite(candidate.getId(), predState, predCartCov, 
                                       predicted.getDate(), j2000);
            
        // Set up output updated
        double[] updatedArray = updatedState.getColumn(0);
        Vector3D posUpdated = new Vector3D(updatedArray[0], updatedArray[1], updatedArray[2]);
        Vector3D velUpdated = new Vector3D(updatedArray[3], updatedArray[4], updatedArray[5]);
        PVCoordinates pvUpdated = new PVCoordinates(posUpdated, velUpdated);
        CartesianOrbit updatedOrbit = new CartesianOrbit(pvUpdated, j2000, predicted.getDate(), Constants.WGS84_EARTH_MU);
        SpacecraftState updated = new SpacecraftState(updatedOrbit);
        StateVector corrState = ObservedObject.spacecraftStateToStateVector(updated, j2000);
        StateCovariance updatedStateCov = 
            new StateCovariance(updatedCov, predicted.getDate(), j2000, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN);
        CartesianCovariance corrCartCov = 
            ObservedObject.stateCovToCartesianCov(updatedOrbit, updatedStateCov, j2000);
        output[1] = new Satellite(candidate.getId(), corrState, corrCartCov, 
                                       predicted.getDate(), j2000);

        // Corrected covariance projected in measurement space
        Transform updatedToTopo = 
            updated.getFrame().getTransformTo(meas.getFrame(), updated.getDate());
        PVCoordinates pvUpdatedTopo = updatedToTopo.transformPVCoordinates(updated.getPVCoordinates());
        Vector3D posUpdatedTopo = pvUpdatedTopo.getPosition();

        RealMatrix Hupdated = App.getObservationPartialDerivative(posUpdatedTopo, false);
        RealMatrix covUpdatedInMeasSpace = Hupdated.multiply(updatedCov).multiplyTransposed(Hupdated);
        double sigmaRAUpdated = FastMath.sqrt(covUpdatedInMeasSpace.getEntry(0, 0));
        //System.out.println("Updated uncertainty RA: " + FastMath.toDegrees(sigmaRAUpdated));
        double sigmaDECUpdated = FastMath.sqrt(covUpdatedInMeasSpace.getEntry(1, 1));
        //System.out.println("Updated uncertainty Dec: " + FastMath.toDegrees(sigmaDECUpdated));
        
        return output;
    }


    /* protected ObservedObject[] estimateStateWithKalman(Iterable<ObservedMeasurement<?>> azElMeas,
                                                     ObservedObject candidate) {

        // Initialise output
        ObservedObject[] output = new ObservedObject[3];

        // State at beginning of tasking
        output[0] = new ObservedObject(candidate.getId() + 0, candidate.getState(), 
                                       candidate.getCovariance(), candidate.getFrame(), 
                                       candidate.getPseudoTle());

        // Set initial state covariance
        RealMatrix initialP = candidate.getCovariance().getCovarianceMatrix();

       //initialP = MatrixUtils.createRealIdentityMatrix(6).scalarMultiply(0.0);

        // Set process noise
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8});
        Q = MatrixUtils.createRealIdentityMatrix(numObs).scalarMultiply(0.0);
        
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
            new StateCovariance(estimatedCartesianP, lastMeasEpoch, candidate.getFrame(),       // TODO: check frame
                                OrbitType.CARTESIAN, PositionAngleType.MEAN);
        CartesianCovariance cartCovEndUpdated = 
            ObservedObject.stateCovToCartesianCov(estimatedO, stateCovEndUpdated, 
                                                  candidate.getFrame());

        // State at the end of tasking with measurement updates
        TLE pseudoTleMeas = new FixedPointTleGenerationAlgorithm().generate(stateEndUpdated, 
                                                                        candidate.getPseudoTle());
        output[2] = new ObservedObject(candidate.getId() + 3, stateVecEndUpdate, cartCovEndUpdated,
                                       stationHorizon, pseudoTleMeas);

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
        output[1] = new ObservedObject(candidate.getId() + 2, stateVecEndNotUpdate, cartCovNotEndUpdated, 
                                       stationHorizon, pseudoTle);

        return output;
    } */

    /**
     * Transform angular diretion into orekit AngularAzEl object.
     * TODO: shift this function into AngularDirection.java
     * 
     * @param meas          Angular direction that is to be transformed.
     * @return              Orekit azimuth/elevation object.
     */
    protected static AngularAzEl transformAngularAzEl2OrekitMeasurements(AngularDirection meas, 
                                                                  TopocentricFrame station) {

        if(!meas.getAngleType().equals(AngleType.AZEL)) {
            throw new Error("This is not an azimuth/elevation measurement pair");
        }
        if(Objects.isNull(meas.getDate())) {
            throw new NullPointerException("Angular measurement has unknown date");
        }
        AngularAzEl azElOrekit = 
            new AngularAzEl(new GroundStation(station), meas.getDate(), 
                            new double[]{meas.getAngle1(), meas.getAngle2()}, 
                            new double[]{1e-3, 1e-3}, 
                            new double[]{1., 1.},
                            new ObservableSatellite(0));
        return azElOrekit;
    }

    /**
     * Transform spacecraft state into angular direction in topocentric frame.
     * TODO: move function to AngularDirection class
     * 
     * @param state         Spacecraft state.
     * @param topo          Topocentric frame (either inertial or horizon frame).
     * @return              Angular pointing direction with respect to topocentric frame.
     */
    protected static AngularDirection transformStateToPointing(SpacecraftState state, Frame topo) {

        Transform toTopo = state.getFrame().getTransformTo(topo, state.getDate());
        TimeStampedPVCoordinates stateTopo = 
            toTopo.transformPVCoordinates(state.getPVCoordinates());
        Vector3D posTopo = stateTopo.getPosition();
        double posRange = posTopo.getNorm();
        AngleType angleType;
        if(topo.getClass().isInstance(TopocentricFrame.class)) {
            angleType = AngleType.AZEL;
        } else {
            angleType = AngleType.RADEC;
        }
        AngularDirection dirTopo = 
            new AngularDirection(topo, 
                                 new double[]{posTopo.getAlpha(), posTopo.getDelta()}, 
                                 angleType, posRange);
        dirTopo.setDate(state.getDate());
        return dirTopo;
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        // Get epoch of last tracking measurement
        AbsoluteDate lastUpdateEpoch = new AbsoluteDate();
        for(ObservedObject obj : this.getUpdatedTargets()) {
            if (obj.getId() == this.lastUpdated) {
                lastUpdateEpoch = obj.getEpoch();
                break;
            }
        }
        
        /* AbsoluteDate[] interval = 
            new AbsoluteDate[]{current, lastUpdateEpoch.shiftedBy(sensor.getExposureT()/2 
                                                                    + sensor.getReadoutT())}; */
        AbsoluteDate[] interval = new AbsoluteDate[]{current, lastUpdateEpoch};

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
    public List<Satellite> propagateOutcome() {

        // return copy of updated targets
        List<Satellite> out = new ArrayList<Satellite>();
        for (ObservedObject obj : this.updatedTargets) {
            Satellite copy = new Satellite(obj.getId(), obj.getState(), 
                                                     obj.getCovariance(), obj.getEpoch(), 
                                                     obj.getFrame());
            out.add(copy);
        }
        
        return out;
    }

    @Override
    public AngularDirection setMicroAction(AbsoluteDate current, AngularDirection sensorPointing){
        final double tstep = 10.;   
        double tobs = current.durationFrom(this.startCampaign) + tstep;

        // No cars to track
        if (this.updatedTargets.isEmpty()) {
            return null;
        }

        // List of candidates that might be trackable
        Map<ObservedObject, Double> checkTrackable = new HashMap<ObservedObject, Double>();        
        for (ObservedObject obj : updatedTargets) {
            ObservedObject copy = new Satellite(obj.getId(), obj.getState(), 
                                                     obj.getCovariance(), obj.getEpoch(), 
                                                     obj.getFrame());
            
            double[] state = new double[]{copy.getState().getPositionVector().getX(),
                                          copy.getState().getPositionVector().getY(),
                                          copy.getState().getPositionVector().getZ(),
                                          copy.getState().getVelocityVector().getX(),
                                          copy.getState().getVelocityVector().getY(),
                                          copy.getState().getVelocityVector().getZ()};
            double[][] stateCov = copy.getCovariance().getCovarianceMatrix().getData();
            Filter est = new Filter();
            est.run_ckf(state, stateCov, copy.getEpoch(), this.startCampaign.shiftedBy(tobs));
            
            // Compute informtion gain
            double iG = TrackingObjective.computeTraceChange(est.getCovPred(), 
                                                                est.getCovCorr());

            ObservedObject copyUpdated = 
                new Satellite(copy.getId(), 
                              ObservedObject.arrayToStateVector(est.getStateCorr()), 
                              ObservedObject.arrayToCartesianCov(est.getCovCorr()), 
                              current.shiftedBy(tstep), copy.getFrame());
            checkTrackable.put(copyUpdated, iG);
        }

        // Step 1: Find max IG
        Random rand = new Random();
        double iGmax = -Double.MAX_VALUE;
        for (Entry<ObservedObject, Double> entry : checkTrackable.entrySet()) {
            if (entry.getValue() > iGmax) {
                iGmax = entry.getValue();
            }
        }

        // Step 2: Collect all cars with max IG
        List<ObservedObject> bestCandidates = new ArrayList<>();
        for (Entry<ObservedObject, Double> entry : checkTrackable.entrySet()) {
            if (entry.getValue() == iGmax) {
                bestCandidates.add(entry.getKey());
            }
        }

        // Step 3: Pick one randomly
        ObservedObject selectedRaw = bestCandidates.get(rand.nextInt(bestCandidates.size()));

         // Step 4: Extract state vector of the selected Car object
        double[] stateUpdated = new double[]{selectedRaw.getState().getPositionVector().getX(),
                                             selectedRaw.getState().getPositionVector().getY(), 
                                             selectedRaw.getState().getPositionVector().getZ(),
                                             selectedRaw.getState().getVelocityVector().getX(),
                                             selectedRaw.getState().getVelocityVector().getY(),
                                             selectedRaw.getState().getVelocityVector().getZ()};

        // Step 5: Update targets
        for (ObservedObject candidate : this.updatedTargets) {
            if (candidate.getId() == selectedRaw.getId()) {
                candidate.setState(ObservedObject.arrayToStateVector(stateUpdated));
                candidate.setCovariance(selectedRaw.getCovariance());
                candidate.setEpoch(current.shiftedBy(tstep)); 
                candidate.setFrame(selectedRaw.getFrame());               
                this.lastUpdated = selectedRaw.getId();
                this.lastUpdatedIG = iGmax;

                // Step 6: Transform into topocentric frame
                Frame topo = sensor.getTopoInertialFrame(candidate.getEpoch(), sensor.getPosition());
                Transform toTopo = 
                    candidate.getFrame().getTransformTo(topo, candidate.getEpoch());
                PVCoordinates pvGeo = new PVCoordinates(candidate.getState().getPositionVector(), 
                                                        candidate.getState().getVelocityVector());
                PVCoordinates pvTopo = toTopo.transformPVCoordinates(pvGeo);
                Vector3D posTopo = pvTopo.getPosition();
                double[] simMeas = 
                    OrbitRangeAngularMeasurementModel.generateHk(new double[]{posTopo.getX(), 
                                                                              posTopo.getY(), 
                                                                              posTopo.getZ()}).Gk;
                // Extract angular measurement
                double ra = simMeas[0];
                double dec = simMeas[1];
                double range = simMeas[2];
                AngularDirection angle = 
                    new AngularDirection(topo, new double[]{ra, dec}, AngleType.RADEC, range);
                angle.setDate(candidate.getEpoch());
                return angle;
            }
        } 
        // No targets to track
        return null;
    }
    /**
     * 
     * @param current           Epoch of the last decision node
     * @param sensorPointing    Pointing location of the sensor at the current epoch
     * 
    */
    public AngularDirection setMicroActionWithVisibilityChecks(AbsoluteDate current, AngularDirection sensorPointing){

        // List of potentially updated target 
        List<ObservedObject> targets = new ArrayList<ObservedObject>();

        // List of trackable objects with their potential IG and relocation duration
        Map<AngularDirection, double[]> trackable = new LinkedHashMap<AngularDirection, double[]>();

        // List of candidates that might be trackable
        List<ObservedObject> checkTrackable = new ArrayList<ObservedObject>();        
        for (ObservedObject obj : updatedTargets) {
            ObservedObject copy = new Satellite(obj.getId(), obj.getState(), 
                                                     obj.getCovariance(), obj.getEpoch(), 
                                                     obj.getFrame());
            checkTrackable.add(copy);
        }

        // Initialise target date
        AbsoluteDate measEpoch = 
                    current.shiftedBy(TrackingObjective.allocation 
                                        + this.sensor.getSettlingT() 
                                        + TrackingObjective.preparation 
                                        + this.sensor.getExposureT()/2);

        // Check in the upcoming time range between 1s and 200s when objects are trackable 
        double tStep = 1.; 
        for (int t=0; t<200; t++) {
            measEpoch = measEpoch.shiftedBy(tStep);

            // Transform sensor pointing direction in new topocentric inertial frame
            Frame topoInertial = this.sensor.getTopoInertialFrame(measEpoch, this.sensor.getPosition());
            AngularDirection sensorP = 
                sensorPointing.transformReference(topoInertial, measEpoch, 
                                                  sensorPointing.getAngleType());
  
            // Iterate through list of objects of interest
            ListIterator<ObservedObject> iterator = checkTrackable.listIterator();
            while (iterator.hasNext()) {
            //for (ObservedObject candidate : checkTrackable) {
            ObservedObject candidate = iterator.next();

                final EventDetector visibility =
                        new ElevationDetector(maxcheck, threshold, stationHorizon)
                        .withConstantElevation(this.sensor.getElevCutOff())
                        .withHandler((s, d, increasing) -> {
                            System.out.println(" Visibility on object " +
                                            candidate.getId() +
                                            (increasing ? " begins at " : " ends at ") +
                                            s.getDate().toStringWithoutUtcOffset(utc, 3));
                            return increasing ? Action.CONTINUE : Action.STOP;  // stop propagation when object leaves FOR
                        });
                // set up eclipse detector
                EclipseDetector eclipseDetector = new EclipseDetector(sun, Constants.SUN_RADIUS, earth)
                                                    .withMaxCheck(60.0)
                                                    .withThreshold(1.0e-3)
                                                    .withHandler(new ContinueOnEvent())
                                                    .withUmbra();

                // Set up propagator
                Vector3D pos = candidate.getState().getPositionVector();
                Vector3D vel = candidate.getState().getVelocityVector();
                PVCoordinates pv = new PVCoordinates(pos, vel);
                Orbit initialOrbit = new CartesianOrbit(pv, candidate.getFrame(), 
                                        candidate.getEpoch(), Constants.WGS84_EARTH_MU);
                KeplerianPropagator kepPropo = new KeplerianPropagator(initialOrbit);
                
                // Add event to be detected
                final EventsLogger horizonLogger = new EventsLogger();
                EventsLogger earthShadowLogger = new EventsLogger();
                kepPropo.addEventDetector(horizonLogger.monitorDetector(visibility));
                kepPropo.addEventDetector(earthShadowLogger.monitorDetector(eclipseDetector));

                // Propagate
                SpacecraftState predState = kepPropo.propagate(measEpoch);
                if (eclipseDetector.g(predState)<0.) {
                    // Observation cannot be performed because object in Earth shadow
                    continue;
                } else if (visibility.g(predState) < 0.) {
                    // Object not in FOV even though sensor was placed such that visibility is provided
                    throw new IllegalArgumentException("Object is not in FOV");
                }

                // Transform spacecraft state into sensor pointing direction
                AngularDirection raDecPointing = transformStateToPointing(predState, topoInertial);
                raDecPointing.setDate(measEpoch);

                boolean goodSolarPhase = 
                    Tasking.checkSolarPhaseCondition(measEpoch, raDecPointing);

                if (!goodSolarPhase) {
                    // Observation cannot be performed because of lack of visibility
                    continue;
                }
                double distMoon = 
                    AngularDirection.computeAngularDistMoon(measEpoch, topoInertial, raDecPointing);
                if (distMoon < minMoonDist) {
                    // Observation cannot be performed because of lack of visibility
                    continue;
                }
                double actualSlewT = 
                    this.sensor.computeRepositionT(sensorP, raDecPointing, true);
                double reloc = TrackingObjective.allocation + t*tStep;
                if(actualSlewT > reloc) {
                    // not enough time to slew to target pointing direction
                    continue;
                }
                
                // Generate real measurement
                RealMatrix R = 
                    MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow(1./206265, 2), 
                                                                    FastMath.pow(1./206265, 2)});
                
                double[] residuals = new double[2];
                ObservedObject[] predAndCorr = 
                    estimateStateWithOwnExtendedKalman(kepPropo, measEpoch, R, candidate, residuals, sensor);
                
                // Check if predicted uncertainty fits into FOV
                RealMatrix covPredGeocentric = predAndCorr[0].getCovariance().getCovarianceMatrix();
                PVCoordinates pvPred = 
                    new PVCoordinates(predAndCorr[0].getState().getPositionVector(),
                                      predAndCorr[0].getState().getVelocityVector());
                Orbit orbitPred = new CartesianOrbit(pvPred, predAndCorr[0].getFrame(), measEpoch, 
                                                     Constants.WGS84_EARTH_MU);
                StateCovariance stateCov = 
                    new StateCovariance(covPredGeocentric, measEpoch, predAndCorr[0].getFrame(), 
                                        OrbitType.CARTESIAN, PositionAngleType.MEAN);
                StateCovariance stateCovTopo = 
                    stateCov.changeCovarianceFrame(orbitPred, topoInertial);

                double iG = computeInformationGain(predAndCorr[0], predAndCorr[1]);
                
                // Generate optimisation vector with IG as 1st and relocation time as 2nd entry
                double[] iGreloc = new double[]{iG, reloc};
                trackable.put(raDecPointing, iGreloc);
                targets.add(predAndCorr[1]);
                
                // Remove object that is trackable from list of check candidates
                iterator.remove(); 
            } 
            if(checkTrackable.isEmpty()) {
                break;
            }
        }
        // If list of trackable options is empty, tracking task not possible
        if(trackable.isEmpty()) {
            // none of the considered taregts is observable
            return null;
        } else if (trackable.size() == 1) {
            // track object 
            ObservedObject candidate = targets.get(0);
            this.updatedTargets.get(0).setState(candidate.getState());
            this.updatedTargets.get(0).setCovariance(candidate.getCovariance());
            this.updatedTargets.get(0).setEpoch(candidate.getEpoch());
            this.lastUpdated = candidate.getId();
            
            return trackable.keySet().iterator().next();
        } else {
            // select target based on optimisation: max IG, min 
            List<double[]> toOpt = new ArrayList<double[]>(trackable.values());

            // Compute number of dominating solutions
            int[] numDominating = new int[trackable.size()];
            for (int i=0; i<toOpt.size(); i++) {
                double[] toCompare = toOpt.get(0);
                toOpt.remove(0);

                OptimisingVector optIgReloc = new OptimisingVector(toOpt, 0);
                numDominating[i] = optIgReloc
                                    .getDominatingVecs(toCompare, new boolean[]{true, false}, 0)
                                    .size();
                toOpt.add(toCompare);
            }

            // Find optimal solution(s) that minimises number of dominating solutions
            List<Integer> solutionIndexes = new ArrayList<>();
            for(int i=0; i<trackable.size(); i++) {
                solutionIndexes.add(i);
            }
            int min = Integer.MAX_VALUE;
            for (int i=0; i<numDominating.length; i++) {
                if(numDominating[i] < min) {
                    min = numDominating[i];
                    solutionIndexes.clear();
                    solutionIndexes.add(i);
                } else if (numDominating[i] == min) {
                    solutionIndexes.add(i);
                }
            }  
            for (int i=0; i<solutionIndexes.size(); i++) {
                int checkWithinCampaign = targets.get(solutionIndexes.get(i))
                                                        .getEpoch()
                                                        .shiftedBy(sensor.getExposureT()/2 
                                                                    + sensor.getReadoutT())
                                                        .compareTo(this.endCampaign);
                if(checkWithinCampaign>=0) {
                    // Tracking epoch outside campaign window
                    solutionIndexes.remove(i);
                    i--;
                }
            }

            // Select solution randomly from all left options
            Random rand = new Random();
            if (solutionIndexes.size() == 0) {
                // Targets not trackable because measurement epoch outside campaign window
                return null; 
            }
            int indexOfIndexes = rand.nextInt(solutionIndexes.size());
            int solutionIndex = solutionIndexes.get(indexOfIndexes);
            List<AngularDirection> trackableDir = 
                new ArrayList<AngularDirection>(trackable.keySet());
            ObservedObject target = targets.get(solutionIndex);

            // Update targeted candidate in the list of objects of interest TODO: only update in expansion not simulation phase --> update in the frame of simulation but don't overwrite on expansion global level
            for(ObservedObject candidate : updatedTargets) {
                if(candidate.getId() == target.getId()) {
                    candidate.setState(target.getState());
                    candidate.setCovariance(target.getCovariance());
                    candidate.setEpoch(target.getEpoch());
                    this.lastUpdated = candidate.getId();
                    break;
                }
            }   
            
            return trackableDir.get(solutionIndex);
        }
    }

    @Deprecated
    /**
     * Do not use this function since scale is not justified!
     * @param current
     * @param sensorPointing
     * @return
     */
    public AngularDirection setMicroActionFixedAllocAngularDirection(AbsoluteDate current, AngularDirection sensorPointing) {

        Frame topoInertial = this.sensor.getTopoInertialFrame(current, this.sensor.getPosition());

        // Output
        double maxIG = Double.NEGATIVE_INFINITY;
        AngularDirection pointing = new AngularDirection(topoInertial, new double[]{0., 0.},
                                                            AngleType.RADEC, 1.);
        ObservedObject target = null;    

        AbsoluteDate targetDate = 
                current.shiftedBy(allocation + this.sensor.getSettlingT() + preparation 
                                + this.sensor.getExposureT()/2);

        // Iterate through list of objects of interest
        for (ObservedObject candidate : updatedTargets) {

            final EventDetector visibility =
                    new ElevationDetector(maxcheck, threshold, stationHorizon)
                    .withConstantElevation(this.sensor.getElevCutOff())
                    .withHandler((s, d, increasing) -> {
                        System.out.println(" Visibility on object " +
                                           candidate.getId() +
                                           (increasing ? " begins at " : " ends at ") +
                                           s.getDate().toStringWithoutUtcOffset(utc, 3));
                        return increasing ? Action.CONTINUE : Action.STOP;  // stop propagation when object leaves FOR
                    });
            // set up eclipse detector
            EclipseDetector eclipseDetector = new EclipseDetector(sun, Constants.SUN_RADIUS, earth)
                                                .withMaxCheck(60.0)
                                                .withThreshold(1.0e-3)
                                                .withHandler(new ContinueOnEvent())
                                                .withUmbra();

            // Set up propagator
            Vector3D pos = candidate.getState().getPositionVector();
            Vector3D vel = candidate.getState().getVelocityVector();
            PVCoordinates pv = new PVCoordinates(pos, vel);
            Orbit initialOrbit = new CartesianOrbit(pv, candidate.getFrame(), 
                                     candidate.getEpoch(), Constants.WGS84_EARTH_MU);
            KeplerianPropagator kepPropo = new KeplerianPropagator(initialOrbit);
            
            // Add event to be detected
            final EventsLogger horizonLogger = new EventsLogger();
            EventsLogger earthShadowLogger = new EventsLogger();
            kepPropo.addEventDetector(horizonLogger.monitorDetector(visibility));
            kepPropo.addEventDetector(earthShadowLogger.monitorDetector(eclipseDetector));

            // Propagate
            SpacecraftState predState = kepPropo.propagate(targetDate);
            if (eclipseDetector.g(predState)<0.) {
                // Observation cannot be performed because object in Earth shadow
                continue;
            } else if (visibility.g(predState) < 0.) {
                // Object not in FOV even though sensor was placed such that visibility is provided
                throw new IllegalArgumentException("Object is not in FOV");
            }

            // Transform spacecraft state into sensor pointing direction
            AngularDirection raDecPointing = transformStateToPointing(predState, topoInertial);
            // System.out.println("RA of " + candidate.getId() + ": " + FastMath.toDegrees(raDecPointing.getAngle1()));
            // System.out.println("DEC of " + candidate.getId() + ": "  + FastMath.toDegrees(raDecPointing.getAngle2()));

            boolean goodSolarPhase = 
                Tasking.checkSolarPhaseCondition(targetDate, raDecPointing);

            if (!goodSolarPhase) {
                // Observation cannot be performed because of lack of visibility
                continue;
            }
            double distMoon = 
                AngularDirection.computeAngularDistMoon(targetDate, topoInertial, raDecPointing);
            if (distMoon < minMoonDist) {
                // Observation cannot be performed because of lack of visibility
                continue;
            }
            double actualSlewT = 
                this.sensor.computeRepositionT(sensorPointing, raDecPointing, true);
            if(actualSlewT > TrackingObjective.allocation) {
                // not enough time to slew to target pointing direction
                continue;
            }
            
            // Generate real measurement
            //AngularDirection realRaDec = generateOneMeasurement(predState, targetDate);
            RealMatrix R = 
                MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow(1./206265, 2), 
                                                                  FastMath.pow(1./206265, 2)});
                //MatrixUtils.createRealDiagonalMatrix(new double[]{0., 0.});
            
            double[] residuals = new double[2];
            ObservedObject[] predAndCorr = 
                estimateStateWithOwnExtendedKalman(kepPropo, targetDate, R, candidate, residuals, sensor);
            double iG = computeInformationGain(predAndCorr[0], predAndCorr[1]);
            
            if (iG>maxIG) {
                maxIG = iG;
                pointing = raDecPointing;
                target = predAndCorr[1];
                //App.printCovariance(target.getCovariance().getCovarianceMatrix());
            }
        }
        // Update targeted candidate in the list of objects of interest
        for(ObservedObject candidate : updatedTargets) {
            if (Objects.isNull(target)) {
                // none of the considered taregts is observable
                return null;
            }
            if(candidate.getId() == target.getId()) {
                /* System.out.println("---");
                System.out.println("Point at " + candidate.getId());
                System.out.println(targetDate);
                System.out.println("RA [deg]: " + FastMath.toDegrees(pointing.getAngle1()));
                System.out.println("DEC [deg]: " + FastMath.toDegrees(pointing.getAngle2())); */
                candidate.setState(target.getState());
                candidate.setCovariance(target.getCovariance());
                candidate.setEpoch(targetDate);
                this.lastUpdated = candidate.getId();
                break;
            }
        }

        return pointing;
    }

    private AngularDirection generateOneMeasurement(SpacecraftState predState, AbsoluteDate targetDate) {

        Frame topoInertial = this.sensor.getTopoInertialFrame(targetDate, this.sensor.getPosition());
        Transform eciToTopo = predState.getFrame().getTransformTo(topoInertial, targetDate);
        PVCoordinates pvTopo = 
            eciToTopo.transformPVCoordinates(predState.getPVCoordinates());
        double angle1 = pvTopo.getPosition().getAlpha();
        double angle2 = pvTopo.getPosition().getDelta();
        double range = pvTopo.getPosition().getNorm();
        AngularDirection measurement = 
            new AngularDirection(topoInertial, new double[]{angle1, angle2}, 
                                 AngleType.RADEC, range);
        return measurement;
    }

    public static double computeTraceChange(double[][] covPrior, double[][] covPost) {

        // Retrieve covariances
        RealMatrix covP = new Array2DRowRealMatrix(covPrior);
        RealMatrix covQ = new Array2DRowRealMatrix(covPost);

        double change = covP.getTrace() - covQ.getTrace();
        return change;
    }
}
