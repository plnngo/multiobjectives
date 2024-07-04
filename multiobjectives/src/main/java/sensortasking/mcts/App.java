package sensortasking.mcts;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.linear.Array2DRowRealMatrix;
import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.linear.MatrixUtils;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.ode.events.Action;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.GeodeticPoint;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.errors.OrekitException;
import org.orekit.estimation.measurements.AngularAzEl;
import org.orekit.estimation.measurements.GroundStation;
import org.orekit.estimation.measurements.ObservableSatellite;
import org.orekit.estimation.measurements.ObservedMeasurement;
import org.orekit.estimation.sequential.ConstantProcessNoise;
import org.orekit.estimation.sequential.KalmanEstimator;
import org.orekit.estimation.sequential.KalmanEstimatorBuilder;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.forces.ForceModel;
import org.orekit.forces.gravity.HolmesFeatherstoneAttractionModel;
import org.orekit.forces.gravity.potential.GravityFieldFactory;
import org.orekit.forces.gravity.potential.NormalizedSphericalHarmonicsProvider;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.frames.Transform;
import org.orekit.models.earth.ReferenceEllipsoid;
import org.orekit.orbits.CartesianOrbit;
import org.orekit.orbits.KeplerianOrbit;
import org.orekit.orbits.Orbit;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.MatricesHarvester;
import org.orekit.propagation.Propagator;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.StateCovarianceMatrixProvider;
import org.orekit.propagation.analytical.KeplerianPropagator;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEConstants;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.analytical.tle.generation.FixedPointTleGenerationAlgorithm;
import org.orekit.propagation.conversion.DormandPrince853IntegratorBuilder;
import org.orekit.propagation.conversion.KeplerianPropagatorBuilder;
import org.orekit.propagation.conversion.NumericalPropagatorBuilder;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.propagation.numerical.NumericalPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

/**
 * Hello world!
 *
 */
public class App {
    public static void main( String[] args ){

        // Load orekit data
        String orekitDataDir = "multiobjectives\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        //App.testPropo2();
        //App.propagateKeplerianDynamics();
        //App.propagateCovarianceOrekitExample();
        //App.compareNormalPropWithKalmanPrediction();
        App.setUpOwnKalmanFilter();
    }

    public static void setUpOwnKalmanFilter(){
        // Frame
        Frame eci = FramesFactory.getEME2000();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Definition of initial conditions with position and velocity
        //------------------------------------------------------------
        Vector3D position = new Vector3D(7.0e6, 1.0e6, 4.0e6);
        Vector3D velocity = new Vector3D(-500.0, 8000.0, 1000.0);
        PVCoordinates pvInit = new PVCoordinates(position, velocity);
        double mu = 3.9860047e14;
        System.out.println("----- Initial condition -----");
        System.out.println("MU [m^3/s^-2]: \t" + mu);
        AbsoluteDate initDate = AbsoluteDate.J2000_EPOCH.shiftedBy(584.);
        Orbit initialOrbit = new CartesianOrbit(pvInit, eci, initDate, mu);
        System.out.println("Initial date: \t" + initDate);

        // Kalman initialisation
        RealMatrix xbar0 = 
            MatrixUtils.createColumnRealMatrix(new double[]{0., 0., 0., 0., 0., 0.});
        RealMatrix xhatPre = xbar0; 

        // Extrapolator definition
        // -----------------------
        KeplerianPropagator extrapolator = new KeplerianPropagator(initialOrbit);
        System.out.println("Propagator: \t" + extrapolator.toString());
        System.out.println("Frame: \t" + eci);
        System.out.println("Position [m]: \t" + position);
        System.out.println("Velocity [m/s]: \t" + velocity);
        System.out.println("Initial covariance: ");
        final SpacecraftState initialState = extrapolator.getInitialState();
 
        // Initial covariance
        RealMatrix covInitMatrix = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                              0.1, 0.1, 0.1});
       /*  StateCovariance covInit = 
            new StateCovariance(covInitMatrix, initDate, eci, OrbitType.CARTESIAN, 
                                PositionAngleType.MEAN); */
        final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = 
            extrapolator.setupMatricesComputation(stmAdditionalName, null, null);
        printCovariance(covInitMatrix);
        
        // Set up covariance matrix provider and add it to the propagator
/*         final StateCovarianceMatrixProvider providerCov = 
            new StateCovarianceMatrixProvider("covariance", stmAdditionalName, harvester, covInit);
        extrapolator.addAdditionalStateProvider(providerCov); */

        // State transition matrix
        final AbsoluteDate target = initialState.getDate().shiftedBy(100.);
        System.out.println("Target date: \t" + target);
        System.out.println("Propagation duration [s]: \t" + initialState.getKeplerianPeriod());
        System.out.println("----- Propagation -----");
        SpacecraftState finalOrbit = extrapolator.propagate(target);
        RealMatrix dYdY0 = harvester.getStateTransitionMatrix(finalOrbit);
        System.out.println("STM at " + finalOrbit.getDate());
        printCovariance(dYdY0);
        System.out.println("Predicted covariance through multiplication of STM with init Cov: ");
        RealMatrix predictedCov = dYdY0.multiply(covInitMatrix).multiplyTransposed(dYdY0);
        printCovariance(predictedCov);

        // Process noise
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-12, 1e-12, 1e-12});
        RealMatrix gamma = getGammaMatrix(initDate, target);
        RealMatrix mappedAcc = gamma.multiply(Q).multiplyTransposed(gamma);
        System.out.println("Process noise Q:");
        printCovariance(Q);
        System.out.println("Mapped unmodelled accelerations:");
        printCovariance(mappedAcc);
        System.out.println("Predicted covariance incl unmodelled accelerations (i.e. including process noise): ");
        predictedCov = predictedCov.add(mappedAcc);
        printCovariance(predictedCov);

        RealMatrix xbar = dYdY0.multiply(xhatPre);

/*         // Direct covariance of orekit
        System.out.println("Predicted covariance through covariance provider: ");
        printCovariance(providerCov.getStateCovariance(finalOrbit).getMatrix()); */

        // Predicted state
        Vector3D predictedPos = finalOrbit.getPVCoordinates().getPosition();
        Vector3D predictedVel = finalOrbit.getPVCoordinates().getVelocity();
        double[] dataPredictedState = 
            new double[]{predictedPos.getX(), predictedPos.getY(), predictedPos.getZ(),
                         predictedVel.getX(), predictedVel.getY(), predictedVel.getZ()};
        RealMatrix predictedStateColumnVec = new Array2DRowRealMatrix(dataPredictedState);
        System.out.println("Propagated state vector [m] or [m/s]");
        printCovariance(predictedStateColumnVec);

        // Set up station
        Transform eciToEcef = eci.getTransformTo(ecef, target);       
        PVCoordinates pvEcef = 
            eciToEcef.transformPVCoordinates(new PVCoordinates(predictedPos, predictedVel));
        double lon = pvEcef.getPosition().getAlpha();
        double lat = pvEcef.getPosition().getDelta();
       /*  System.out.println("Latitude [rad]: \t" + lat);
        System.out.println("Longitude [rad]: \t" + lon);
        System.out.println("Altitude [m]: \t" + 0.);
        System.out.println("Earth equatorial radius [m]: \t" + Constants.WGS84_EARTH_EQUATORIAL_RADIUS);
        System.out.println("Earth flattening: \t" + Constants.WGS84_EARTH_FLATTENING); */

        GeodeticPoint station = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "New Station");
        Transform horizonToEci = topoHorizon.getTransformTo(eci, target);  
        Vector3D coordinatesStationEci = horizonToEci.transformPosition(Vector3D.ZERO);
/*         System.out.println(FastMath.toDegrees(coordinatesStationEci.getAlpha()));
        System.out.println(FastMath.toDegrees(coordinatesStationEci.getDelta()));
        System.out.println(FastMath.toDegrees(lat));
        System.out.println(FastMath.toDegrees(lon)); */

        Transform eciToTopo = new Transform(target, coordinatesStationEci.negate());
        Frame topoCentric = new Frame(eci, eciToTopo, "Topocentric", true);

        // Generate real measurement
        PVCoordinates pvTopo = 
            eciToTopo.transformPVCoordinates(new PVCoordinates(predictedPos, predictedVel));
        double ra = pvTopo.getPosition().getAlpha();
        double dec = pvTopo.getPosition().getDelta();
        AngularDirection realRaDec = new AngularDirection(topoCentric, new double[]{ra, dec}, AngleType.RADEC);
        RealMatrix R = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{FastMath.pow(1./206265, 2), 
                                                              FastMath.pow(1./206265, 2)});
        System.out.println("----- Measurement update -----");
        System.out.println("Measurement frame: \t" + realRaDec.getFrame());
        System.out.println("Measured right ascension [rad]: \t" + ra);
        System.out.println("Measured declination [rad]: \t" + dec);
        System.out.println("Measurement epoch: \t" + target);

  

        Vector3D posTopo = predictedPos.subtract(coordinatesStationEci);
        AngularDirection radec = predictMeasurement(posTopo, topoCentric); 
        // AngularDirection predictRaDec = 
        //     radec.transformReference(topoCentric, initDate, AngleType.RADEC);
        RealMatrix H = getObservationPartialDerivative(posTopo, false);
        System.out.println("Observation matrix H:");
        printCovariance(H);

        // Measurement error
        AngularDirection residuals = realRaDec.substract(radec);
        double[][] residualsArray = new double[2][1];
        residualsArray[0] = new double[]{residuals.getAngle1()};
        residualsArray[1] = new double[]{residuals.getAngle2()};
        RealMatrix residualMatrix = MatrixUtils.createRealMatrix(residualsArray);
        
        // Compute Kalman Gain
        RealMatrix covInMeasSpace = H.multiply(predictedCov).multiplyTransposed(H);
        RealMatrix kalmanGain = predictedCov.multiplyTransposed(H)
                                            .multiply(MatrixUtils.inverse(covInMeasSpace.add(R)));
        System.out.println("Kalman Gain");
        printCovariance(kalmanGain);

        // Correction
        RealMatrix xhat = xbar.add(kalmanGain.multiply(residualMatrix.subtract(H.multiply(xbar))));
        RealMatrix updatedState = predictedStateColumnVec.add(xhat);

        RealMatrix kRkT = kalmanGain.multiply(R).multiplyTransposed(kalmanGain);
        RealMatrix identity = MatrixUtils.createRealIdentityMatrix(6);
        RealMatrix iMinusKgH = identity.subtract(kalmanGain.multiply(H));
        RealMatrix updatedCov = 
            iMinusKgH.multiply(predictedCov).multiplyTransposed(iMinusKgH).add(kRkT); // add process noise to predicted cov

        System.out.println("---- After Kalman update: -----");
        System.out.println("Frame: \t" + topoCentric);
        System.out.println("Date: \t" + target);
        System.out.println("Updated state:");
        printCovariance(updatedState);
        System.out.println("Updated state covariance");
        printCovariance(updatedCov);
    }

    public static RealMatrix getGammaMatrix(AbsoluteDate initDate, AbsoluteDate target) {
        RealMatrix gamma = MatrixUtils.createRealMatrix(6, 3);
        double deltaT = target.durationFrom(initDate);
        for(int i=0; i<gamma.getColumnDimension(); i++ ) {
            gamma.setEntry(i, i, FastMath.pow(deltaT, 2)/2);
            gamma.setEntry(i+3, i, deltaT);
        }
/*         System.out.println("Gamma");
        printCovariance(gamma); */
        return gamma;
    }

    public static RealMatrix getObservationPartialDerivative(Vector3D posTopo, boolean withRange) {
        double range = posTopo.getNorm();
        
        double h11 = posTopo.getX()/range;
        double h12 = posTopo.getY()/range;
        double h13 = posTopo.getZ()/range;
        double h21 = -posTopo.getY()/(FastMath.pow(posTopo.getX(), 2) 
                        * (1 + FastMath.pow(posTopo.getY()/posTopo.getX(), 2)));
        double h22 = 1./(posTopo.getX() * (1 + FastMath.pow(posTopo.getY()/posTopo.getX(), 2)));
        double h31 = - posTopo.getZ() * posTopo.getX()
                        /(FastMath.pow(range, 3) 
                            * FastMath.sqrt(1 - FastMath.pow(posTopo.getZ()/range,2)));
        double h32 = - posTopo.getZ() * posTopo.getY() 
                        /(FastMath.pow(range, 3) 
                            * FastMath.sqrt(1 - FastMath.pow(posTopo.getZ()/range,2)));
        double h33 = (1/range - FastMath.pow(posTopo.getZ(),2)/FastMath.pow(range, 3))
                        / FastMath.sqrt(1 - FastMath.pow(posTopo.getZ()/range, 2));
/*         System.out.println("H21: " + h21);
        System.out.println("H22: " + h22);
        System.out.println("H31: " + h31);
        System.out.println("H32: " + h32);
        System.out.println("H33: " + h33); */
        if (withRange) {
            double[][] data = new double[3][6];
            data[0] = new double[]{h11, h12, h13, 0., 0., 0.};
            data[1] = new double[]{h21, h22, 0., 0., 0., 0.};
            data[2] = new double[]{h31, h32, h33, 0., 0., 0.};
            RealMatrix obsPartialDeriv = MatrixUtils.createRealMatrix(data);
            return obsPartialDeriv;
        } else {
            double[][] data = new double[2][6];
            data[0] = new double[]{h21, h22, 0., 0., 0., 0.};
            data[1] = new double[]{h31, h32, h33, 0., 0., 0.};
            RealMatrix obsPartialDeriv = MatrixUtils.createRealMatrix(data);
            return obsPartialDeriv;
        }
    }

    public static AngularDirection predictMeasurement(Vector3D posTopo, Frame frame) {
    
        // Observation parameters
        double range = posTopo.getNorm();
        double ra = FastMath.atan2(posTopo.getY(), posTopo.getX());
        double dec = FastMath.asin(posTopo.getZ()/range);

        AngularDirection raDec = new AngularDirection(frame, new double[]{ra, dec}, AngleType.RADEC);
        return raDec;
    }

    public static void compareNormalPropWithKalmanPrediction(){
        // Frame
        Frame eme = FramesFactory.getEME2000();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);


        // Definition of initial conditions with position and velocity
        //------------------------------------------------------------
        Vector3D position = new Vector3D(7.0e6, 1.0e6, 4.0e6);
        Vector3D velocity = new Vector3D(-500.0, 8000.0, 1000.0);
        PVCoordinates pvInit = new PVCoordinates(position, velocity);
        double mu = 3.9860047e14;

        AbsoluteDate initDate = AbsoluteDate.J2000_EPOCH.shiftedBy(584.);
        Orbit initialOrbit = new CartesianOrbit(pvInit, eme, initDate, mu);

        // Extrapolator definition
        // -----------------------
        KeplerianPropagator extrapolator = new KeplerianPropagator(initialOrbit);

         // Initial covariance
        RealMatrix covInitMatrix = 
        MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                           0.1, 0.1, 0.1});
        StateCovariance covInit = new StateCovariance(covInitMatrix, initDate, eme, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = 
            extrapolator.setupMatricesComputation(stmAdditionalName, null, null);

     // Set up covariance matrix provider and add it to the propagator
        final StateCovarianceMatrixProvider providerCov = 
            new StateCovarianceMatrixProvider("covariance", stmAdditionalName, harvester, covInit);
        extrapolator.addAdditionalStateProvider(providerCov);


        // Extrapolation at a final date different from initial date
        // ---------------------------------------------------------
        double delta_t = 100000.0; // extrapolation duration in seconds
        //double delta_t = 0.0; // extrapolation duration in seconds

        AbsoluteDate extrapDate = initDate.shiftedBy(delta_t);
        SpacecraftState finalOrbit = extrapolator.propagate(extrapDate);
        PVCoordinates pv = finalOrbit.getPVCoordinates();
        System.out.println("Initial scenario at " + initDate.toString());
        System.out.println(extrapolator.getInitialState().getPVCoordinates());
        System.out.println("Initial covariance ");
        printCovariance(covInitMatrix);
        System.out.println("propagated scenario at " + extrapDate.toString());
        
        System.out.println(pv);
        System.out.println("propagated covariance");
        printCovariance(providerCov.getStateCovariance(finalOrbit).getMatrix());


        Transform emeToEcef = eme.getTransformTo(ecef, extrapDate);
        PVCoordinates pvEcef = emeToEcef.transformPVCoordinates(pv);
        double lon = pvEcef.getPosition().getAlpha();
        double lat = pvEcef.getPosition().getDelta();

        GeodeticPoint station = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "New Station");

        Transform ecefToHorizon = ecef.getTransformTo(topoHorizon, extrapDate);
        PVCoordinates pvHorizon = ecefToHorizon.transformPVCoordinates(pvEcef);
        Vector3D posHorizon = pvHorizon.getPosition();
        //System.out.println("El: :" + FastMath.toDegrees(posHorizon.getDelta()));

        List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
        AngularAzEl azElOrekit = 
            new AngularAzEl(new GroundStation(topoHorizon), extrapDate, 
                            new double[]{posHorizon.getAlpha(), posHorizon.getDelta()}, 
                            new double[]{1e-3, 1e-3}, 
                            new double[]{1., 1.},
                            new ObservableSatellite(0));
        orekitAzElMeas.add(azElOrekit);


        callKeplerianKalman(initialOrbit, extrapDate, covInitMatrix, orekitAzElMeas);
    }

    public static void callKeplerianKalman(Orbit initialOrbit, AbsoluteDate date, RealMatrix covInitMatrix, Iterable<ObservedMeasurement<?>> orekitAzElMeas){
       /* Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
 
        // Ground station
        PVCoordinates posEcef = new PVCoordinates(new Vector3D(-5466.071, -2403.990, 2242.473));
        double lon = posEcef.getPosition().getAlpha();
        double lat = posEcef.getPosition().getDelta();
        GeodeticPoint station = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "Steve's Station");

        List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
        AngularAzEl azElOrekit = 
            new AngularAzEl(new GroundStation(topoHorizon), date, 
                            new double[]{0., 0.}, 
                            new double[]{1e-3, 1e-3}, 
                            new double[]{1., 1.},
                            new ObservableSatellite(0));
        orekitAzElMeas.add(azElOrekit); */

        KeplerianPropagatorBuilder builder =  new KeplerianPropagatorBuilder(initialOrbit, PositionAngleType.MEAN, 1.);
        // Build Kalman filter
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8});
        Q = Q.scalarMultiply(0.);
        //covInitMatrix = covInitMatrix.scalarMultiply(0.);
        KalmanEstimator kalman = 
            new KalmanEstimatorBuilder()
                .addPropagationConfiguration(builder, new ConstantProcessNoise(covInitMatrix, Q))
                .build();
        Propagator[] estimated = kalman.processMeasurements(orekitAzElMeas);
        System.out.println("Kalman Estimator results at " + kalman.getCurrentDate());
        System.out.println(kalman.getPhysicalEstimatedState());
        System.out.println("updated covariance: ");
        printCovariance(kalman.getPhysicalEstimatedCovarianceMatrix());

    }
    public static void propagateKeplerianDynamics() {

        AbsoluteDate date = new AbsoluteDate();
        Frame eci = FramesFactory.getGCRF();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        double MU = 3.986004415 * 1e14;     // in m^3/s^(-2)

        // Initial scenario
        PVCoordinates pv = new PVCoordinates(new Vector3D(757700., 5222607., 4851500.), 
                                             new Vector3D(2213.21, 4678.34, -5371.30));
        CartesianOrbit orbit = new CartesianOrbit(pv, eci, date, MU);
        final SpacecraftState stateInit = new SpacecraftState(orbit);

        // Initial covariance
        RealMatrix covInitMatrix = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{100*1e3, 100*1e3, 100*1e3, 
                                                              0.1, 0.1, 0.1});
        StateCovariance covInit = new StateCovariance(covInitMatrix, date, eci, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        System.out.println("INITIAL_CART_COVARIANCE " + date.toString() + " EXPRESSED_IN " + covInit.getFrame().getName() + " FRAME");

        printCovariance(covInitMatrix);

        // Ground station
        PVCoordinates posEcef = new PVCoordinates(new Vector3D(-5466.071, -2403.990, 2242.473));
        double lon = posEcef.getPosition().getAlpha();
        double lat = posEcef.getPosition().getDelta();
        GeodeticPoint station = new GeodeticPoint(lat,   // Geodetic latitude
                                              lon,   // Longitude
                                              0.);              // in [m]
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "Steve's Station");

        // Generate measurement
        final AbsoluteDate targetEpoch = date.shiftedBy(Constants.JULIAN_DAY);
        List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
        AngularAzEl azElOrekit = 
            new AngularAzEl(new GroundStation(topoHorizon), date, 
                            new double[]{0., 0.}, 
                            new double[]{1e-3, 1e-3}, 
                            new double[]{1., 1.},
                            new ObservableSatellite(0));
        orekitAzElMeas.add(azElOrekit);

        KeplerianPropagatorBuilder builder =  new KeplerianPropagatorBuilder(orbit, PositionAngleType.MEAN, 1.);
        /* KeplerianPropagator extrapolator = new KeplerianPropagator(orbit);
        SpacecraftState state = extrapolator.propagate(date); */


        // Build Kalman filter
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8});
        Q = Q.scalarMultiply(0.);
        covInitMatrix = covInitMatrix.scalarMultiply(0.);
        KalmanEstimator kalman = 
            new KalmanEstimatorBuilder()
                .addPropagationConfiguration(builder, new ConstantProcessNoise(covInitMatrix, Q))
                .build();
        Propagator[] estimated = kalman.processMeasurements(orekitAzElMeas);
        System.out.println(estimated.length);
        System.out.println(estimated[estimated.length -1]);
    }

    public static void propagateCovarianceOrekitExample() {

        // Frame
        //Frame teme = FramesFactory.getTEME();
        Frame e2000 = FramesFactory.getEME2000();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        double MU = 3.986004415 * 1e14;     // in m^3/s^(-2)

        // Ground station
        GeodeticPoint station = new GeodeticPoint(FastMath.toRadians(39.007),   // Geodetic latitude
                                              FastMath.toRadians(-104.883),   // Longitude
                                              2194.56);              // in [m]
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topoHorizon = new TopocentricFrame(earth, station, "United States Air Force Academy");

        // Initiate scenario
        AbsoluteDate date = new AbsoluteDate("2016-02-13T16:00:00.0", TimeScalesFactory.getUTC());
        Vector3D pos = new Vector3D(7526.993581890527 * 1e3, 
                                    -9646.310100269711 * 1e3, 
                                    1464.1104928112086 * 1e3);
        Vector3D vel = new Vector3D(3.0337945609969803 * 1e3, 
                                    1.715265069098717 * 1e3, 
                                    -4.447658745923896 * 1e3);
        PVCoordinates pv = new PVCoordinates(pos, vel);
        CartesianOrbit orbit = new CartesianOrbit(pv, e2000, date, MU);
        final SpacecraftState stateInit = new SpacecraftState(orbit);

        // Initial covariance
        double[][] covInitArray = new double[6][6];
        covInitArray[0] = new double[]{8.651816029065393E+1, 5.68998712665727E+1, -2.763870763721462E+1, 
                                       -2.435617200936485E-2, 2.0582741368923926E-2, -5.872883050786668E-3};
        covInitArray[1] = new double[]{5.68998712665727E+1, 7.070624320814591E+1, 1.3671209089262682E+1,
                                       -6.112622012706486E-3, 7.623626007527378E-3, -1.2394131901568663E-2};
        covInitArray[2] = new double[]{-2.763870763721462E+1, 1.3671209089262682E+1, 1.811858898030072E+2,
                                       3.143798991624274E-2, -4.963106558582378E-2, -7.420114384979073E-4};
        covInitArray[3] = new double[]{-2.435617200936485E-2, -6.112622012706486E-3, 3.143798991624274E-2,
                                       4.65707738862789E-5, 1.4699436343326518E-5, 3.3284755933116514E-5};
        covInitArray[4] = new double[]{2.0582741368923926E-2, 7.623626007527378E-3, -4.963106558582378E-2,
                                       1.4699436343326518E-5, 3.950715933635926E-5, 2.516044257517839E-5};
        covInitArray[5] = new double[]{-5.872883050786668E-3, -1.2394131901568663E-2, -7.420114384979073E-4,
                                       3.3284755933116514E-5, 2.516044257517839E-5, 3.54746611996909E-5};
        RealMatrix covInitMatrix = MatrixUtils.createRealMatrix(covInitArray);
        StateCovariance covInit = new StateCovariance(covInitMatrix, date, e2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        System.out.println("INITIAL_CART_COVARIANCE " + date.toString() + " EXPRESSED_IN " + covInit.getFrame().getName() + " FRAME");

        printCovariance(covInitMatrix);

        // Create numerical propagator
        final double    minStep           = 0.001;
        final double    maxStep           = 1000.0;
        final double    positionTolerance = 10.0;
        final OrbitType orbitType         = OrbitType.CARTESIAN;
        final double    dP                = 1.;
/*        final double[][] tol = NumericalPropagator.tolerances(positionTolerance, orbit, orbitType);
         final ODEIntegrator integrator = 
            new DormandPrince853Integrator(minStep, maxStep, tol[0], tol[1]); */
        NumericalPropagatorBuilder propagatorBuilder =
            new NumericalPropagatorBuilder(orbitType.convertType(orbit),
                                        new DormandPrince853IntegratorBuilder(minStep, maxStep, dP),
                                        PositionAngleType.MEAN, dP);
        final NormalizedSphericalHarmonicsProvider provider = 
            GravityFieldFactory.getNormalizedProvider(2, 0);
        final ForceModel holmesFeatherstone = 
            new HolmesFeatherstoneAttractionModel(ecef, provider);
        propagatorBuilder.addForceModel(holmesFeatherstone);
        NumericalPropagator propagator = propagatorBuilder.buildPropagator(new double[6]);
/*         final NumericalPropagator propagator = new NumericalPropagator(integrator);
        propagator.setInitialState(stateInit);
        propagator.setOrbitType(orbitType);
        final NormalizedSphericalHarmonicsProvider provider = 
            GravityFieldFactory.getNormalizedProvider(2, 0);
        final ForceModel holmesFeatherstone = 
            new HolmesFeatherstoneAttractionModel(ecef, provider);
        propagator.addForceModel(holmesFeatherstone); */

        // Set up computation of State Transition Matrix and Jacobians matrix with respect to parameters
        final String stmAdditionalName = "stm";
        final MatricesHarvester harvester = 
            propagator.setupMatricesComputation(stmAdditionalName, null, null);

        // Set up covariance matrix provider and add it to the propagator
        final StateCovarianceMatrixProvider providerCov = 
            new StateCovarianceMatrixProvider("covariance", stmAdditionalName, harvester, covInit);
        propagator.addAdditionalStateProvider(providerCov);

        // Generate measurement
        /* final AbsoluteDate targetEpoch = date.shiftedBy(Constants.JULIAN_DAY);
        List<ObservedMeasurement<?>> orekitAzElMeas = new ArrayList<>();
        AngularAzEl azElOrekit = 
            new AngularAzEl(new GroundStation(topoHorizon), targetEpoch, 
                            new double[]{0., 0.}, 
                            new double[]{1e-3, 1e-3}, 
                            new double[]{1., 1.},
                            new ObservableSatellite(0));
        orekitAzElMeas.add(azElOrekit);

        // Build Kalman filter
        RealMatrix Q = 
            MatrixUtils.createRealDiagonalMatrix(new double[]{1e-8, 1e-8, 1e-8, 1e-8, 1e-8, 1e-8});
        KalmanEstimator kalman = 
            new KalmanEstimatorBuilder()
                .addPropagationConfiguration(propagatorBuilder, new ConstantProcessNoise(covInitMatrix, Q))
                .build();
        Propagator[] estimated = kalman.processMeasurements(orekitAzElMeas); */

        // Perform propagation
        final AbsoluteDate targetEpoch = date.shiftedBy(Constants.JULIAN_DAY);
        SpacecraftState finalState = propagator.propagate(targetEpoch);
        RealMatrix covProp = providerCov.getStateCovariance(finalState).getMatrix();    // in EME2000
        System.out.println("PROPAGATED_CART_COVARIANCE " + targetEpoch.toString() + " EXPRESSED_IN " + finalState.getFrame().getName() + " FRAME");
        printCovariance(covProp);
        System.out.println("Final state: " + finalState.getPVCoordinates());



        /* // Propagate from start of window to last measurement epoch without measurement update
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
                                       stationFrame, pseudoTle); */

    }

    public static void printCovariance(final RealMatrix covariance) {

        // Create a string builder
        final StringBuilder covToPrint = new StringBuilder();
        for (int row = 0; row < covariance.getRowDimension(); row++) {
            for (int column = 0; column < covariance.getColumnDimension(); column++) {
                covToPrint.append(String.format(Locale.US, "%16.9e", covariance.getEntry(row, column)));
                covToPrint.append(" ");
            }
            covToPrint.append("\n");
        }

        // Print
        System.out.println(covToPrint);

    }
    public void testFrameConversion() {
        // Load orekit data
        String orekitDataDir = "multiobjectives\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        // Create frames
        Frame eci = FramesFactory.getGCRF();
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        Frame teme = FramesFactory.getTEME();

        // Set up input data from Vallado
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        PVCoordinates pvEci = 
            new PVCoordinates(new Vector3D(-605792.21660, -5870229.51108, 3493053.19896),
                              new Vector3D(-1568.25429, -3702.34891, -6479.48395));

        // Transform from ECI to TEME and ECEF
        Transform eciToTeme = eci.getTransformTo(teme, date);
        PVCoordinates pvTeme = eciToTeme.transformPVCoordinates(pvEci);
        System.out.println("State in TEME using Transform");
        System.out.println(pvTeme.toString());

        CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        SpacecraftState scStateTeme = new SpacecraftState(orbit);
        StateVector stateTeme = ObservedObject.spacecraftStateToStateVector(scStateTeme, teme);
        System.out.println("State in TEME using spacecraftStateToStateVector");
        System.out.println(stateTeme.getPositionVector());
        System.out.println(stateTeme.getVelocityVector());

        TLE testTle = new TLE("1 55586U 23020T   23336.86136309  .00002085  00000-0  16934-3 0  9990", 
                           "2 55586  43.0014 182.7273 0001366 277.9331  82.1357 15.02547145 44391");
        //LeastSquaresTleGenerationAlgorithm converter = new LeastSquaresTleGenerationAlgorithm();
        //TLE pseudoTle = TLE.stateToTLE(scStateTeme, testTle, converter);   //TODO: check if SGP4 model is applied or if the state vector is still the same
        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(scStateTeme, testTle);

 /*        // Retrieve semi major axis from TLE
        double n = pseudoTle.getMeanMotion();
        double a = FastMath.pow(TLEConstants.MU, 1./3.)/(FastMath.pow(n, 2./3.));

        KeplerianOrbit kepOrbitFromTle = 
            new KeplerianOrbit(a, pseudoTle.getE(), pseudoTle.getI(), 
                               pseudoTle.getPerigeeArgument(), pseudoTle.getRaan(), 
                               pseudoTle.getMeanAnomaly(), PositionAngleType.MEAN, teme, date, 
                               TLEConstants.MU);
        System.out.println("State in TEME using stateToTLE");
        System.out.println(kepOrbitFromTle.getPVCoordinates().getPosition());
        System.out.println(kepOrbitFromTle.getPVCoordinates().getVelocity()); */

        //SGP4 Propagator 
        TLEPropagator tleProp = TLEPropagator.selectExtrapolator(pseudoTle);
        SpacecraftState stateProp = tleProp.propagate(date);
        System.out.println("State in TEME after propagation");
        System.out.println(stateProp.getPVCoordinates().getPosition());
        System.out.println(stateProp.getPVCoordinates().getVelocity());
        System.out.println("State in TEME before propagation");
        System.out.println(tleProp.getInitialState().getPVCoordinates().getPosition());
        System.out.println(tleProp.getInitialState().getPVCoordinates().getVelocity());
    }

    public static void testPropo() {
        try {

            // Test TLE
            TLE iss = new TLE("1 25544U 98067A   24142.35003124  .00025335  00000-0  42455-3 0  9992",
                            "2 25544  51.6384  88.3676 0003163 192.6310 305.1222 15.51672766454387");

            //  Initial state definition : date, orbit
            final TimeScale utc = TimeScalesFactory.getUTC();
            
            final AbsoluteDate initialDate = iss.getDate();
/*             final double mu =  3.986004415e+14; // gravitation coefficient
            final Frame inertialFrame = FramesFactory.getEME2000(); // inertial frame for orbit definition
            final Vector3D position  = new Vector3D(-6142438.668, 3492467.560, -25767.25680);
            final Vector3D velocity  = new Vector3D(505.8479685, 942.7809215, 7435.922231);
            final PVCoordinates pvCoordinates = new PVCoordinates(position, velocity);
            final Orbit initialOrbit = new KeplerianOrbit(pvCoordinates, inertialFrame, initialDate, mu); */

            // Propagator : consider a simple Keplerian motion (could be more elaborate)
            //final Propagator kepler = new KeplerianPropagator(initialOrbit);
            TLEPropagator tleProp = TLEPropagator.selectExtrapolator(iss);

            // Earth and frame
            final Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
            final BodyShape earth = ReferenceEllipsoid.getWgs84(ecef);

            // Station
            final double longitude = FastMath.toRadians(-104.883);
            final double latitude  = FastMath.toRadians(39.007);
            final double altitude  = 2194.56;
            final GeodeticPoint station1 = new GeodeticPoint(latitude, longitude, altitude);
            final TopocentricFrame sta1Frame = new TopocentricFrame(earth, station1, "station1");

            // Covariance
            final RealMatrix covInitMatrix = new DiagonalMatrix(new double[]{10., 10., 10., 100., 100., 100.});
            final StateCovariance covInit = new StateCovariance(covInitMatrix, initialDate, FramesFactory.getTEME(), OrbitType.CARTESIAN, PositionAngleType.MEAN);
            final String stmAdditionalName = "stm";
            final MatricesHarvester harvester = tleProp.setupMatricesComputation(stmAdditionalName, null, null);

            final StateCovarianceMatrixProvider provider = 
                new StateCovarianceMatrixProvider("cov", stmAdditionalName, harvester, covInit);

            tleProp.addAdditionalStateProvider(provider);

            // Event definition
            final double maxcheck  = 60.0;
            final double threshold =  0.001;
            final double elevation = FastMath.toRadians(5.0);
            
            final EventDetector sta1Visi =
                    new ElevationDetector(maxcheck, threshold, sta1Frame).
                    withConstantElevation(elevation).
                    withHandler((s, d, increasing) -> {
                        System.out.println(" Visibility on " +
                                            ((ElevationDetector) d).getTopocentricFrame().getName() +
                                           (increasing ? " begins at " : " ends at ") +
                                           s.getDate().toStringWithoutUtcOffset(utc, 3));
                        
                        return increasing ? Action.CONTINUE : Action.STOP;
                    });

            // Add event to be detected
            final EventsLogger logger = new EventsLogger();
            tleProp.addEventDetector(logger.monitorDetector(sta1Visi));

            // Propagate from the initial date to the first raising or for the fixed duration
            AbsoluteDate targetEpoch = initialDate.shiftedBy(60.*60.*24);
            targetEpoch = new AbsoluteDate(2024, 5, 21, 21, 56, 0, utc);
            final SpacecraftState finalState = tleProp.propagate(targetEpoch);

            System.out.println(" Final state duration from initial date (s) : " + finalState.getDate().durationFrom(initialDate));
            

             for (LoggedEvent le : logger.getLoggedEvents()) {
                if (le.isIncreasing()) {
                    System.out.println("Enter FOR " + le.getDate());
                } else {
                    System.out.println("Exit FOR " + le.getDate());;
                }
                System.out.println("Propagate the covariance to " + le.getDate().toString() + " ...");
                final RealMatrix covProp = provider.getStateCovariance(le.getState()).getMatrix();
                System.out.println(covProp.toString());
            }
        } catch (OrekitException oe) {
            System.err.println(oe.getLocalizedMessage());
        }
    }

    public static void testPropo2() {
        
        // Create frames
        Frame eci = FramesFactory.getGCRF();
        Frame teme = FramesFactory.getTEME();

        // Set up input data from Vallado
        AbsoluteDate date = 
            new AbsoluteDate(2000, 12, 15, 16, 58, 50.208, TimeScalesFactory.getUTC());
        PVCoordinates pvEci = 
            new PVCoordinates(new Vector3D(-605792.21660, -5870229.51108, 3493053.19896),
                                new Vector3D(-1568.25429, -3702.34891, -6479.48395));
        
        // Transform from ECI to TEME and ECEF
        Transform eciToTeme = eci.getTransformTo(teme, date);
        PVCoordinates pvTeme = eciToTeme.transformPVCoordinates(pvEci);
        //CartesianOrbit orbit = new CartesianOrbit(pvTeme, teme, date, Constants.IERS2010_EARTH_MU);
        
        KeplerianOrbit kep = new KeplerianOrbit(pvTeme, teme, date, TLEConstants.MU);
        SpacecraftState scStateTeme = new SpacecraftState(kep);
        System.out.println(scStateTeme.getPVCoordinates());
        final TLE tleISS = new TLE("1 25544U 98067A   21035.14486477  .00001026  00000-0  26816-4 0  9998",
                                "2 25544  51.6455 280.7636 0002243 335.6496 186.1723 15.48938788267977");
        double n = FastMath.sqrt(TLEConstants.MU/FastMath.pow(kep.getA(), 3));
        TLE tle = new TLE(tleISS.getSatelliteNumber(), tleISS.getClassification(), 
                            tleISS.getLaunchYear(), tleISS.getLaunchNumber(), tleISS.getLaunchPiece(), 
                            tleISS.getEphemerisType(), tleISS.getElementNumber(), date, n, 
                            tleISS.getMeanMotionFirstDerivative(), tleISS.getMeanMotionSecondDerivative(), 
                            kep.getE(), kep.getI(), kep.getPerigeeArgument(), 
                            kep.getRightAscensionOfAscendingNode(), kep.getMeanAnomaly(), 
                            tleISS.getRevolutionNumberAtEpoch(), tleISS.getBStar(), 
                            TimeScalesFactory.getUTC());

        TLE pseudoTle = new FixedPointTleGenerationAlgorithm().generate(scStateTeme, tle);
        //LeastSquaresTleGenerationAlgorithm converter = new LeastSquaresTleGenerationAlgorithm();
        //TLE pseudoTle = TLE.stateToTLE(scStateTeme, tle, converter);
        TLEPropagator tleProp = TLEPropagator.selectExtrapolator(pseudoTle);
        //System.out.println(tle.toString());
        System.out.println(pseudoTle.toString());

        // Covariance
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
        
        // final RealMatrix covInitMatrix = new DiagonalMatrix(new double[]{10., 10., 10., 100., 100., 100.});
        // final StateCovariance covInit = new StateCovariance(covInitMatrix, date, teme, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        final String stmAdditionalName = "STM";
        final MatricesHarvester harvester = tleProp.setupMatricesComputation(stmAdditionalName, null, null);

        final StateCovarianceMatrixProvider provider = 
            new StateCovarianceMatrixProvider("cartCov", stmAdditionalName, harvester, covTeme);

        tleProp.addAdditionalStateProvider(provider);
        SpacecraftState s = tleProp.propagate(date.shiftedBy(0.));
        StateVector vec = ObservedObject.spacecraftStateToStateVector(s, teme);
        System.out.println(vec.getPositionVector());
        System.out.println(vec.getVelocityVector());
        StateCovariance covProp = provider.getStateCovariance(s);
        RealMatrix covPropMatrix = covProp.getMatrix();
        for (int row=0; row<covPropMatrix.getRowDimension(); row++) {
            double[] rowVec = covPropMatrix.getRow(row);
            for(int col=0; col<covPropMatrix.getColumnDimension(); col++) {
                System.out.print(rowVec[col] + " ");
            }
            System.out.println();
        }
    }
}
