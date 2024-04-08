package sensortasking.mcts;

import java.util.List;

import org.hipparchus.analysis.polynomials.PolynomialFunction;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.BodyShape;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.bodies.OneAxisEllipsoid;
import org.orekit.frames.Frame;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.propagation.events.EclipseDetector;
import org.orekit.propagation.events.EventsLogger;
import org.orekit.propagation.events.handlers.ContinueOnEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;
import org.orekit.utils.PVCoordinates;

import lombok.Getter;
import sensortasking.stripescanning.Tasking;

@Getter
public class SpatialDensityModel {

    /** Reference sensor. */
    Sensor sensor;

    /** Reference epoch. */
    AbsoluteDate epoch;

    /** Topocentric horizontal frame with respect to reference sensor and epoch. */
    TopocentricFrame topoHorizon;

    /** Generated density model. */
    int[][] densityModel;

    /**
     * Constructor.
     * 
     * @param sensor            Reference sensor.
     * @param epoch             Reference epoch.
     */
    public SpatialDensityModel(Sensor sensor, AbsoluteDate epoch) {

        // Assign reference sensor and epoch
        this.sensor = sensor;
        this.epoch = epoch;

        // Set up topocentric horizon frame 
        Frame earthFrame = FramesFactory.getITRF(IERSConventions.IERS_2010, true);
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               earthFrame);
        this.topoHorizon = new TopocentricFrame(earth, sensor.getPosition(), "station");

        // Discretise sensor's field of regard
        double height = sensor.getFov().getHeight();        // in [rad]
        double width = sensor.getFov().getWidth();          // in [rad]
        double cutOffEl = sensor.getElevCutOff();           // in [rad]

        int numPatchesEl = (int) Math.ceil(((FastMath.PI/2) - cutOffEl) / height);  // Number of patches along elevation axis
        int numPatchesAz = (int) Math.ceil((FastMath.PI*2)  / width);               // Number of patches along azimuth axis
        
        this.densityModel = new int[numPatchesEl][numPatchesAz];
    }
    
    /**
     * Generate density model of space environment with respect to {@link #sensor} and 
     * {@link #epoch}. 
     * 
     * @param tleSeries         Series of TLEs related to objects in space environment.
     * @param startObs          Epoch at the start of the observation campaign. Necessary in order
     *                          to avoid looking at the moon during the entire observation interval.
     * @param endObs            Epoch at the end of the observation campaign. Necessary in order
     *                          to avoid looking at the moon during the entire observation interval.
     * @return                  2D integer array representing spatial density model. 
     */
    protected int[][] createDensityModel(List<TLE> tleSeries, AbsoluteDate startObs, AbsoluteDate endObs){

        // Get celestial bodies
        CelestialBody sun = CelestialBodyFactory.getSun();
        OneAxisEllipsoid obateEarth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                                           Constants.WGS84_EARTH_FLATTENING,
                                                           FramesFactory.getITRF(IERSConventions.IERS_2010,
                                                                                 true));

        // Zero out position of Moon
        zeroOutRegionMoonInDensityModel(startObs, endObs);

        // Propagate TLE series to reference epoch
        for(TLE tle : tleSeries) {
            EventsLogger logger = new EventsLogger();

            // set up eclipse detector
            EclipseDetector detector = new EclipseDetector(sun, Constants.SUN_RADIUS, obateEarth)
                                            .withMaxCheck(60.0)
                                            .withThreshold(1.0e-3)
                                            .withHandler(new ContinueOnEvent<>())
                                            .withUmbra();

            TLEPropagator propagator = TLEPropagator.selectExtrapolator(tle);
            propagator.addEventDetector(logger.monitorDetector(null));
            SpacecraftState finalState = propagator.propagate(epoch);       // state in TEME


            PVCoordinates pvTeme = finalState.getPVCoordinates();
            PVCoordinates pvTopoHorizon = finalState.getFrame()
                                                    .getTransformTo(this.topoHorizon, epoch)
                                                    .transformPVCoordinates(pvTeme);

            // Extract angular position of space object 
            double azimuth = pvTopoHorizon.getPosition().getAlpha();        // between -Pi and +Pi
            double elevation = pvTopoHorizon.getPosition().getDelta();      // between -Pi/2 and +Pi/2
            AngularDirection azEl = new AngularDirection(topoHorizon, 
                                                         new double[]{azimuth, elevation}, 
                                                         AngleType.AZEL);
                                                    
            

            // Extract indices in spatial density 2D array
            int[] indices = angularDirectionToGridPosition(azEl);
            int row = indices[0];
            int col = indices[1];

            // Check if spacecraft is inside earth shadow or outside favourble solar phase angle condition
            if (detector.g(finalState)<0.0 || !Tasking.checkSolarPhaseCondition(epoch, azEl)){
                // object entered earth shadow or its phase angle is too large
                densityModel[row][col] = -1;
            } else {
                // object outside earth shadow
                // Increment number of object at corresponding patch position in spatial density model
                densityModel[row][col]++;
            }
        }
        return densityModel;
    }


    protected void zeroOutRegionMoonInDensityModel(AbsoluteDate startObs, AbsoluteDate endObs) {

        // Build moon
        CelestialBody moon = CelestialBodyFactory.getMoon();

        double fovHeight = sensor.getFov().getHeight();
        int halfMoonDist = (int) FastMath.ceil(FastMath.toRadians(10.)/fovHeight);

        // Get row and col position of Moon at the start of observation campaign
        double moonAzStart = moon.getPVCoordinates(startObs, topoHorizon).getPosition().getAlpha();
        double moonElStart = moon.getPVCoordinates(startObs, topoHorizon).getPosition().getDelta();
        AngularDirection moonAzElStart = new AngularDirection(topoHorizon, new double[]{moonAzStart, moonElStart}, AngleType.AZEL);
        int[] moonRowColStart = angularDirectionToGridPosition(moonAzElStart);

        // Get row and col position of Moon at the end of observation campaign
        double moonAzEnd = moon.getPVCoordinates(endObs, topoHorizon).getPosition().getAlpha();
        double moonElEnd = moon.getPVCoordinates(endObs, topoHorizon).getPosition().getDelta();
        AngularDirection moonAzElEnd = new AngularDirection(topoHorizon, new double[]{moonAzEnd, moonElEnd}, AngleType.AZEL);
        int[] moonRowColEnd = angularDirectionToGridPosition(moonAzElEnd);

        // Construct line connecting start with end
        int rowStart = moonRowColStart[0];
        int colStart = moonRowColStart[1];
        int rowEnd = moonRowColEnd[0];
        int colEnd = moonRowColEnd[1];

        double slope = (rowEnd - rowStart)/(colEnd - colStart);
        double intercept = (colEnd*rowStart - colStart*rowEnd)/(colEnd - colStart);
        PolynomialFunction line = new PolynomialFunction(new double[]{intercept, slope});

        // Zero out region between start and end
        for(int c=moonRowColStart[1]; c<=moonRowColEnd[1]; c++){

            // compute row coordinate along the line connecting start and end
            int r = (int) FastMath.round(line.value(c));
            densityModel[r][c] = -1;        // zero out patch

            // Zero out 10 deg above and below line
            for(int offset=0; offset<halfMoonDist; offset++){
                densityModel[r+offset][c] = -1;
                densityModel[r-offset][c] = -1;
            }
        }

        // Zero out boundary region outside start and end --> start and end mark mid position of moon
        for(int colOffset=0; colOffset<halfMoonDist; colOffset++) {

            int cStart = moonRowColStart[1] - colOffset;
            int cEnd = moonRowColEnd[1] + colOffset;
            int rStart = (int) line.value(cStart);
            int rEnd = (int) line.value(cEnd);

            // Zero out 10 deg above and below line
            for(int rowOffset=0; rowOffset<halfMoonDist; rowOffset++){
                densityModel[rStart+rowOffset][cStart] = -1;
                densityModel[rStart-rowOffset][cStart] = -1;

                densityModel[rEnd+rowOffset][cEnd] = -1;
                densityModel[rEnd-rowOffset][cEnd] = -1;
            }
        }
    }


    /**
     * Retrieve position of the field whose angular position is the closest to {@code angles}.
     * 
     * @param angles            Angular position of the space object.
     * @return
     */
    protected int[] angularDirectionToGridPosition(AngularDirection angles) {
        double azimuth = angles.getAngle1();
        double elevation =  angles.getAngle2();

        double heightFov = sensor.getFov().getHeight();
        double widthFov = sensor.getFov().getWidth();

        int row = (int) FastMath.round((elevation-sensor.getElevCutOff())/heightFov);
        int col = (int) FastMath.round(azimuth/widthFov);

        // Throw error if frame of angles is not the same as of the spatial density model
        if (!angles.getFrame().getName().equals(topoHorizon.getName())) {
            throw new IllegalArgumentException("Angular position is not defined in topocentric horizon frame.");
        }
        return new int[]{row, col};
    }
    
}
