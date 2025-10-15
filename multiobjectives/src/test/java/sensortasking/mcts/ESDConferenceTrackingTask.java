package sensortasking.mcts;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.hipparchus.linear.DiagonalMatrix;
import org.hipparchus.linear.RealMatrix;
import org.hipparchus.util.FastMath;
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
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.orbits.OrbitType;
import org.orekit.orbits.PositionAngleType;
import org.orekit.propagation.SpacecraftState;
import org.orekit.propagation.StateCovariance;
import org.orekit.propagation.analytical.tle.TLE;
import org.orekit.propagation.analytical.tle.TLEPropagator;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScalesFactory;
import org.orekit.utils.Constants;
import org.orekit.utils.IERSConventions;

import com.opencsv.CSVWriter;

import benchtest.Satellite;

import org.orekit.frames.Frame;

public class ESDConferenceTrackingTask {
    @Before
    public void init() {
        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }
    
    @Test
    public void testPaperTracking() throws IOException {

        // Date
        AbsoluteDate date = new AbsoluteDate(2025, 3, 24, 22, 1, 2.62, TimeScalesFactory.getUTC());
        AbsoluteDate end = date.shiftedBy(475.);

        // Frame
        Frame ecef = FramesFactory.getITRF(IERSConventions.IERS_2010, true);

        // Ground station
        GeodeticPoint pos = new GeodeticPoint(FastMath.toRadians(6.),   // Geodetic latitude
                                              FastMath.toRadians(-37.),   // Longitude
                                              0.);              // in [m]
        // Model Earth
        BodyShape earth = new OneAxisEllipsoid(Constants.WGS84_EARTH_EQUATORIAL_RADIUS,
                                               Constants.WGS84_EARTH_FLATTENING,
                                               ecef);
        TopocentricFrame topohorizon = new TopocentricFrame(earth, pos, "TDRS Station");

        // create FileWriter object with file as parameter 
        List<List<Node>> solutions = new ArrayList<List<Node>>();
        String out = "";
        FileWriter outputfile = new FileWriter("OnlyTracking2.csv"); 

        // create CSVWriter object filewriter object as parameter 
        CSVWriter writer = new CSVWriter(outputfile); 
        List<double[]> utilityStrategiesRatio = new ArrayList<double[]>();

        // MCTS monte carlo runs
        int mctsCalls = 1;
        int mctsIter = 2000;

        for (int i=0; i<mctsCalls; i++) {

            // Settings for searching objective
            List<Integer> stripeBullseyeCompleted = new ArrayList<Integer>();
            stripeBullseyeCompleted.add(0); // Stripe scan
            stripeBullseyeCompleted.add(0); // Bullseye scan

            List<ObservedObject> ooi = generateListOfCandidates(date);

            final PropoagatedEnvironment enviro = new PropoagatedEnvironment(ooi, stripeBullseyeCompleted);
            MultiObjectiveMcts mcts = MultiObjectiveMctsTest.setUpMcts(date, end, topohorizon, enviro);
            List<Node> strategy = mcts.run(mctsIter);
            System.out.println(strategy.size());

            String[] selected = new String[(strategy.size()-1)/2 + 1 + ooi.size()];
            int j =0;
            for(Node currentNode : strategy) {
                if (currentNode.getClass().getSimpleName().equals("ChanceNode")) {
                    String objective = ((ChanceNode) currentNode).getMacro().getClass().getSimpleName();
                    if (objective.equals("TrackingObjective")) {
                        long id = ((TrackingObjective)((ChanceNode) currentNode).getMacro())
                                                                                    .getLastUpdated();

                        if (id == 21639) {
                            out = out + "A ";

                        } else if(id == 22314) {
                            out = out + "B ";
                        } else {
                            out = out + "C ";
                        }
                        selected[j] = Long.toString(id);
                        System.out.print(selected[j] + " ");
                        j++;
                        
                    } else {
                        out = out + "S ";
                        selected[j] = "S";
                        System.out.print(selected[j] + " ");
                        j++;
                    }
                } else {
                    //j=0;
                    continue;
                }
            }
            // Compute IG of final strategy
            double[] iG = new double[]{}; //mcts.computeTrackReward((DecisionNode)strategy.get(strategy.size()-1));
            double searchT = ((DecisionNode)strategy.get(strategy.size()-1)).getTimeSpentStripe();
            for(int k=0; k<iG.length; k++) {
                selected[j + k] = Double.toString(iG[k]);
                System.out.print(selected[j + k] + " - ");
            }
            selected[j + iG.length] = Double.toString(searchT);

            // Calculate ratio 
            double[] ratioUtility = new double[iG.length + 1];
            for(int k=0; k<iG.length; k++) {
                //ratioUtility[k] = FastMath.abs((iG[k]/totalIG) - weight);
                ratioUtility[k] = iG[k];
            }
            ratioUtility[iG.length] = searchT;
            utilityStrategiesRatio.add(ratioUtility);
            System.out.println();

            writer.writeNext(selected);
            j=0;
            out = out + "\n";
            solutions.add(strategy);
        }
        System.out.println(out);
        // closing writer connection 
        writer.close(); 

    }

    public static List<ObservedObject> generateListOfCandidates(AbsoluteDate current) {

        // Reference frame in ECI
        Frame j2000 = FramesFactory.getEME2000();

         // Create list of objects of interest
        TLE tleTdrs05 = new TLE("1 21639U 91054B   25081.51181291 -.00000056  00000-0  00000+0 0  9995", 
                                "2 21639  14.1260 357.0831 0003813   3.2787 258.6419  0.99948788123177");      
        TLE tleTdrs06 = new TLE("1 22314U 93003B   25081.20863616 -.00000286  00000-0  00000+0 0  9991",
                                "2 22314  14.1718   0.3832 0006404 172.6992  36.1023  1.00270003117841");
        TLE tleTdrs12 = new TLE("1 39504U 14004A   25080.81834953 -.00000263  00000-0  00000-0 0  9998", 
                                "2 39504   3.6316  10.0282 0002090 260.4678 162.8071  1.00272110 39741");

        // Compute state
        TLEPropagator propTdrs05 = TLEPropagator.selectExtrapolator(tleTdrs05);
        TLEPropagator propTdrs06 = TLEPropagator.selectExtrapolator(tleTdrs06);
        TLEPropagator propTdrs12 = TLEPropagator.selectExtrapolator(tleTdrs12);

        SpacecraftState spacecraftTdrs05 = propTdrs05.propagate(current);
        SpacecraftState spacecraftTdrs06 = propTdrs06.propagate(current);
        SpacecraftState spacecraftTdrs12 = propTdrs12.propagate(current);

        StateVector stateTdrs05 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs05, j2000);
        StateVector stateTdrs06 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs06, j2000);
        StateVector stateTdrs12 = ObservedObject.spacecraftStateToStateVector(spacecraftTdrs12, j2000);

        
        System.out.println("posX " + stateTdrs12.getPositionVector().getX() + " velX " + stateTdrs12.getVelocityVector().getX());
        System.out.println("posY " + stateTdrs12.getPositionVector().getY() + " velY " + stateTdrs12.getVelocityVector().getY());
        System.out.println("posZ " + stateTdrs12.getPositionVector().getZ() + " velZ " + stateTdrs12.getVelocityVector().getZ());


        RealMatrix covMatrixTdrs05 = new DiagonalMatrix(new double[]{1e6, 1e6, 1e6, 1., 1., 1.});
        RealMatrix covMatrixTdrs06 = new DiagonalMatrix(new double[]{1e6, 1e6, 1e6, 1., 1., 1.});
        RealMatrix covMatrixTdrs12 = new DiagonalMatrix(new double[]{1e6, 1e6, 1e6, 1., 1., 1.});

        StateCovariance covEciTdrs05 = new StateCovariance(covMatrixTdrs05, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        StateCovariance covEciTdrs06 = new StateCovariance(covMatrixTdrs06, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);
        StateCovariance covEciTdrs12 = new StateCovariance(covMatrixTdrs12, current, j2000, OrbitType.CARTESIAN, PositionAngleType.MEAN);

        CartesianCovariance stateCovTdrs05 =
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs05.getOrbit(), covEciTdrs05, j2000); 
        CartesianCovariance stateCovTdrs06 =
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs06.getOrbit(), covEciTdrs06, j2000);
        CartesianCovariance stateCovTdrs12 = 
            ObservedObject.stateCovToCartesianCov(spacecraftTdrs12.getOrbit(), covEciTdrs12, j2000);

        // Create list of objects of interest
        ObservedObject tdrs05 = new Satellite(tleTdrs05.getSatelliteNumber(), stateTdrs05, stateCovTdrs05, current, j2000);
        ObservedObject tdrs06 = new Satellite(tleTdrs06.getSatelliteNumber(), stateTdrs06, stateCovTdrs06, current, j2000);
        ObservedObject tdrs12 = new Satellite(tleTdrs12.getSatelliteNumber(), stateTdrs12, stateCovTdrs12, current, j2000);

        List<ObservedObject> ooi = new ArrayList<ObservedObject>();
        ooi.add(tdrs05);
        ooi.add(tdrs06);
        ooi.add(tdrs12);
        return ooi;
    }
}
