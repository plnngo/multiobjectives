package sensortasking.mcts;

import java.util.ArrayList;
import java.util.List;

import org.hipparchus.ode.events.Action;
import org.hipparchus.util.FastMath;
import org.orekit.files.ccsds.ndm.cdm.StateVector;
import org.orekit.files.ccsds.ndm.odm.CartesianCovariance;
import org.orekit.frames.FramesFactory;
import org.orekit.frames.TopocentricFrame;
import org.orekit.propagation.events.ElevationDetector;
import org.orekit.propagation.events.EventDetector;
import org.orekit.propagation.events.EventsLogger.LoggedEvent;
import org.orekit.time.AbsoluteDate;
import org.orekit.time.TimeScale;
import org.orekit.time.TimeScalesFactory;

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
    List<LoggedEvent> loggedFORpasses = new ArrayList<LoggedEvent>();

    /** Maximal propagation duration in [sec] to check if satellite is entering field of regard. */
    final double maxPropDuration = 60.*15;


    public TrackingObjective(List<ObservedObject> targets) {

        // Initialise list of targets
        for (ObservedObject target : targets) {
            initTargets.add(target);
            updatedTargets.add(target);
        }
    }
    @Override
    public AngularDirection setMicroAction(AbsoluteDate current, TopocentricFrame stationFrame) {

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


        // Iterate through list of objects of interest
        for (ObservedObject candidate : updatedTargets) {


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

    @Override
    public double getUtility() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getUtility'");
    }

    @Override
    public AbsoluteDate[] getExecusionDuration(AbsoluteDate current) {
        //return 60.*5.;
        AbsoluteDate[] interval = new AbsoluteDate[]{current, current};
        if (this.loggedFORpasses.size() == 2) {
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
