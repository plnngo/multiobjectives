package sensortasking.stripescanning;

import org.hipparchus.util.FastMath;
import org.orekit.frames.Frame;

import lombok.Getter;
import sensortasking.mcts.AngleType;
import sensortasking.mcts.AngularDirection;
import sensortasking.mcts.Sensor;

@Getter
public class Stripe {

    /** Fixed right ascension in [rad]. */
    // TODO: write getter for right ascension to ensure that it is returned between [0, 2*Pi]
    //private double fixRa;

    /** Number of declination fields in the stripe. */
    private int numDecFields;

    /** Reference frame. */
    private Frame frame;

    /** Observing sensor. */
    private Sensor sensor;

    /** Field size in [rad, rad]. */
    //private Fov fov;

    /** Pointing location of the first field. */
    private AngularDirection firstPosField;

    /** Purpose of the stripe, either fixed or re-observation stripe. */
    private StripeType functionality;

    /** Integer number of stripe duration that defines the position of re-obs stripes. 
     *  For fixed stripe j=0. */
    private int j;

    /** Repositioning time within stripe from one field to the next one. */
    private double reposInStripeT;

    /**
     * Constructor. 
     * 
     * @param ra                    Fixed right ascension angle.
     * @param numDecFields          Number of declination fields in the stripe.
     * @param frame                 Reference frame.
     * @param sensor                Sensor.
     * @param fov                   Field of view.
     * @param firstPosField         Angular pointing direction of the first field.
     * @param functionality         Stripe tyoe, either fixed or re-observation stripe.
     * @param j                     = 0 for fixed stripes, otherwise an integer number of stripe 
     *                              duration that defines the position of re-obs stripes.
     */
    public Stripe(int numDecFields, Frame frame, Sensor sensor,  
                  AngularDirection firstPosField, StripeType functionality, int j) {

        this.numDecFields = numDecFields;
        this.frame = frame;
        this.sensor = sensor;
        this.firstPosField = firstPosField;

        if (StripeType.FIXED.equals(functionality)) {
            this.functionality = StripeType.FIXED;
            this.j = 0;
        } else {
            this.functionality = StripeType.RE_OBS;
            this.j = j;
        }
    }


    /**
     * Get pointing direction of the requested field.
     * 
     * @param fieldNum              Requested field number.
     * 
     * @return                      Pointing direction of the requested field in [rad, rad]
     */
    public AngularDirection getPosField(int fieldNum) {

        if(fieldNum >= numDecFields) {
            throw new IllegalArgumentException("Field does not exist in the given stripe.");
        } else {
            double dec = firstPosField.getAngle2() + fieldNum*sensor.getFov().getHeight();
            return new AngularDirection(frame, new double[]{firstPosField.getAngle1(), dec}, 
                                        AngleType.RADEC);
        }
    }

    /**
     * Get duration to scan the entire stripe from the first to the last declination field.
     * Implementation according to Eq. (1) in Frueh, Carolin et al., "Heuristic and Optimized 
     * Sensor Tasking Observation Strategies with Exemplification for Geosynchronous Objects".
     * 
     * @param numExpos              Number of exposures of one field.
     * 
     * @return                      Stripe duration in [s].
     */
    public double getStripeT(int numExpos) {

        double exposureT = sensor.getExposureT();
        double readoutT = sensor.getReadoutT();

        // Compute repositioning time within stripe from one field to the next one
        reposInStripeT =  0.;        // if only one field in stripe
        if (this.getNumDecFields() > 1){
            reposInStripeT = sensor.computeRepositionT(this.getPosField(0), this.getPosField(1), 
                                                       sensor.isSlewVelInclSensorSettle());
        }

        double t1 = 0.;
        double t2 = readoutT;    // In contrast to Frueh, stripeT last from initial to final field
        if(reposInStripeT>readoutT) {
            t1 = reposInStripeT;
        } else {
            t1 = readoutT;
        }

        double stripeT = numDecFields * (numExpos * exposureT + (numExpos - 1) * readoutT) 
                            + (numDecFields - 1) * t1 + t2;
        return stripeT;
    }

    /**
     * Configure fixed observation stripe.
     * 
     * @param numDecFields      Number of declination fields.
     * @param frame             Reference frame.
     * @param observer          Sensor.
     * @param midStripePos      Angular position of mid stripe declination field.
     * @param j                 
     * 
     * @return                  Fixed observation stripe.
     */
    protected static Stripe setStripe(int numDecFields, Frame frame, Sensor observer, 
                                           AngularDirection midStripePos, int j) {
        
        double heightFov = observer.getFov().getHeight();

        // Deal with smaller earth shadow boundary
        double[] angles = new double[2];
        angles[0] = midStripePos.getAngle1();
        angles[1] = midStripePos.getAngle2() - (heightFov*FastMath.floor(numDecFields/2.));
        AngularDirection firstPos = 
            new AngularDirection(frame, angles, AngleType.RADEC);
                 
        if(j!=0) {
            return new Stripe(numDecFields, frame, observer, firstPos, StripeType.RE_OBS, j);
        } else {
            return new Stripe(numDecFields, frame, observer, firstPos, StripeType.FIXED, 0);
        }
    }
}
