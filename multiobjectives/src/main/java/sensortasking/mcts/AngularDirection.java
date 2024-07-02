package sensortasking.mcts;

import org.hipparchus.geometry.euclidean.threed.Vector3D;
import org.hipparchus.util.FastMath;
import org.orekit.bodies.CelestialBody;
import org.orekit.bodies.CelestialBodyFactory;
import org.orekit.frames.Frame;
import org.orekit.frames.Transform;
import org.orekit.time.AbsoluteDate;

import lombok.Getter;

@Getter
public class AngularDirection {

    /** Date. */
    private AbsoluteDate date;          //TODO: Question the date as attribute

    /** Reference frame. */
    private Frame frame;

    /** Values in [rad] of the pair of angles. */
    private double[] angles;

    /** Angle type. */
    private AngleType angleType;

    /** Description of pointing direction. */
    private String name;

    /**
     * Simple constructor.
     * 
     * @param frame                 Reference frame.
     * @param angles                Angular direction in [rad]. First angle relates to azimuthal 
     *                              angle (on x-y plane, rotated around z-axis). The second angle
     *                              represents the inclination wrt the x-y plane.
     * @param type                  Angle type, either RADEC, AZEL or LONLAT.
     */
    public AngularDirection(Frame frame, double[] angles, AngleType type) {
        this.frame = frame;
        this.angles = new double[]{angles[0], angles[1]};
        this.angleType = type;
    }

    /** 
     * Transform the pointing direction into {@code dest} frame.
     * 
     * @param dest                  Destination reference frame.
     * @param date                  Epoch related to frame transformation.
     * @param destAngleType         Transformed angle type.
     * 
     * @return                      Angular direction with respect to the destination frame.
     */
    public AngularDirection transformReference(Frame dest, AbsoluteDate date, 
                                               AngleType destAngleType) {
        Transform t = this.frame.getTransformTo(dest, date);
        Vector3D transformedDir = t.transformVector(new Vector3D(angles[0], angles[1]));
        double angle1 = transformedDir.getAlpha();
        if(angle1 < 0){
            angle1 += 2*FastMath.PI;
        }
        double[] transformedAngles = new double[] {angle1, transformedDir.getDelta()};
        return new AngularDirection(dest, transformedAngles, destAngleType);
    }

    /**
     *  Get first angle in [rad] within (0; 2*PI)-interval.
     * 
     * @return                      Angle in [rad].
     */
    public double getAngle1() {
        if(angles[0] < 0) {
            angles[0] = angles[0] + 2*FastMath.PI;
        }
        return angles[0];
    }

    /**
     *  Get second angle in [rad] within (-PI/2; PI/2)-interval.
     * 
     * @return                      Angle in [rad].
     */
    public double getAngle2() {
        return angles[1];
    }

    /** Set description of angular direction.
     * 
     * @param description           Description of angular pointing direction.
     */
    public void setName(String description) {
        this.name = description;
    }

    /**
     * Set date corresponding to pointing direction.
     * 
     * @param date                  Date.
     */
    public void setDate(AbsoluteDate date) {
        this.date = date;
    }

    /**
     * Get angle between this instance and {@code other}.
     * 
     * @param other                 Other vector.
     * 
     * @return                      Angle enclosed by this instance and {@code other}.
     */
    public double getEnclosedAngle(AngularDirection other) {

        if (!this.getFrame().equals(other.getFrame())) {

            // TODO: Frame transformation
            throw new IllegalArgumentException("The two pointing directions are expressed in " + 
                                               "two different frames.");
        }
        double thisAngle1 = this.getAngle1();
        double otherAngle1 = other.getAngle1();
        if(thisAngle1 > FastMath.PI) {
            // Need to transform interval to [-pi, pi] to create a Vector3D
            thisAngle1 = thisAngle1 - 2*FastMath.PI;
        }
        if(otherAngle1 > FastMath.PI) {
            // Need to transform interval to [-pi, pi] to create a Vector3D
            otherAngle1 = otherAngle1 - 2*FastMath.PI;
        }
        Vector3D thisDir = new Vector3D(thisAngle1, this.getAngle2());
        Vector3D otherDir = new Vector3D(otherAngle1, other.getAngle2());
        double angle = Vector3D.angle(thisDir, otherDir);
        return angle;
    }

    /**
     * Compute angular distance to Moon.
     * 
     * @param date                  Reference date.
     * @param frame                 Reference frame.
     * @param pos                   Angular position of object of interest.
     * 
     * @return                      Angular distance in [rad] to Moon.
     */
    public static double computeAngularDistMoon(AbsoluteDate date, Frame frame, 
                                                   AngularDirection pos) {
        CelestialBody moon = CelestialBodyFactory.getMoon();
        double moonRa = moon.getPVCoordinates(date, frame)
                            .getPosition()
                            .getAlpha();
        double moonDec = moon.getPVCoordinates(date, frame)
                            .getPosition()
                            .getDelta();
        AngularDirection moonAngles = 
            new AngularDirection(frame, new double[]{moonRa, moonDec}, AngleType.RADEC);
        return pos.getEnclosedAngle(moonAngles);
    }

    /**
     * Substract angular direction other from instance, i.e. this - other.
     * TODO: add this function to other projects.
     * 
     * @param other         Angular direction to be substracted from this.
     * @return              Angular direction resulting from this minus other.
     */
    public AngularDirection substract(AngularDirection other) {
        if(!this.getAngleType().equals(other.getAngleType())) {
            throw new IllegalArgumentException("Different angle types");
        }
        if(!this.getFrame().equals(other.getFrame())) {
            throw new IllegalArgumentException("Different refernece frames");
        }
        double angle1 = this.getAngle1() - other.getAngle1();
        double angle2 = this.getAngle1() - other.getAngle2();

        return new AngularDirection(this.frame, new double[]{angle1, angle2}, this.getAngleType());
    }
}
