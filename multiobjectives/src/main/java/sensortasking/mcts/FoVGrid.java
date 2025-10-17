package sensortasking.mcts;

import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

import org.hipparchus.util.FastMath;

public class FoVGrid {
    public final List<Fov> cells = new ArrayList<>();

    /**
     * Discretaise field of regard where each FoV now covers an approximately equal 
     * area (solid angle) on the dome.
     * 
     * @param deltaEl           vertical step size [rad]
     * @param azRef             reference azimuth step at horizon [rad]
     */
    public FoVGrid(double deltaEl, double azRef) {
        for (double elMin = 0; elMin < FastMath.PI/2; elMin += deltaEl) {
            double elCenter = elMin + deltaEl/2.0;
            // approximate # of azimuth cells proportional to cos(el)
            int nAz = FastMath.max(1, (int) FastMath.round(2*FastMath.PI / (azRef / FastMath.cos(elCenter))));
            double deltaAz = 2*FastMath.PI / nAz;

            for (int i = 0; i < nAz; i++) {
                double azMin = i * deltaAz;
                double azMax = azMin + deltaAz;
                cells.add(new Fov(azMin, azMax, elMin, elMin + deltaEl));
            }
        }
    }

    /**
     * Generate discretised region of interest mapped on field of regard.
     * 
     * @param region            Discretised region of interest.
     */
    public FoVGrid(List<Fov> region) {
        for(Fov patch : region) {
            this.cells.add(patch);
        }
    }

    public static void main(String[] args) {
        double deltaEl = FastMath.toRadians(5.); // elevation step
        double azRef = FastMath.toRadians(5.);   // reference azimuth step at horizon
        FoVGrid grid = new FoVGrid(deltaEl, azRef);

        String filename = "fov_grid.csv";
        try (FileWriter writer = new FileWriter(filename)) {
            // Header (angles in radians)
            writer.write(
                "azMin,azMax,elMin,elMax,azCenter,elCenter," +
                "x,y,z," +
                "x1,y1,z1,x2,y2,z2,x3,y3,z3,x4,y4,z4\n"
            );

            // Data rows
            for (Fov cell : grid.cells) {
                writer.write(String.format(Locale.US,
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f," +  // angles
                    "%.6f,%.6f,%.6f," +                 // center
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f%n", // 4 corners
                    cell.azMin, cell.azMax, cell.elMin, cell.elMax,
                    cell.azCenter, cell.elCenter,
                    cell.centerVec[0], cell.centerVec[1], cell.centerVec[2],
                    cell.corners[0][0], cell.corners[0][1], cell.corners[0][2],
                    cell.corners[1][0], cell.corners[1][1], cell.corners[1][2],
                    cell.corners[2][0], cell.corners[2][1], cell.corners[2][2],
                    cell.corners[3][0], cell.corners[3][1], cell.corners[3][2]
                ));
            }
            System.out.println("FoV grid written to " + filename);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Find the FoV cell containing the given azimuth and elevation.
     *
     * @param az                Azimuth [rad], in [0, 2π)
     * @param el                Elevation [rad], in [0, π/2]
     * 
     * @return                  The Fov cell containing (az, el), or null if outside range.
     */
    public Fov getCell(double az, double el) {
        for (Fov cell : cells) {
            if (az >= cell.azMin && az < cell.azMax &&
                el >= cell.elMin && el < cell.elMax) {
                return cell;
            }
        }
        return null;
    }
}
