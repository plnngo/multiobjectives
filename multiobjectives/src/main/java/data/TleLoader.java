package data;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.orekit.propagation.analytical.tle.TLE;

public class TleLoader {

    /** Series of downloaded TLEs from space-track. */
    List<TLE> tleSeries;

    /**
     * Load file of TLE history into a list of TLEs orekit objects.
     * 
     * @param file                          File with TLE data.
     * @return                              List of TLEs orekit objects
     * @throws IOException                  If an I/O error occurs.
     */
    public List<TLE> loadData(File file) throws IOException{
        try (BufferedReader br = new BufferedReader(new FileReader(file))) {
            List<TLE> tleHistory = new ArrayList<TLE>();
            String line;
            String line1 = null;
            boolean isFirstLine = true;
            while ((line = br.readLine()) != null) {
                if (isFirstLine) {
                    // Store first Line of TLE
                    line1 = line;
                    isFirstLine = false;
                } else {
                    // Create TLE and add to list
                    tleHistory.add(new TLE(line1, line));
                    isFirstLine = true;
                }
            }
            return tleHistory;
        }
    }  
    
}
