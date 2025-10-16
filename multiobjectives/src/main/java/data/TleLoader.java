package data;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
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
    public static List<TLE> loadData(File file) throws IOException{
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

    public static List<TLE> parse(File tleFile) throws IOException {

        List<TLE> tles = new ArrayList<>();
        try (BufferedReader reader = new BufferedReader(new FileReader(tleFile))) {
            String line;
            String line1 = null;
            String line2 = null;
            while ((line = reader.readLine()) != null) {
                if (line.startsWith("1 ")) {
                    line1 = line.trim();
                } else if (line.startsWith("2 ")) {
                    line2 = line.trim();
                    if (line1 != null) {
                        TLE tle = new TLE(line1, line2);
                        tles.add(tle);
                        // Reset for next TLE
                        line1 = null;
                        line2 = null;
                    }
                }
                // ignore lines that start with "0" (name) or are blank
            }
        }

        return tles;
    }


    public static void main(String[] args) throws IOException {

        // Load orekit data
        String workingDir = System.getProperty("user.dir");
        String orekitDataDir = "\\multiobjectives\\src\\test\\java\\resources\\orekit-data";
        File orekitData = new File(workingDir + orekitDataDir);
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));

        File tleFile = new File( System.getProperty("user.dir") + "\\multiobjectives\\src\\main\\java\\data\\Catalogue_16_10_2025.txt");
        List<TLE> tles = TleLoader.parse(tleFile);

        for(TLE tle: tles) {
            System.out.println(tle.getLine1());
            System.out.println(tle.getLine2());
        }
        
    }
    
}
