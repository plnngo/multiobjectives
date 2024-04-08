package data;

import java.io.File;
import java.io.IOException;
import java.util.List;

import org.junit.Before;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;
import org.orekit.propagation.analytical.tle.TLE;

public class TleLoaderTest {

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
    public void testLoadData() throws IOException{
        TleLoader loader = new TleLoader();
        File tleFile = new File( System.getProperty("user.dir") + "\\src\\test\\java\\output\\TestFile.tle");
        List<TLE> tles = loader.loadData(tleFile);

        for(TLE tle: tles) {
            System.out.println(tle.getLine1());
            System.out.println(tle.getLine2());
        }
    }
}
