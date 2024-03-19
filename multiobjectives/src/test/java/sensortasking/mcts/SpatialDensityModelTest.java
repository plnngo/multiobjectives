package sensortasking.mcts;

import java.io.File;

import org.junit.Before;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;

public class SpatialDensityModelTest {

    @Before
    public void init() {
        
        // Load orekit data
        File orekitData = new File("C:/Users/plnngo/Documents/Programs/orekit/orekit-data");
        DataProvidersManager manager = DataContext.getDefault().getDataProvidersManager();
        manager.addProvider(new DirectoryCrawler(orekitData));
    }
    
}
