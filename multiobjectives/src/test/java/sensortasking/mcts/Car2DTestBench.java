package sensortasking.mcts;

import java.io.File;

import org.hipparchus.linear.DiagonalMatrix;
import org.junit.Before;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;

import benchtest.Filter;

public class Car2DTestBench {
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
    public void testBench(){
        Filter estimateCarA = new Filter();
        double[] init = new double[]{0, -1, 5, 0};
        DiagonalMatrix P0 = new DiagonalMatrix(new double[]{0.1, 0.1, 0.01, 0.01});
        estimateCarA.run_ckf(init, P0.getData(), 5., 25.02);  
    }
}
