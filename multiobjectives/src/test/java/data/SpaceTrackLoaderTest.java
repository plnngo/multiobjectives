package data;

import java.io.File;

import org.junit.Before;
import org.junit.Test;
import org.orekit.data.DataContext;
import org.orekit.data.DataProvidersManager;
import org.orekit.data.DirectoryCrawler;


public class SpaceTrackLoaderTest {

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
    public void testBuildOmmQueryLimitNoradIdAndEpoch(){

        // Set input of the query
        int[] noradId = new int[]{25544, 38101};
        boolean sortByEpoch = true;
        String limitInc = "";
        String limitEpoch = ">now-30";

        String actualQuery = 
            SpaceTrackLoader.buildOmmQuery(noradId, sortByEpoch, null, null, 
                                           limitInc, null, null, null, limitEpoch);

        // Build expected query string
        String expectedQuery = "/basicspacedata/query/class/gp" +  "/NORAD_CAT_ID/" 
                                + Integer.toString(noradId[0]) + "," + Integer.toString(noradId[1])
                                + "/orderby/" + "EPOCH%20ASC"
                                + "/EPOCH/" + limitEpoch 
                                + "/format/xml";

        // Assert
        org.junit.Assert.assertEquals(expectedQuery, actualQuery);
    } 

    @Test
    public void testBuildOmmQueryLimitKeplerElements(){
        boolean sortByEpoch = true;
        String limitInc = "<5";
        String limitEcc = "<0.01";
        String limitMeanMotion = "0.99--1.01";
        String limitRaan = ">45";
        String limitMeanAnomaly = "80--100";

        String actualQuery = 
        SpaceTrackLoader.buildOmmQuery(null, sortByEpoch, limitMeanMotion, limitEcc, limitInc, 
                                       limitRaan, null, limitMeanAnomaly, null);

        String expectedQuery = "/basicspacedata/query/class/gp/orderby/EPOCH%20ASC"
                                + "/MEAN_MOTION/" + limitMeanMotion
                                + "/ECCENTRICITY/" + limitEcc
                                +"/INCLINATION/" + limitInc
                                + "/RA_OF_ASC_NODE/" + limitRaan
                                + "/MEAN_ANOMALY/" + limitMeanAnomaly 
                                + "/format/xml";

        // Assert
        org.junit.Assert.assertEquals(expectedQuery, actualQuery);
    }
}
