package data;

import java.util.Objects;

public class SpaceTrackLoader {

    /**
     * Build query of list of satellites either with their latest TLE or with all TLEs in a 
     * predefined time range. Query shall then be forwarded to space-track.
     * 
     * @param noradId                       Array of Norad IDs.
     * @param sortByEpoch                   True, if history shall be sorted by epoch. 
     *                                      False, if it shall be sorted by Norad ID.
     * @param startDate                     Start date in the format YYYY-MM-DD, if null then 
     *                                      retrieve only latest TLE.
     * @param endDate                       End date in the format YYYY-MM-DD, if null then 
     *                                      retrieve only latest TLE.
     * @return                              Generated query.
     */
    public static String buildTleQuery(final int[] noradId, final boolean sortByEpoch,
                                   final String startDate, final String endDate) {
            
        // Build Norad ID query (remove last comma)
        String noradIdQuery = "";
        for (int i = 0; i<noradId.length; i++) {
            noradIdQuery = noradIdQuery + String.valueOf(noradId[i]) + ",";
        }
        noradIdQuery = noradIdQuery.substring(0, noradIdQuery.length() - 1);

        if(Objects.isNull(startDate) || Objects.isNull(endDate)) {

            // Only download the latest TLE from the list of objects
            String query = 
            "/basicspacedata/query/class/gp_history/NORAD_CAT_ID/"
            + noradIdQuery
            + "/orderby/TLE_LINE1 ASC/format/tle";
            return query;
        }

        // Build Epoch query
        String epochQuery = startDate + "--" + endDate;

        // Build sorting query
        String sortingQuery = "";
        if (sortByEpoch) {
            sortingQuery = "EPOCH%20ASC";
        } else {
            sortingQuery = "TLE_LINE1%20ASC";
        }

        String query = 
            "/basicspacedata/query/class/gp_history/NORAD_CAT_ID/"
            + noradIdQuery
            + "/orderby/"
            + sortingQuery
            + "/EPOCH/"
            + epochQuery
            + "/"
            + "format/tle";
        
        return query;
    }
    
    public static String buildOmmQuery(final int[] noradId, final boolean sortByEpoch, 
                                     final String limitMeanMotion, final String limitEcc, 
                                     final String limitInc, final String limitRaan, 
                                     final String limitArgPerigee, final String limitMeanAnomaly,
                                     final String limitEpoch){

        String query = "/basicspacedata/query/class/gp";

        // Build Norad ID query (remove last comma)
        if (noradId.length != 0) {
            String noradIdQuery = "";
            for (int i = 0; i<noradId.length; i++) {
                noradIdQuery = noradIdQuery + String.valueOf(noradId[i]) + ",";
            }
            noradIdQuery = noradIdQuery.substring(0, noradIdQuery.length() - 1);
            query += "/NORAD_CAT_ID/" + noradIdQuery;
        }

        // Build sorting query
        String sortingQuery = "";
        if (sortByEpoch) {
            sortingQuery = "EPOCH%20ASC";
        } else {
            sortingQuery = "TLE_LINE1%20ASC";
        }
        query += "/orderby/" + sortingQuery;

        // Build time constrain query
        if (!Objects.isNull(limitEpoch) && !limitEpoch.isEmpty()) {
            query += "/EPOCH/" + limitEpoch;
        }

        // Constrain mean motion
        if (!Objects.isNull(limitMeanMotion) && !limitMeanMotion.isEmpty()) {
            query += "/MEAN_MOTION/" + limitMeanMotion;
        }

        // Constrain eccentricity
        if (!Objects.isNull(limitEcc) && !limitEcc.isEmpty()) {
            query += "/ECCENTRICITY/" + limitEcc;
        }

        // Constrain inclination
        if (!Objects.isNull(limitInc) && !limitInc.isEmpty()) {
            query += "/INCLINATION/" + limitInc;
        }

        // Constrain Raan
        if (!Objects.isNull(limitRaan) && !limitRaan.isEmpty()) {
            query += "/RA_OF_ASC_NODE/" + limitRaan;
        }

        // Constrain argument of perigee
        if (!Objects.isNull(limitArgPerigee) && !limitArgPerigee.isEmpty()) {
            query += "/ARG_OF_PERICENTER/" + limitArgPerigee;
        }

        // Constrain mean anomaly
        if (!Objects.isNull(limitMeanAnomaly) && !limitMeanAnomaly.isEmpty()) {
            query += "/MEAN_ANOMALY/" + limitMeanAnomaly;
        }

        // Build format query
        query +="/format/xml";

        return query;
    }
}
