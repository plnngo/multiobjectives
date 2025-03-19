package tools;

import org.orekit.estimation.sequential.KalmanEstimation;
import org.orekit.estimation.sequential.KalmanObserver;

public class ModelLogger implements KalmanObserver {
    KalmanEstimation estimation;

    @Override
    public void evaluationPerformed(KalmanEstimation estimation) {
        this.estimation = estimation;
    }
}