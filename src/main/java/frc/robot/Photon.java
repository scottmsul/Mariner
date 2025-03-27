package frc.robot;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class Photon {
    private static Photon instance;

    public static Photon getInstance() {
        if (instance == null) {
            instance = new Photon();
        }

        return instance;
    }

    Photon() {
    }

    public PhotonCamera upper = new PhotonCamera("Upper");

    private List<PhotonPipelineResult> results = Collections.emptyList();

    // Must be called ONCE per robot iteration
    public void update() {
        results = upper.getAllUnreadResults();
    }

    private static PhotonPipelineResult kEmptyPiplineResult = new PhotonPipelineResult();

    public PhotonPipelineResult getLastResult() {
        if (results.size() == 0) {
            return kEmptyPiplineResult;
        }
        return results.get(results.size() - 1);
    }

    public List<PhotonPipelineResult> getAllResults() {
        return results;
    }
}
