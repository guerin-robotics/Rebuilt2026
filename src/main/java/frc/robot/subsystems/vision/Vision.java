package frc.robot.subsystems.vision;

import static frc.robot.Constants.Vision.*;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// PhotonVision classes are not available in the classpath right now, so local stubs are provided below to allow compilation.
// When you add the PhotonVision dependency to your build, remove these stubs and import the real classes.
import com.ctre.phoenix6.Utils;

public class Vision extends SubsystemBase{
    // Temporary local stub instance; replace with the real PhotonCamera usage once the dependency is added.
    private final PhotonCamera camera1 = new PhotonCamera(cameraName1);
    private final PhotonCamera camera2 = new PhotonCamera(cameraName2);

    public Vision() {
        // constructor stub — add initialization as needed
    }
}


class PhotonCamera {
    private final String name;

    public PhotonCamera(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }
}

class PhotonPoseEstimator {
    public enum PoseStrategy {
    }

    public PhotonPoseEstimator(Object arg0, PoseStrategy strategy, Object arg2) {
    }
}
