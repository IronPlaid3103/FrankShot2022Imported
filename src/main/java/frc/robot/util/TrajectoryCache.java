package frc.robot.util;

import java.io.IOException;
import java.nio.file.Path;
import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class TrajectoryCache {
    private static HashMap<String, Trajectory> cache = new HashMap<String, Trajectory>();

    public static void add(String key, String jsonPath) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(jsonPath);
            cache.put(key, TrajectoryUtil.fromPathweaverJson(trajectoryPath));
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + jsonPath, ex.getStackTrace());
        }
    }

    public static Trajectory get(String key) {
        return cache.get(key);
    }

    public static void clear() {
        cache.clear();
    }
}
