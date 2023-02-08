package frc.robot;

public class AutoManager {
    private static AutoManager instance = null;

    private AutoManager() {

    }

    public static AutoManager getInstance() {
        if (instance == null) {
            instance = new AutoManager();
        }

        return instance;
    }
}
