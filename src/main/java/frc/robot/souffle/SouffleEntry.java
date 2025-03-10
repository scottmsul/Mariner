package frc.robot.souffle;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.util.sendable.Sendable;

public class SouffleEntry {
    private final String key;
    private SouffleInstance instance;

    public SouffleEntry(SouffleInstance instance, String key) {
        this.instance = instance;
        this.key = key;
    }

    public String getKey() {
        return this.key;
    }

    private static String joinKey(String keyl, String keyr) {
        if (keyl.charAt(keyl.length() - 1) == '/' || keyl.charAt(keyl.length() - 1) == '/') {
            return keyl + keyr;
        }
        return keyl + "/" + keyr;

    }

    public SouffleEntry e(String key) {
        return new SouffleEntry(instance, joinKey(this.key, key));
    }

    public void add(String name, Sendable sendable) {
        instance.add(name,sendable);
    }

    public void addDouble(String name, DoubleSupplier getter) {
        addDouble(name, getter, null);
    }

    public void addDouble(String name, DoubleSupplier getter, DoubleConsumer setter) {
        instance.add(name, getter, setter);
    }
}
