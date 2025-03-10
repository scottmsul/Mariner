package frc.robot;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.DoubleConsumer;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class NTDouble {
    private double val;
    private ArrayList<DoubleConsumer> listeners = new ArrayList<>();

    public class NTD {
        public static NTDouble of(double val) {
            return new NTDouble(val);
        }
    }

    NTDouble(double val) {
        this.val = val;
    }

    NTDouble(double val, String name, String klass) {
        this.val = val;

        var key = "/AutoTune/" + klass + "/" + name;
        var entry = NetworkTableInstance.getDefault().getEntry(key);
        entry.setDouble(val);
        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            e -> {
                this.val = e.valueData.value.getDouble();
                System.out.println("Updated " +klass + "."+name + " to " + this.val);
                for (var l : listeners) {
                    l.accept(this.val);
                }
            });
    }

    public double get() {
        return val;
    }

    public void subscribe(DoubleConsumer sink) {
        sink.accept(val);
        listeners.add(sink);
    }
}
