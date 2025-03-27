package frc.souffle;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.util.struct.StructSerializable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;

class SuppliedValue<T> {
    private Supplier<T> supplier;
    private Consumer<T> setter;

    public SuppliedValue(Supplier<T> supplier, Consumer<T> setter) {
        this.supplier = supplier;
        this.setter = setter;
    }

    public void update() {
        setter.accept(supplier.get());
    }
}

public class Souffle {
    private static Map<String, StructPublisher<?>> publishers = new ConcurrentHashMap<>();

    private static Map<String, SuppliedValue<?>> valueSuppliers = new ConcurrentHashMap<>();

    public static <T extends StructSerializable> void log(String key, T value) {
        if (value == null) {
            return;
        }

        var publisher = publishers.get(key);
        if (publisher == null) {
            var maybeStruct = StructRegistry.getStruct(value.getClass());
            if (maybeStruct.isPresent()) {
                @SuppressWarnings("unchecked")
                var struct = (Struct<T>) maybeStruct.get();
                publisher = NetworkTableInstance.getDefault().getStructTopic(key, struct).publish();
                publishers.put(key, publisher);
            } else {
                return;
            }
        }

        @SuppressWarnings("unchecked")
        StructPublisher<T> structPub = (StructPublisher<T>) publisher;
        structPub.set(value);
    }

    public static void log(String key, double value) {
        NetworkTableInstance.getDefault().getEntry(key).setDouble(value);
    }

    public static void log(String key, boolean value) {
        NetworkTableInstance.getDefault().getEntry(key).setBoolean(value);
    }

    public static void record(String key, DoubleSupplier supplier) {
        valueSuppliers.put(key,
                new SuppliedValue<>(
                        supplier::getAsDouble,
                        NetworkTableInstance.getDefault().getEntry(key)::setDouble));
    }

    public static void record(String key, BooleanSupplier supplier) {
        valueSuppliers.put(key,
                new SuppliedValue<>(
                        supplier::getAsBoolean,
                        NetworkTableInstance.getDefault().getEntry(key)::setBoolean));
    }

    public static <T extends StructSerializable> void record(String key, Supplier<T> supplier) {
        var value = supplier.get();

        var maybeStruct = StructRegistry.getStruct(value.getClass());
        if (maybeStruct.isPresent()) {
            @SuppressWarnings("unchecked")
            var struct = (Struct<T>) maybeStruct.get();

            var publisher = NetworkTableInstance.getDefault().getStructTopic(key,
                    struct).publish();
            publisher.set(value);
            valueSuppliers.put(key,
                    new SuppliedValue<T>(
                            supplier::get, publisher::set));
        } else {
            return;
        }
    }

    // Very janky, needs review
    public static <T> void record(String key, Sendable sendable) {
        String name = SendableRegistry.getName(sendable);
        var builder = new SendableBuilderImpl();
        builder.setTable(NetworkTableInstance.getDefault().getTable(key).getSubTable(name));
        sendable.initSendable(builder);
        builder.startListeners();
        valueSuppliers.put(key, new SuppliedValue<Void>(() -> null, (v) -> builder.update()));
        builder.update();
    }

    /**
     * Must be called in robotPeriodic. Should not be called anywhere else.
     */
    public static void update() {
        for (var sup : valueSuppliers.values()) {
            sup.update();
        }
    }
}
