package frc.robot.souffle;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.sendable.Sendable;

class SouffleInstance {

    private class Component<T> {
        private Supplier<T> supplier;
        private Consumer<T> setter;
        private SouffleEntry entry;
        // private  entry;

        Component(SouffleEntry entry, Supplier<T> supplier, Consumer<T> setter, BiConsumer<GenericPublisher, T> publisher) {
            this.supplier = supplier;
            this.setter = setter;
            this.entry = entry;
            // NetworkTableInstance.getDefault().getEntry(entry.getKey());
        }

        public void acceptUpdate(T val) {
            setter.accept(val);
        }

        public T getLocal() {
            return supplier.get();
        }
    }

    private Map<String, Component<?>> components = new HashMap<>();

    public SouffleEntry getEntry(String key) {
        return new SouffleEntry(this, key);
    }

    public void update() {
        for (var comp : components.values()) {
            // components;
        }
    }

    public void add(String name, DoubleSupplier getter, DoubleConsumer setter) {
        // components.put(name, new Component<>(, getter::getAsDouble, setter::accept, GenericPublisher::setDouble));
    }

    public void add(BooleanSupplier getter, BooleanConsumer setter) {
        // components.put(name, new Component<>(this, getter::getAsBoolean, setter::accept, GenericPublisher::setBoolean));
    }

    public void add(String name, Sendable sendable) {
        throw new UnsupportedOperationException("Unimplemented method 'add'");
    }
}
