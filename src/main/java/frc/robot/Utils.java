package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Utils {
  static <T> void makeClassTunable(Class<T> thisClass) {
    try {
      // var thisClass = new Object() {}.getClass().getEnclosingClass();
      var parts = thisClass.getName().split("\\$");
      var className = parts[parts.length-1];
      for (var f : thisClass.getFields()) {
        if (f.getType() != double.class) {
          continue;
        }
        var entry = NetworkTableInstance.getDefault().getEntry("/Tune/" + className + "/" + f.getName());
        entry.setDouble(f.getDouble(thisClass));
        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
            e -> {
              try {
                f.set(thisClass, e.valueData.value.getDouble());
              } catch (IllegalArgumentException | IllegalAccessException e1) {
                e1.printStackTrace();
              }
            });
      }
    } catch (IllegalArgumentException | IllegalAccessException e) {
      e.printStackTrace();
    }
  }
    
}