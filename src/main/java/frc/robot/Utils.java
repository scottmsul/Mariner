package frc.robot;

import java.util.EnumSet;

import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class Utils {
  public Command rumble(GenericHID controller, double intensity, double duration) {
    return Commands.startEnd(
        () -> controller.setRumble(RumbleType.kBothRumble, intensity),
        () -> controller.setRumble(RumbleType.kBothRumble, 0))
        .withTimeout(duration);
  }

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