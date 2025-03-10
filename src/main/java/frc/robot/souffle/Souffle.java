package frc.robot.souffle;

import java.nio.file.Path;
import java.nio.file.Paths;

import com.pathplanner.lib.config.PIDConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Souffle {
    static {
        // new TrapezoidProfile(Constraints(x));
        // new Marvin()
        var max = new SparkMax(0, null);
        var ctrl = max.getClosedLoopController();

        Shuffleboard.getTab("foo").add("f", 0).getEntry();
        Souffle.e("foo").addDouble("f", () -> 0);
        // Shuffleboard.getTab("Foo").add(builder -> {
        // double p = max.configAccessor.closedLoop.getP();
        // builder.addDoubleProperty("P", () -> p, (v) -> {
        // p = v;
        // max.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().p(v)),
        // ResetMode.kNoResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // });
        // });

        // new SendableBuilder().addDoubleProperty("P", () ->
        // max.configAccessor.closedLoop.getP(),
        // v -> max.configure(new SparkBaseConfig().closedLoop.p(v)));
    }
    protected static SouffleInstance root = new SouffleInstance();

    // static {

    // Shuffleboard.getTab(null).addDouble("", ()->1);
    // Souffle.e("/My/Data").e("val").addDouble("MyName", () ->1,
    // (v)->System.out.println("got " + v));
    // }

    public static void update() {
        root.update();
    }

    public static SouffleEntry e(String key) {
        return root.getEntry(key);
    }
}
