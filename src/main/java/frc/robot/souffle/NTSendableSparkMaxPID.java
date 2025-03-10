package frc.robot.souffle;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class NTSendableSparkMaxPID implements Sendable {
    private SparkMax spark;
    private double p;
    private double i;
    private double d;
    private double izone;

    NTSendableSparkMaxPID(SparkMax spark) {
        this.spark = spark;
    }

    private double getP() {
        return p;
    }

    private double getI() {
        return i;
    }

    private double getD() {
        return d;
    }

    private void setP(double p) {
        this.p = p;
        spark.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().p(p)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);

    }

    private void setI(double i) {
        this.i = i;
        spark.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().i(i)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    private void setD(double d) {
        this.d = d;
        spark.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().d(d)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    private double getIZone() {
        return izone;
    }

    private void setIZone(double izone) {
        this.izone = izone;
        spark.configure(new SparkMaxConfig().apply(new ClosedLoopConfig().iZone(izone)),
                ResetMode.kNoResetSafeParameters,
                PersistMode.kNoPersistParameters);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("PIDController");
        builder.addDoubleProperty("p", this::getP, this::setP);
        builder.addDoubleProperty("i", this::getI, this::setI);
        builder.addDoubleProperty("d", this::getD, this::setD);
        builder.addDoubleProperty(
                "izone",
                this::getIZone,
                (double toSet) -> {
                    try {
                        setIZone(toSet);
                    } catch (IllegalArgumentException e) {
                        MathSharedStore.reportError("IZone must be a non-negative number!",
                                e.getStackTrace());
                    }
                });
    }
}