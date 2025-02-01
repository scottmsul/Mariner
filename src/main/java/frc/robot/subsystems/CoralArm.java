package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
    // motors
    // coralpivotmotor
    // wheelmotor
    SparkMaxConfig coralWristConfig;
    SparkMaxConfig coralWheelConfig;
    SparkClosedLoopController coralWristController;

    // sensor coralSensor = new Sensor();

    private double coralArmSetpoint;

    SparkMax coralWheel = new SparkMax(0, MotorType.kBrushless);
    SparkMax coralWrist = new SparkMax(0, MotorType.kBrushless);

    public CoralArm() {
        SparkMaxConfig configWrist = new SparkMaxConfig();
        configWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configWrist.closedLoop
                .pid(0, 0, 0)
                .outputRange(-1, 1);
        coralWrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configWheel = new SparkMaxConfig();
        configWheel
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        coralWheel.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(coralWrist, false, 0, 0,0);
        // Constants.configMotor(coralWheel, true);

        coralWristController = coralWrist.getClosedLoopController();
        coralWristController.setReference(coralArmSetpoint, ControlType.kMAXMotionPositionControl);
    }

    public void intakeCoral() {
        coralWheel.set(0.5);
    }

    public void releaseCoral() {
        coralWheel.set(-0.5);
    }

    public void stopCoralRoller() {
        coralWheel.set(0);
    }

    public void setCoralWristSetpoint(double setpoint) {
        coralArmSetpoint = setpoint;
    }

    public boolean hasCoral() {
        // true if has coral
        // false if not
        return true;
    }

    // out
}
