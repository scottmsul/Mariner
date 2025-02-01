package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveRequest.Idle;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeSub extends SubsystemBase {
    // motors
    SparkMax algaeWrist = new SparkMax(0, MotorType.kBrushless);
    SparkMax algaeSpinMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkClosedLoopController algaeWristController;
    // pincher1
    // pivotmotor
    DigitalInput topLimitSwitch = new DigitalInput(0);
    DigitalInput bottomLimitSwitch = new DigitalInput(1);

    public AlgaeSub() {

        SparkMaxConfig configAlgaeWrist = new SparkMaxConfig();
        configAlgaeWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configAlgaeWrist.closedLoop
                .pid(0, 0, 0)
                .outputRange(-1, 1);
        algaeWrist.configure(configAlgaeWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(algaeWrist,true, 0,0,0);
        // Constants.configMotor(algaeSpinMotor, false);

        SparkMaxConfig configAlgaeSpin = new SparkMaxConfig();
        configAlgaeSpin
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        algaeSpinMotor.configure(configAlgaeSpin, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        algaeWristController = algaeWrist.getClosedLoopController();
        algaeWristController.setReference(0, ControlType.kMAXMotionPositionControl);

    }

    // ok plan a
    // methods
    // grab()
    // pincher1.set(0.5)
    // pincher2.set(0.5)
    public void grab() {
        algaeSpinMotor.set(0.5);
    }

    // grab for amount of time or until we have an algae
    // thatll be in commands though
    public void release() {
        algaeSpinMotor.set(-0.5);
    }

    //
    public void stop() {
        algaeSpinMotor.set(0);
    }

    public void setAlgaeSetpoint(double setpoint) {
        algaeWristController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
    }

    public boolean hasAlgae() {
        // if sensor detects algae
        // return true
        // if sensor doesnt detect algae
        // return false
        if (topLimitSwitch.get()) {
            return true;
        } else {
            return false;
        }

    }
    // release()
    // pincher2.set(-0.5)
    // pincher1.set(-0.5)
    // stop()
    // pincher1.set(0)
    // pincher2.set(0)

}
