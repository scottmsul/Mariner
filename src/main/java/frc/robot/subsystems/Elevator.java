package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax elevator1 = new SparkMax(0, MotorType.kBrushless);
    private SparkMax elevator2 = new SparkMax(0, MotorType.kBrushless);

    private SparkClosedLoopController elevatorController;

    public SparkMaxConfig configElevatorMotor(boolean Inverted, double kP, double kI, double kD) {
        SparkMaxConfig config = new SparkMaxConfig();
        config
                .inverted(Inverted)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        config.closedLoop
                .pid(kP, kI, kD)
                .outputRange(-1, 1);
        return config;
    }

    public Elevator() {
        SparkMaxConfig configLead = new SparkMaxConfig();
        configLead
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configLead.closedLoop
                .pid(0, 0, 0)
                .outputRange(-1, 1);
        configLead.closedLoop.maxMotion
                .maxVelocity(1)
                .maxAcceleration(1);
        elevator1.configure(configLead, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow
                .inverted(true)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configFollow.follow(elevator1);
        elevator2.configure(configFollow, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(elevator1, false, 0, 0, 0);
        // Constants.configPIDMotor(elevator2, false, 0, 0, 0);

        elevatorController = elevator1.getClosedLoopController();
        elevatorController.setReference(0, ControlType.kMAXMotionVelocityControl);
    }

    // may set motors to follow eachother if there are two motors
    public void setPosition(double setpoint) {
        elevatorController.setReference(setpoint, ControlType.kMAXMotionPositionControl);
    }

    public Command stow() {
        return run(() -> setPosition(0));
    }

    public Command l1() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l1));
    }
}
