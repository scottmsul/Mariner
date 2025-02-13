package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
    private SparkMax elevator1 = new SparkMax(60, MotorType.kBrushless);
    private SparkMax elevator2 = new SparkMax(59, MotorType.kBrushless);

    private SparkClosedLoopController elevatorController;
    private SparkClosedLoopController elevatorController2;
    private double currentSetpoint;

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
                .inverted(true)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configLead.encoder.positionConversionFactor(1.0/5.0);
        configLead.closedLoop
                .pid(0.3, 0, 0)
                .outputRange(-0.25, 0.4);
                // .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        configLead.closedLoop.maxMotion
                .maxVelocity(1)
                .maxAcceleration(1);
        elevator1.configure(configLead, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configFollow.encoder.positionConversionFactor(1.0/5.0);
        configFollow.closedLoop
                .pid(0.3, 0, 0)
                .outputRange(-0.25, 0.4);
                // .feedbackSensor(FeedbackSensor.`);
        configFollow.closedLoop.maxMotion
                .maxVelocity(1)
                .maxAcceleration(1);
        // configFollow.follow(elevator1, true);
        elevator2.configure(configFollow, ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(elevator1, false, 0, 0, 0);
        // Constants.configPIDMotor(elevator2, false, 0, 0, 0);

        elevatorController = elevator1.getClosedLoopController();
        elevatorController.setReference(0, ControlType.kPosition);
        elevatorController2 = elevator2.getClosedLoopController();
        elevatorController2.setReference(0, ControlType.kPosition);

        elevator1.getEncoder().setPosition(0);
        elevator2.getEncoder().setPosition(0);

        Shuffleboard.getTab("Debug").addDouble("Current Setpoint", () -> currentSetpoint);
        Shuffleboard.getTab("Debug").addDouble("Currenet position", () -> elevator1.getEncoder().getPosition());
        // Shuffleboard.getTab("Debug").addDouble("Current Setpoint",
        // elevatorController.);
    }

    // may set motors to follow eachother if there are two motors
    public void setPosition(double setpoint) {
        currentSetpoint = setpoint;
        System.out.println("Setting elevator setpoint to " + setpoint);
        elevatorController.setReference(setpoint, ControlType.kPosition);
        elevatorController2.setReference(setpoint, ControlType.kPosition);
    }

    public Command stow() {
        return run(() -> setPosition(0));
    }

    public Command l1() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l1));
    }

    public Command l2() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l2));
    }

    public Command l3() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l3));
    }

    public Command l4() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l4));
    }
}

