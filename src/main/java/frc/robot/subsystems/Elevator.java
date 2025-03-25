package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.SetpointConstants;

public class Elevator extends SubsystemBase {
    private SparkMax elevator1 = new SparkMax(60, MotorType.kBrushless);
    private SparkMax elevator2 = new SparkMax(59, MotorType.kBrushless);

    private SparkClosedLoopController elevatorController;
    private SparkClosedLoopController elevatorController2;
    private double currentSetpoint;
    private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(new Constraints(1, 1.1 / 60.0));
    private double positionFactor = (1.0 / 5.0) * Math.PI * 1.0 * 0.0254;

    TrapezoidProfile.State trapezoidSetpoint = new TrapezoidProfile.State();

    // public SparkMaxConfig configElevatorMotor(boolean Inverted, double kP, double
    // kI, double kD) {
    // SparkMaxConfig config = new SparkMaxConfig();
    // config
    // .inverted(Inverted)
    // .idleMode(SparkMaxConfig.IdleMode.kCoast);
    // config.closedLoop
    // .pid(kP, kI, kD)
    // .outputRange(-1, 1);
    // return config;
    // }

    public Elevator() {
        SparkMaxConfig configLead = new SparkMaxConfig();
        configLead
                .inverted(true)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        configLead.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(positionFactor / 60);
        configLead.closedLoop
                .pid(4, 0, 0)
                .outputRange(-0.5, 0.85);
        // .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        // configLead.closedLoop.maxMotion
        // .maxVelocity(1)
        // .maxAcceleration(1);
        elevator1.configure(configLead, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        SparkMaxConfig configFollow = new SparkMaxConfig();
        configFollow
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(50);
        configFollow.encoder
                .positionConversionFactor(positionFactor)
                .velocityConversionFactor(positionFactor / 60);
        configFollow.closedLoop
                .pid(4, 0, 0)
                .outputRange(-0.5, 0.85);
        // .feedbackSensor(FeedbackSensor.`);
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

        Shuffleboard.getTab("Debug").addDouble("Current Elevator goal", () -> currentSetpoint);
        Shuffleboard.getTab("Debug").addDouble("Current elevator motor 1 position",
                () -> elevator1.getEncoder().getPosition());
        Shuffleboard.getTab("Debug").addDouble("Current elevator motor 2 position",
                () -> elevator2.getEncoder().getPosition());
        Shuffleboard.getTab("Debug").addDouble("Current elevator setpoint", () -> trapezoidSetpoint.position);
        // Shuffleboard.getTab("Debug").addDouble("Current Setpoint",
        // elevatorController.);
        var sysIdRoutine = new SysIdRoutine(
                new SysIdRoutine.Config(),
                new SysIdRoutine.Mechanism(
                        (voltage) -> this.runVolts(voltage.in(Volts)),
                        null, // No log consumer, since data is recorded by URCL
                        this));

        Shuffleboard.getTab("SysId").add("Quasi Forward Elevator",
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward));
        Shuffleboard.getTab("SysId").add("Quasi Backward Elevator",
                sysIdRoutine.quasistatic(SysIdRoutine.Direction.kReverse));
        Shuffleboard.getTab("SysId").add("Dynamic Forward Elevator",
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward));
        Shuffleboard.getTab("SysId").add("Dynamic Backaward Elevator",
                sysIdRoutine.dynamic(SysIdRoutine.Direction.kReverse));
    }

    private void runVolts(double in) {
        elevator1.setVoltage(in);
        elevator2.setVoltage(in);
    }

    // may set motors to follow eachother if there are two motors
    public void setPosition(double setpoint) {
        currentSetpoint = setpoint;
        // elevatorController.setReference(setpoint, ControlType.kPosition);
        // elevatorController2.setReference(setpoint, ControlType.kPosition);
    }

    public double elevatorHeightGet() {
        return elevator1.getEncoder().getPosition();
    }

    public boolean isReady() {
        double elevator1Position = elevator1.getEncoder().getPosition();
        double elevator2Position = elevator2.getEncoder().getPosition();
        boolean elevator1Ready = elevator1Position > (currentSetpoint - 0.1)
                && elevator1Position < (currentSetpoint + 0.1);
        boolean elevator2Ready = elevator2Position > (currentSetpoint - 0.1)
                && elevator2Position < (currentSetpoint + 0.1);
        return elevator1Ready && elevator2Ready;
    }

    // public boolean tooHigh() {
    // boolean tooHigh = currentSetpoint>0.5;
    // return tooHigh;
    // }

    @Override
    public void periodic() {
        trapezoidSetpoint = trapezoidProfile.calculate(0.2, trapezoidSetpoint,
                new TrapezoidProfile.State(currentSetpoint, 0));
        elevatorController.setReference(trapezoidSetpoint.position, ControlType.kPosition);
        elevatorController2.setReference(trapezoidSetpoint.position, ControlType.kPosition);
    }

    public Command stow() {
        return run(() -> setPosition(0));
    }

    public Command l1() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l1.get()));
    }

    public Command l2() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l2.get()));
    }

    public Command l3() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l3.get()));
    }

    public Command l4() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.l4.get()));
    }

    public Command algaeLow() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.algaeLow.get()));
    }

    public Command algaeHigh() {
        return run(() -> setPosition(Constants.SetpointConstants.ElevatorSetpoints.algaeHigh.get()));
    }
}
