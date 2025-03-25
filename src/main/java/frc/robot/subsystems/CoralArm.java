package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
    // motors
    // coralpivotmotor
    // wheelmotor
    // SparkMaxConfig coralWristConfig;
    // SparkMaxConfig coralWheelConfig;
    SparkClosedLoopController coralWristController;

    // sensor coralSensor = new Sensor();

    private double coralWristSetpoint;
    private boolean needsRestoreSetpoint = false;

    // SparkMax coralWheel = new SparkMax(100, MotorType.kBrushless);
    // SparkMax coralWrist = new SparkMax(1001, MotorType.kBrushless);
    TalonSRX coralWheel = new TalonSRX(40);
    SparkMax coralWrist = new SparkMax(55, MotorType.kBrushless);

    // LimitSwitch coralLimitSwitch = coralWheel.LimitSwitch;

    private final PIDController coralWristPID = new PIDController(0.1, 0, 0);

    private Elevator elevator;

    public CoralArm(Elevator elevator) {
        this.elevator = elevator;
        Shuffleboard.getTab("PID").add("Coral", coralWristPID);
        SparkMaxConfig configWrist = new SparkMaxConfig();
        configWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake)
                .smartCurrentLimit(20);
        configWrist.signals
                .absoluteEncoderPositionAlwaysOn(true);
        configWrist.encoder.positionConversionFactor(1.0 / 100.0);
        configWrist.absoluteEncoder.inverted(true)
                .zeroOffset(0.501)
                .zeroCentered(true);
        configWrist.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(3, 0.0001, 0.001)
                .outputRange(-0.8, 0.7);
        coralWrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // coralWrist.getEncoder().setPosition(coralWrist.getAbsoluteEncoder().getPosition());

        // SparkMaxConfig configWheel = new SparkMaxConfig();
        // configWheel
        // .inverted(false)
        // .idleMode(SparkMaxConfig.IdleMode.kBrake);
        // configWheel.encoder.positionConversionFactor(1/10);
        // coralWheel.configure(configWheel, ResetMode.kResetSafeParameters,
        // PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(coralWrist, false, 0, 0,0);
        // Constants.configMotor(coralWheel, true);

        coralWristController = coralWrist.getClosedLoopController();
        coralWristController.setReference(Constants.SetpointConstants.CoralPivotAngles.up.get(), ControlType.kPosition);

        coralWheel.setInverted(true);
        coralWheel.setSelectedSensorPosition(0);

        // coralWheel.getSensorCollection().getp

        // Shuffleboard.getTab("Debug").addDouble("cwsetpoint", () ->
        // coralWristSetpoint);
        // Shuffleboard.getTab("Debug").addDouble("Cweabs", () ->
        // coralWrist.getAbsoluteEncoder().getPosition());
        // Shuffleboard.getTab("Debug").addDouble("test", () -> 0.01);
        // Shuffleboard.getTab("Debug").addDouble("cwe", () ->
        // coralWrist.getEncoder().getPosition());
        // Shuffleboard.getTab("Debug").addDouble("Coral Wrist Power", () ->
        // coralWrist.getAppliedOutput());
        Shuffleboard.getTab("Drive").addBoolean("Has Coral", this::hasCoral);
    }

    @Override
    public void periodic() {
        if (!elevator.isReady()) {
            // if coral arm setpoint is below horizontal and will hit the elevator
            if (!needsRestoreSetpoint && coralWristSetpoint > 0) {
                System.out.println("Saving setpoint " + coralWristSetpoint);
                needsRestoreSetpoint = true;
                applyWristSetpointToMotor(0);
            }
        } else {
            if (needsRestoreSetpoint) {
                System.out.println("Restoring setpoint " + coralWristSetpoint);
                needsRestoreSetpoint = false;
                applyWristSetpointToMotor(coralWristSetpoint);
            }
        }

        var inst = NetworkTableInstance.getDefault();
        inst.getEntry("/Debug/CoralArm/cwsetpoint").setDouble(coralWristSetpoint);
        inst.getEntry("/Debug/CoralArm/Cweabs").setDouble(coralWrist.getAbsoluteEncoder().getPosition());
        inst.getEntry("/Debug/CoralArm/test").setDouble(0.01);
        inst.getEntry("/Debug/CoralArm/cwe").setDouble(coralWrist.getEncoder().getPosition());
        inst.getEntry("/Debug/CoralArm/Coral Wrist Power").setDouble(coralWrist.getAppliedOutput());

    }

    // @Override
    // public void periodic() {
    // // MathUtil.clamp(coralWristSetpoint, -0.25, 0.25);
    // double setMotor =
    // MathUtil.clamp(coralWristPID.calculate(coralWheel.getSelectedSensorPosition()),
    // -0.25, 0.25);
    // coralWrist.set(setMotor);
    // }

    public void intakeCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    public void releaseCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, -0.3);
    }

    public void stopCoralRoller() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void stopCoralWrist() {
        coralWrist.set(0);
    }

    public void setCoralWristSetpoint(double setpoint) {
        coralWristSetpoint = setpoint;
        // coralWristPID.setSetpoint(setpoint);
        applyWristSetpointToMotor(setpoint);
    }

    private void applyWristSetpointToMotor(double setpoint) {
        coralWristController.setReference(setpoint, ControlType.kPosition);
    }

    public boolean isReady() {
        double coralPosition = coralWrist.getAbsoluteEncoder().getPosition();
        return coralPosition > (coralWristSetpoint - 0.1) && coralPosition < (coralWristSetpoint + 0.1);
    }

    public boolean hasCoral() {
        return coralWheel.isFwdLimitSwitchClosed() == 1;
    }

    public boolean hasNoCoral() {
        return coralWheel.isFwdLimitSwitchClosed() == 0;
    }

    // public boolean hasCoral() {
    // true if has coral
    // false if not
    // return coralLimitSwitch.isPressed();
    // }

    // public boolean hasNoCoral() {
    // //return !coralLimitSwitch.isPressed();
    // }

    public Command l1() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.l1.get()));
    }

    // out
    public Command lmid() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.lmid.get()));
    }

    public Command l4() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.l4.get()));
    }

    public Command coralStation() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.CoralSt.get()));
    }

    public Command up() {
        return run(() -> setCoralWristSetpoint(0));
    }

    // public Command backwards() {
    // return run(() -> setCoralWristSetpoint(-1));
    // }
    public Command intakeCoralCommand() {
        return run(() -> intakeCoral());
    }

    public Command outtakeCoralCommand() {
        return run(() -> releaseCoral());
    }

    public Command stopCoralSpin() {
        return run(() -> stopCoralRoller());
    }
}
