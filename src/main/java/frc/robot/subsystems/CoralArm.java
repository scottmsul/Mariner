package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
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

    // SparkMax coralWheel = new SparkMax(100, MotorType.kBrushless);
    // SparkMax coralWrist = new SparkMax(1001, MotorType.kBrushless);
    TalonSRX coralWheel = new TalonSRX(40);
    SparkMax coralWrist = new SparkMax(55, MotorType.kBrushless);

    //LimitSwitch coralLimitSwitch = coralWheel.LimitSwitch;

    private final PIDController coralWristPID = new PIDController(0.1, 0, 0);

    public CoralArm() {
        Shuffleboard.getTab("PID").add("Coral", coralWristPID);
        SparkMaxConfig configWrist = new SparkMaxConfig();
        configWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configWrist.encoder.positionConversionFactor(1.0 / 100.0);
        configWrist.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(1.45, 0, 0)
                .outputRange(-0.8, 0.7);
        coralWrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        Shuffleboard.getTab("Debug").addDouble("CoralEncoder", () -> coralWrist.getAbsoluteEncoder().getPosition());

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
        coralWristController.setReference(0, ControlType.kPosition);

        coralWheel.setSelectedSensorPosition(0);


        // coralWheel.getSensorCollection().getp

        Shuffleboard.getTab("Debug").addDouble("Coral Wrist Setpoint", () -> coralWristSetpoint);
        Shuffleboard.getTab("Debug").addDouble("Coral Wrist Current", () -> coralWrist.getEncoder().getPosition());
        Shuffleboard.getTab("Debug").addDouble("Coral Wrist Power", () -> coralWrist.getAppliedOutput());
    }

    // @Override
    // public void periodic() {
    //     // MathUtil.clamp(coralWristSetpoint, -0.25, 0.25);
    //     double setMotor = MathUtil.clamp(coralWristPID.calculate(coralWheel.getSelectedSensorPosition()), -0.25, 0.25);
    //     coralWrist.set(setMotor);
    // }

    public void intakeCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    public void releaseCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, -0.5);
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
        coralWristController.setReference(setpoint, ControlType.kPosition);
    }

    public boolean isReady(){
        double coralPosition = coralWrist.getEncoder().getPosition();
        return coralPosition > (coralWristSetpoint - 0.1) && coralPosition < (coralWristSetpoint + 0.1);
    }

    //public boolean hasCoral() {
        // true if has coral
        // false if not
        //return coralLimitSwitch.isPressed();
    //}

    // public boolean hasNoCoral() {
    //     //return !coralLimitSwitch.isPressed();
    // }

    public Command l1() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.l1));
    }

    // out
    public Command lmid() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.lmid));
    }

    public Command l4() {
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.l4));
    }

    public Command coralStation(){
        return run(() -> setCoralWristSetpoint(Constants.SetpointConstants.CoralPivotAngles.CoralSt));
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
