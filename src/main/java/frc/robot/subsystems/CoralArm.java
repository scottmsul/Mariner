package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CoralArm extends SubsystemBase {
    // motors
    // coralpivotmotor
    // wheelmotor
    // SparkMaxConfig coralWristConfig;
    // SparkMaxConfig coralWheelConfig;
    // coralWristController;

    // sensor coralSensor = new Sensor();

    private double coralWristSetpoint;

    //SparkMax coralWheel = new SparkMax(100, MotorType.kBrushless);
    //SparkMax coralWrist = new SparkMax(1001, MotorType.kBrushless);
    TalonSRX coralWheel = new TalonSRX(0);
    TalonSRX coralWrist = new TalonSRX(0);

    private final PIDController coralWristPID = new PIDController(0, 0, 0);

    public CoralArm() {
        // SparkMaxConfig configWrist = new SparkMaxConfig();
        // configWrist
        //         .inverted(false)
        //         .idleMode(SparkMaxConfig.IdleMode.kBrake);
        //         configWrist.encoder.positionConversionFactor(1.0/100.0);
        // configWrist.closedLoop
        //         .pid(0.01, 0, 0)
        //         .outputRange(-0.25, 0.25);
        //coralWrist.configure(configWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);


        // SparkMaxConfig configWheel = new SparkMaxConfig();
        // configWheel
        //         .inverted(false)
        //         .idleMode(SparkMaxConfig.IdleMode.kBrake);
        //         configWheel.encoder.positionConversionFactor(1/10);
        //coralWheel.configure(configWheel, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(coralWrist, false, 0, 0,0);
        // Constants.configMotor(coralWheel, true);

        //coralWristController = coralWrist.getClosedLoopController();
        //coralWristController.setReference(0, ControlType.kMAXMotionPositionControl);
    }

    @Override
    public void periodic() {
        coralWrist.set(TalonSRXControlMode.PercentOutput, coralWristPID.calculate(coralWristSetpoint));
    }

    public void intakeCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    public void releaseCoral() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, -0.5);
    }

    public void stopCoralRoller() {
        coralWheel.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void setCoralWristSetpoint(double setpoint) {
        coralWristSetpoint = setpoint;
    }
    
    public boolean hasCoral() {
            // true if has coral
            // false if not
            return true;
    }
    
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
}
