package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class AlgaeSub extends SubsystemBase {
    // motors
    SparkMax algaeWrist = new SparkMax(56, MotorType.kBrushless);
    TalonSRX algaeSpinner = new TalonSRX(13);
    //SparkMax algaeSpinMotor = new SparkMax(0, MotorType.kBrushless);
    private SparkClosedLoopController algaeWristController;
    // pincher1
    // pivotmotor
    // DigitalInput topLimitSwitch = new DigitalInput(0);
    // DigitalInput bottomLimitSwitch = new DigitalInput(1);

    public AlgaeSub() {

        SparkMaxConfig configAlgaeWrist = new SparkMaxConfig();
        configAlgaeWrist
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kBrake);
        configAlgaeWrist.encoder.positionConversionFactor(1.0/125.0);
        configAlgaeWrist.closedLoop
                .pid(0.15, 0, 0)
                .outputRange(-0.25, 0.25);
        algaeWrist.configure(configAlgaeWrist, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        // Constants.configPIDMotor(algaeWrist,true, 0,0,0);
        // Constants.configMotor(algaeSpinMotor, false);

        // SparkMaxConfig configAlgaeSpin = new SparkMaxConfig();
        // configAlgaeSpin
        //         .inverted(false)
        //         .idleMode(SparkMaxConfig.IdleMode.kBrake);
        // configAlgaeSpin.encoder.positionConversionFactor(1/10);
        //algaeSpinMotor.configure(configAlgaeSpin, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        
        algaeWristController = algaeWrist.getClosedLoopController();
        algaeWristController.setReference(0, ControlType.kPosition);

        algaeWrist.getEncoder().setPosition(0);

    }

    // ok plan a
    // methods
    // grab()
    // pincher1.set(0.5)
    // pincher2.set(0.5)
    public void grab() {
        algaeSpinner.set(TalonSRXControlMode.PercentOutput, 0.5);
    }

    // // grab for amount of time or until we have an algae
    // // thatll be in commands though

    public void release() {
        algaeSpinner.set(TalonSRXControlMode.PercentOutput, -0.5);
    }

    // //
    public void stop() {
       algaeSpinner.set(TalonSRXControlMode.PercentOutput, 0);
    }

    public void stopWrist() {
        algaeWrist.set(0);
    }

    public void setAlgaeSetpoint(double setpoint) {
        algaeWristController.setReference(setpoint, ControlType.kPosition);
    }

    // public boolean hasAlgae() {
    //     // if sensor detects algae
    //     // return true
    //     // if sensor doesnt detect algae
    //     // return false
    //     if (topLimitSwitch.get()) {
    //         return true;
    //     } else {
    //         return false;
    //     }

    // }

    public Command algaeArmDown(){
        return run(() -> setAlgaeSetpoint(Constants.SetpointConstants.AlgaeArmAngles.down));
    }

    // public Command algaeArmUp(){
    //     return run(()-> setAlgaeSetpoint(3));
    // }

    public Command algaeArmStop(){
        return run(() -> stopWrist());
    }

    public Command algaeSpinIn(){
        return run(() -> grab());
    }

    public Command algaeSpinOut(){
        return run(()-> release());
    }

    public Command algaeSpinStop(){
        return run(()-> stop());
    }

    // public Command algaeArmStop(){
    //     return run(() -> stop());
    // }

    
    // release()
    // pincher2.set(-0.5)
    // pincher1.set(-0.5)
    // stop()
    // pincher1.set(0)
    // pincher2.set(0)



}
