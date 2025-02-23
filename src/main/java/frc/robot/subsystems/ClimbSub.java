package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.controls.Follower;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimbSub extends SubsystemBase {
    // this is the climb system :0
    // idek man
    // guy waas here

    // Motor pushArm = new Motor();
    //SparkMax climbMotorOne = new SparkMax(102, MotorType.kBrushless);
    //SparkMax climbMotorTwo = new SparkMax(103, MotorType.kBrushless);

    double climbSetpoint;
    SparkClosedLoopController climbController;
    SparkClosedLoopController climbController2;
    private final SparkMax climb1 = new SparkMax(58, MotorType.kBrushless);
    private final SparkMax climb2 = new SparkMax(57, MotorType.kBrushless);

    // either have the non pid motor follow the pid one or have 2 pid controllers
    // idk

    // setpoint = Constants.SetpointConstants.coralSetpointArray[index]
    public ClimbSub() {
        // Constants.configPIDMotor(climbMotorOne,false, 0,0,0);

        SparkMaxConfig configLeader = new SparkMaxConfig();
        configLeader
                .inverted(true)
                .idleMode(SparkMaxConfig.IdleMode.kCoast);
        configLeader.closedLoop.pid(0.5,0, 0).outputRange(0, 0.7);
        configLeader.smartCurrentLimit(30, 30);
        configLeader.secondaryCurrentLimit(35);
        configLeader.encoder.positionConversionFactor(1.0/60.0);
        climb1.configure(configLeader, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        SparkMaxConfig configFollower = new SparkMaxConfig();
        configFollower
                .inverted(false)
                .idleMode(SparkMaxConfig.IdleMode.kCoast);
        configFollower.closedLoop.pid(0.5,0, 0).outputRange(0, 0.7);
        configFollower.encoder.positionConversionFactor(1.0/60.0);
        configFollower.smartCurrentLimit(30, 30);
        configFollower.secondaryCurrentLimit(35);
        //configFollower.follow(climbMotorOne);
        climb2.configure(configFollower, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        climbController = climb1.getClosedLoopController();
        climbController2 = climb2.getClosedLoopController();

        // climbMotorTwo.setControl(new Follower(climbMotorOne.getDeviceId(),true));
        climbController.setReference(0, ControlType.kPosition);
        climbController2.setReference(0, ControlType.kPosition);

        Shuffleboard.getTab("Debug").addDouble("Climb 1 Current", () -> climb1.getEncoder().getPosition());
        Shuffleboard.getTab("Debug").addDouble("Climb 2 Current", () -> climb2.getEncoder().getPosition());
    }

    public void climbDown() {
        // set climb motors to down
        climb1.set(0.8);
        climb2.set(0.8);
    }

    // public void climbUp() {
    //     // set climb motors up??
    //     climb1.set(-0.8);
    //     climb2.set(-0.8);
    // }

    public void climbStop(){
        climb1.set(0);
        climb2.set(0);
    }

    public void setClimbSetpoint(double Setpoint){
        climbController.setReference(Setpoint, ControlType.kPosition);
        climbController2.setReference(Setpoint, ControlType.kPosition);
    }
    // i wonder still
    // we could have it push down until it reaches a setpoint
    // or have the pull up initiated as the driver presses a button then have it
    // freeze in place when not being moved
    // that way the drivers can tell it when to stop

    public Command climb(){
        return run(() -> setClimbSetpoint(0.4));
    }
    public Command climbStopCommand(){
        return run(() -> setClimbSetpoint(0));
    }

}