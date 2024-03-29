package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.Joystick;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerveSlow extends CommandBase {

    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;

    private double mapdouble(double x, double in_min, double in_max, double out_min, double out_max){
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }

    /**
     * Driver control
     */
    public TeleopSwerveSlow(Swerve s_Swerve, Joystick controller, int translationAxis, int strafeAxis, int rotationAxis, boolean fieldRelative, boolean openLoop) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.controller = controller;
        this.translationAxis = translationAxis;
        this.strafeAxis = strafeAxis;
        this.rotationAxis = rotationAxis;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
    }

    @Override
    public void execute() {
        double yAxis = controller.getRawAxis(translationAxis) / 3;
        double xAxis = -controller.getRawAxis(strafeAxis) / 3;
        double rAxis = -controller.getRawAxis(rotationAxis) / 3;
        
        /* Deadbands */

        if (Math.abs(yAxis) > Constants.stickDeadband){

            if (yAxis > 0){
                yAxis = mapdouble(yAxis, Constants.stickDeadband, 1, 0, 1);
            }
            if (yAxis < 0){
                yAxis = mapdouble(yAxis, -Constants.stickDeadband, -1, 0, -1);
            }
        }
        else{
            yAxis = 0;
        }

        
        if (Math.abs(xAxis) > Constants.stickDeadband){

            if (xAxis > 0){
                xAxis = mapdouble(xAxis, Constants.stickDeadband, 1, 0, 1);
            }
            if (xAxis < 0){
                xAxis = mapdouble(xAxis, -Constants.stickDeadband, -1, 0, -1);
            }
        }
        else{
            xAxis = 0;
        }

        if (Math.abs(rAxis) > Constants.stickDeadband){

            if (rAxis > 0){
                rAxis = mapdouble(rAxis, Constants.stickDeadband, 1, 0, 1);
            }
            if (rAxis < 0){
                rAxis = mapdouble(rAxis, -Constants.stickDeadband, -1, 0, -1);
            }
        }
        else{
            rAxis = 0;
        }

        

        // yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        // xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        // rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);
    }
}
