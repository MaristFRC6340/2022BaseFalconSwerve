package frc.robot.commands;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class PhotonFollow extends CommandBase {
    private double rotation;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    
    private Swerve s_Swerve;
    private Joystick controller;
    private int translationAxis;
    private int strafeAxis;
    private int rotationAxis;
    private PhotonCamera camera = new PhotonCamera("Live!_Cam_Chat_HD_VF0790");
    
    public PhotonFollow() {
    }

    @Override
    public void execute() {
        var result = camera.getLatestResult();

        boolean hasTargets = result.hasTargets();
        double yaw = 0;

        if (hasTargets){
            PhotonTrackedTarget target = result.getBestTarget();
            /*
            error = target.getYaw();
            System.out.println(error);
            */
            yaw = target.getYaw();
        }
        double rInput=(yaw-180)/180;
        double yAxis = 0;
        double xAxis = 0;
        double rAxis = rInput;
        
        /* Deadbands */
        yAxis = (Math.abs(yAxis) < Constants.stickDeadband) ? 0 : yAxis;
        xAxis = (Math.abs(xAxis) < Constants.stickDeadband) ? 0 : xAxis;
        rAxis = (Math.abs(rAxis) < Constants.stickDeadband) ? 0 : rAxis;

        translation = new Translation2d(yAxis, xAxis).times(Constants.Swerve.maxSpeed);
        rotation = rAxis * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, fieldRelative, openLoop);

    }
}
