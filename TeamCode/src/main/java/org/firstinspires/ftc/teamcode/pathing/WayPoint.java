package org.firstinspires.ftc.teamcode.pathing;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;

public class WayPoint {
    Pose2d position;
    Transform2d tolerance;
    public WayPoint(Pose2d pos, Transform2d tol){
        this.position=pos;
        this.tolerance=tol;
    }
    public WayPoint(Pose2d pos, double tol){
        this.position=pos;
        this.tolerance=new Transform2d(new Translation2d(tol, tol), Rotation2d.fromDegrees(tol));
    }
    public Pose2d getPosition(){
        return position;
    }
    public Transform2d getTolerance(){
        return tolerance;
    }

}