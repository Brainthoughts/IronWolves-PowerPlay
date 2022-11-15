//
// Source code recreated from a .class file by IntelliJ IDEA
// (powered by FernFlower decompiler)
//

package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import com.qualcomm.hardware.bosch.BNO055IMU.AccelerationIntegrator;
import com.qualcomm.hardware.bosch.BNO055IMU.Parameters;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.NavUtil;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

public class WolfAccelerationIntegrator implements AccelerationIntegrator {
    Parameters parameters = null;
    Position position = new Position();
    Velocity velocity = new Velocity();
    Acceleration acceleration = null;

    public Position getPosition() {
        return this.position;
    }

    public Velocity getVelocity() {
        return this.velocity;
    }

    public Acceleration getAcceleration() {
        return this.acceleration;
    }

    WolfAccelerationIntegrator() {
    }

    public void initialize(@NonNull Parameters parameters, @Nullable Position initialPosition, @Nullable Velocity initialVelocity) {
        this.parameters = parameters;
        this.position = initialPosition != null ? initialPosition : this.position;
        this.velocity = initialVelocity != null ? initialVelocity : this.velocity;
        this.acceleration = null;
    }

    public void update(Acceleration linearAcceleration) {
        if (linearAcceleration.acquisitionTime != 0L) {
            if (this.acceleration != null) {
                Acceleration accelPrev = this.acceleration;
                Velocity velocityPrev = this.velocity;
                this.acceleration = linearAcceleration;
                Acceleration acceCutoff = new Acceleration(DistanceUnit.METER,.1,.1,.1, 1L);
                if (acceCutoff.xAccel >= Math.abs(acceleration.xAccel))
                    acceleration.xAccel = 0;
                if (acceCutoff.yAccel >= Math.abs(acceleration.yAccel))
                    acceleration.yAccel = 0;
                if (acceCutoff.zAccel >= Math.abs(acceleration.zAccel))
                    acceleration.zAccel = 0;

                if (accelPrev.acquisitionTime != 0L) {
                    Velocity deltaVelocity = NavUtil.meanIntegrate(this.acceleration, accelPrev);
                    Velocity cutoff = new Velocity(DistanceUnit.METER,.1,.1,.1, 1L);
                    if (cutoff.xVeloc >= Math.abs(deltaVelocity.xVeloc))
                        deltaVelocity.xVeloc = 0;
                    if (cutoff.yVeloc >= Math.abs(deltaVelocity.yVeloc))
                        deltaVelocity.yVeloc = 0;
                    if (cutoff.zVeloc >= Math.abs(deltaVelocity.zVeloc))
                        deltaVelocity.zVeloc = 0;
                    this.velocity = NavUtil.plus(this.velocity, deltaVelocity);
                }

                if (velocityPrev.acquisitionTime != 0L) {
                    Position deltaPosition = NavUtil.meanIntegrate(this.velocity, velocityPrev);
                    Position cutoff = new Position(DistanceUnit.METER,.1,.1,.1, 1L);
                    if (cutoff.x >= Math.abs(deltaPosition.x))
                        deltaPosition.x = 0;
                    if (cutoff.y >= Math.abs(deltaPosition.y))
                        deltaPosition.y = 0;
                    if (cutoff.z >= Math.abs(deltaPosition.z))
                        deltaPosition.z = 0;
                    this.position = NavUtil.plus(this.position, deltaPosition);
                }

                if (this.parameters != null && this.parameters.loggingEnabled) {
                    RobotLog.vv(this.parameters.loggingTag, "dt=%.3fs accel=%s vel=%s pos=%s", new Object[]{(double)(this.acceleration.acquisitionTime - accelPrev.acquisitionTime) * 1.0E-9D, this.acceleration, this.velocity, this.position});
                }
            } else {
                this.acceleration = linearAcceleration;
            }
        }

    }
}
