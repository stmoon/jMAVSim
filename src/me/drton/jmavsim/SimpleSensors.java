package me.drton.jmavsim;

import me.drton.jmavlib.geo.GlobalPositionProjector;
import me.drton.jmavlib.processing.DelayLine;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

/**
 * User: ton Date: 27.11.13 Time: 19:06
 */
public class SimpleSensors implements Sensors {
    private DynamicObject object;
    private GlobalPositionProjector globalProjector = new GlobalPositionProjector();
    private DelayLine<GNSSReport> gpsDelayLine = new DelayLine<GNSSReport>();
    private long gpsStartTime = 0;
    private long gpsInterval = 200;
    private long lidarInterval = 50;
    private long gpsLast = 0;
    private long lidarLast = 0;

    private GNSSReport gps = new GNSSReport();
    private boolean gpsUpdated = false;
    private boolean lidarUpdated = false;

    @Override
    public void setObject(DynamicObject object) {
        this.object = object;
        globalProjector.init(object.getWorld().getGlobalReference());
    }

    public void setGPSStartTime(long time) {
        gpsStartTime = time;
    }

    public void setGPSDelay(long delay) {
        gpsDelayLine.setDelay(delay);
    }

    public void setGPSInterval(long gpsInterval) {
        this.gpsInterval = gpsInterval;
    }
    
    private double getNoise(double average, double stdev) {
	double v1;
	double v2;
	double s;
	double noiseValue;

	do {
	    v1 =  2 * Math.random() - 1;      // -1.0 ~ 1.0 까지의 값
	    v2 =  2 * Math.random() - 1;      // -1.0 ~ 1.0 까지의 값
	    s = v1 * v1 + v2 * v2;
	} while (s >= 1 || s == 0);

	s = Math.sqrt( (-2 * Math.log(s)) / s );

	noiseValue = v1 * s;
	noiseValue = (stdev * noiseValue) + average;

	return noiseValue;
    }

    @Override
    public Vector3d getAcc() {
        Vector3d accBody = new Vector3d(object.getAcceleration());
        accBody.sub(object.getWorld().getEnvironment().getG());
        Matrix3d rot = new Matrix3d(object.getRotation());
        rot.transpose();
        rot.transform(accBody);
        return accBody;
    }

    @Override
    public Vector3d getGyro() {
        return object.getRotationRate();
    }

    @Override
    public Vector3d getMag() {
        Vector3d mag = new Vector3d(object.getWorld().getEnvironment().getMagField(object.getPosition()));
        Matrix3d rot = new Matrix3d(object.getRotation());
        rot.transpose();
        rot.transform(mag);
        return mag;
    }

    @Override
    public double getPressureAlt() {
        Vector3d vel = new Vector3d(object.getVelocity());
	    return -object.getPosition().z + getNoise(vel.length()*0.3 ,1) - 100.0 ;
        //return -object.getPosition().z;
    }

    @Override
    public GNSSReport getGNSS() {
        return gps;
    }

    @Override
    public boolean isGPSUpdated() {
        boolean res = gpsUpdated;
        gpsUpdated = false;
        return res;
    }

    @Override
    public boolean isLidarUpdated() {
        boolean res = lidarUpdated;
        lidarUpdated = false;
        return res;
    }

    @Override
    // ref : http://diydrones.com/forum/topics/sf-9dof-razor-ahrs-dcm-to
    public double getDistance()
    {
	    Matrix3d rot = new Matrix3d(object.getRotation());
        double pitch = -Math.asin(rot.m20);
        double roll =  Math.atan2(rot.m21, rot.m22);
        double distance =  -object.getPosition().z * Math.sqrt(1 + Math.pow(Math.tan(Math.abs(pitch)), 2) + Math.pow(Math.tan(Math.abs(roll)), 2)) ;
	    //System.out.println(Math.toDegrees(pitch) + "," + Math.toDegrees(roll) + " ," + distance +  "," + -object.getPosition().z);
        return distance;
    }

    @Override
    public void update(long t) {
	    // GPS
        if (t > gpsStartTime && t > gpsLast + gpsInterval) {
            gpsLast = t;
            gpsUpdated = true;
            GNSSReport gpsCurrent = new GNSSReport();
            Vector3d pos = object.getPosition();
            gpsCurrent.position = globalProjector.reproject(new double[]{pos.x, pos.y, pos.z});
            gpsCurrent.eph = 1.0;
            gpsCurrent.epv = 1.0;
            gpsCurrent.velocity = object.getVelocity();
            gpsCurrent.fix = 3;
            gpsCurrent.time = System.currentTimeMillis() * 1000;
            gps = gpsDelayLine.getOutput(t, gpsCurrent);
        }

        // Lidar-Lite
        if ( t >  lidarLast + lidarInterval ) {
            lidarLast = t;
            lidarUpdated = true;
        }
    }
}
