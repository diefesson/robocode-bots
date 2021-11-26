package diefesson.rc;
import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.Graphics2D;

/**
 * JudgmentDay - a robot by Diefesson de Sousa Silva
 */
public class JudgmentDay extends AdvancedRobot {

	private final long TARGET_UPDATED_LIMIT = 10;
	private final double RADAR_BLIND_SEARCH = 50;
	private final double GUN_LIMIT = 20;
	private final double MOVE_SPEED = 8;
	private final double TURN_SPEED = 10;
	private final double POWER = 1;
	private final double OVERSCAN = 30;

	private Vec2D myPosition = null;
	private double moveDirection = 1;
	private String targetName = null;
	private Vec2D targetPosition = null;
	private Vec2D targetSpeed = null;
	private Vec2D targetShootPosition = null;
	private long lastTargetUpdate = -9999;

    // Init and update
	
	public void run() {
		init();
		long lastTime = getTime();
		long time;
		long deltaTime;
		while(true){
			time = getTime();
			deltaTime = time - lastTime;
			lastTime = time;
			out.println("time: " + time + ", delta: " + deltaTime);
			update(deltaTime);
		}
	}
	
	private void init(){
		setBodyColor(Color.DARK_GRAY);
		setGunColor(Color.GRAY);
		setRadarColor(Color.RED);
		setBulletColor(Color.ORANGE);
		setScanColor(Color.RED);
		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);
	}
	
	private void update(long deltaTime){
		// Update some basic info
		collect(deltaTime);
		
		// Update actions
		updateRadar();
		updateGun();
		updateMove();
		execute();
		
		// Predict world info for the next turn
		predict(deltaTime);
	}
	
	private void collect(long deltaTime){
		myPosition = new Vec2D(getX(), getY());
		if (isTargetUpdated()){
			double distance = myPosition.distance(targetPosition);
			double bulletDelay = DUtils.bulletDelay(distance, POWER);
			targetShootPosition = targetPosition.move(targetSpeed, bulletDelay);
			targetShootPosition = targetShootPosition.limit(getBattleFieldWidth(), getBattleFieldHeight());
		}
	}
	
	private void predict(long deltaTime){
		if (isTargetUpdated()){
			targetPosition = targetPosition.move(targetSpeed, deltaTime);
		}
	}
	
	private void updateRadar(){
		if (isTargetUpdated()){
			double radarAngle = getRadarHeading();
			double targetAngle = myPosition.look(targetPosition);
			double radarDelta = DUtils.calculateRadar(radarAngle, targetAngle, OVERSCAN);
			setTurnRadarRight(radarDelta);
		} else {
			setTurnRadarRight(RADAR_BLIND_SEARCH);
		}
	}
	
	private void updateGun(){
		if (isTargetUpdated()){
			double gunAngle = getGunHeading();
			double targetShootAngle = myPosition.look(targetShootPosition);
			double gunDelta = Utils.normalRelativeAngleDegrees(targetShootAngle - gunAngle);
			setTurnGunRight(gunDelta);
			if (getGunHeat() == 0){
				setFire(POWER);
			}
		}
	}
	
	private void updateMove(){
		if(isTargetUpdated()){
			double targetAngle = myPosition.look(targetPosition);
			double rightAngle = getHeading() + 90;
			double rightDelta = Utils.normalRelativeAngleDegrees(targetAngle - rightAngle);
			double leftAngle = getHeading() -90;
			double leftDelta = Utils.normalRelativeAngleDegrees(targetAngle - leftAngle);
			if (Math.abs(leftDelta) < Math.abs(rightDelta)){
				setTurnRight(leftDelta);
			} else {
				setTurnRight(rightDelta);
			}
		}
		setAhead(8 * moveDirection);
	}

    // Methods
	
	private boolean isTargetUpdated(){
		return (getTime() - lastTargetUpdate) < TARGET_UPDATED_LIMIT;
	}
	
	private void invertMoveDirection(){
		moveDirection = -moveDirection;
	}
	
	private void updateTarget(ScannedRobotEvent event){
		lastTargetUpdate = getTime();
		targetName = event.getName();
		
		double tarAngle = Utils.normalAbsoluteAngleDegrees(getHeading() + event.getBearing());
		double distance = event.getDistance();
		targetPosition = myPosition.add(Vec2D.fromAngle(tarAngle).mul(distance));
		
		double tarVelocity = event.getVelocity();
		double tarHeading = event.getHeading() % 360;
		targetSpeed = Vec2D.fromAngle(tarHeading).mul(tarVelocity);
	}

    // Events

	public void onScannedRobot(ScannedRobotEvent event) {
		if(isTargetUpdated()){
			String name = event.getName();
			if (targetName.equals(name)){
				updateTarget(event);
			} else {
				double distance = event.getDistance();
				double currentDistance = myPosition.distance(targetPosition);
				if (distance < currentDistance){
					updateTarget(event);
				}
			}
		} else {
			updateTarget(event);
		}
	}

	public void onHitByBullet(HitByBulletEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = DUtils.calculateRadar(radarAngle, hitAngle, OVERSCAN);
		turnRadarRight(radarDelta);
	}
	
	public void onHitRobot(HitRobotEvent event){
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = DUtils.calculateRadar(radarAngle, hitAngle, OVERSCAN);
		turnRadarRight(radarDelta);
	}
	
	public void onHitWall(HitWallEvent event) {
		invertMoveDirection();
	}
	
	public void onWin(WinEvent event){
		setAllColors(Color.RED);
	}
	
	public void onPaint(Graphics2D graphics){
		if (isTargetUpdated()){
			DUtils.drawPoint(graphics, Color.RED, targetPosition, 10);
			DUtils.drawPoint(graphics, Color.ORANGE, targetPosition.add(targetSpeed), 10);
			DUtils.drawPoint(graphics, Color.YELLOW, targetShootPosition, 10);
			DUtils.drawAngle(graphics, Color.YELLOW, myPosition, getGunHeading(), 1000);
		}
	}

}

class Vec2D{

	public final double x, y;
	
	public Vec2D(double x, double y){
		this.x = x;
		this.y = y;
	}
	
	public static Vec2D fromAngle(double angle){
		angle = Math.toRadians(angle);
		return new Vec2D(Math.sin(angle), Math.cos(angle));
	}
	
	public Vec2D add(Vec2D other){
		return new Vec2D(x + other.x, y + other.y);
	}
	
	public Vec2D sub(Vec2D other){
		return new Vec2D(x - other.x, y - other.y);
	}
	
	public Vec2D mul(double scalar){
		return new Vec2D(x * scalar, y * scalar);
	}
	
	public double distance(Vec2D other){
		return Math.hypot(x - other.x, y - other.y);
	}
	
	public Vec2D limit(double width, double height){
		return new Vec2D(
			DUtils.constrain(x, 0, width),
			DUtils.constrain(y, 0, height)
		);
	}
	
	public Vec2D move(Vec2D speed, double time){
		return add(speed.mul(time));
	}
	
	public double magnitude(){
		return Math.sqrt(x*x + y*y);
	}
	
	public Vec2D normal(){
		double magnitude = magnitude();
		return new Vec2D(x / magnitude, y / magnitude);
	}
	
	public double angle(){
		return (360 + Math.toDegrees(Math.atan2(x, y))) % 360;
	}
	
	public double look(Vec2D other){
		return other.sub(this).angle();
	}
	
	@Override
	public String toString(){
		return "x: " + this.x + ", y: " + this.y;
	}

}

class DUtils{
	
	public static double bulletDelay(double distance, double power){
		return distance / (20 - 3 * power);
	}

	public static double constrain(double value, double low, double high){
		if (value < low){
			return low;
		}
		if(high < value){
			return high;
		}
		return value;
	}
	
	public static double calculateRadar(double radarAngle, double targetAngle, double overscan){
		double radarDelta = Utils.normalRelativeAngleDegrees(targetAngle - radarAngle);
		return radarDelta + Math.signum(radarDelta) * overscan;
	}

	public static void drawPoint(Graphics2D graphics, Color color, Vec2D position, double size){
		var x = position.x - size / 2;
		var y = position.y - size / 2;
		graphics.setColor(color);
		graphics.fillRect((int) x, (int) y, (int) size, (int) size);
	}
	
	public static void drawLine(Graphics2D graphics, Color color, Vec2D from, Vec2D to){
		graphics.setColor(color);
		graphics.drawLine((int) from.x, (int) from.y, (int) to.x, (int) to.y);
	}
	
	public static void drawAngle(Graphics2D graphics, Color color, Vec2D from, double angle, double distance){
		Vec2D to = from.move(Vec2D.fromAngle(angle), distance);
		drawLine(graphics, color, from, to);
	}

}