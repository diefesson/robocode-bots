package diefesson.rc;

import robocode.*;
import robocode.util.Utils;
import java.awt.Color;
import java.awt.Graphics2D;

/**
 * JudgmentDay - a robot by Diefesson de Sousa Silva
 */
public class JudgmentDay extends AdvancedRobot {

	// Config
	private static final long TARGET_UPDATED_LIMIT = 10;
	private static final double RADAR_BLIND_SEARCH = 50;
	private static final double NEAR_POWER = 3;
	private static final double POWER = 2;
	private static final double FAR_POWER = 1;
	private static final double OVERSCAN = 30;
	private static final double CORRECTION_RATE = 45;
	private static final double NEAR = 100;
	private static final double FAR = 300;

	// State
	private double moveDirection = 1;

	// Input
	private String targetName;
	private Vec2D targetPosition;
	private Vec2D targetSpeed;

	// Calculated
	private Vec2D myPosition;
	private Vec2D targetShootPosition;
	private double targetDistance;
	private double targetAngle;
	private long lastTargetUpdate = -9999;

	// Init and update

	public void run() {
		init();
		long lastTime = getTime();
		long time;
		long deltaTime;
		while (true) {
			time = getTime();
			deltaTime = time - lastTime;
			lastTime = time;
			out.println("time: " + time + ", delta: " + deltaTime);
			update();
		}
	}

	private void init() {
		setBodyColor(Color.DARK_GRAY);
		setGunColor(Color.GRAY);
		setRadarColor(Color.RED);
		setBulletColor(Color.ORANGE);
		setScanColor(Color.RED);
		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);
	}

	private void update() {
		// calculate info from input
		calculate();

		// Update actions
		updateRadar();
		updateGun();
		updateMove();
		execute();

		// Predict world info for the next turn
		predict();
	}

	private void calculate() {
		myPosition = new Vec2D(getX(), getY());
		if (isTargetUpdated()) {
			calculateTarget();
		}
	}

	private void predict() {
		if (isTargetUpdated()) {
			targetPosition = targetPosition.add(targetSpeed);
		}
	}

	private void updateRadar() {
		if (isTargetUpdated()) {
			double radarAngle = getRadarHeading();
			double radarDelta = DUtils.calculateRadar(radarAngle, targetAngle, OVERSCAN);
			setTurnRadarRight(radarDelta);
		} else {
			setTurnRadarRight(RADAR_BLIND_SEARCH);
		}
	}

	private void updateGun() {
		if (isTargetUpdated()) {
			double gunAngle = getGunHeading();
			double targetShootAngle = myPosition.look(targetShootPosition);
			double gunDelta = Utils.normalRelativeAngleDegrees(targetShootAngle - gunAngle);
			setTurnGunRight(gunDelta);
			if (getGunHeat() == 0) {
				double power = POWER;
				if (targetDistance > FAR){
					power = FAR_POWER;
				} else if (targetDistance < NEAR){
					power = NEAR_POWER;
				}
				setFire(power);
			}
		}
	}

	private void updateMove() {
		if (isTargetUpdated()) {
			targetAngle = myPosition.look(targetPosition);
			double rightAngle = getHeading() + 90;
			double rightDelta = Utils.normalRelativeAngleDegrees(targetAngle - rightAngle);
			double leftAngle = getHeading() - 90;
			double leftDelta = Utils.normalRelativeAngleDegrees(targetAngle - leftAngle);
			double side = (Math.abs(leftDelta) < Math.abs(rightDelta)) ? -1 : 1;
			double delta = (side < 0) ? leftDelta : rightDelta;
			if (targetDistance > FAR) {
				delta += moveDirection + side * moveDirection * CORRECTION_RATE;
			} else if (targetDistance < NEAR) {
				delta += moveDirection + side * moveDirection * -CORRECTION_RATE;
			}
			setTurnRight(delta);
		} else {
			setTurnRight(0);
		}
		setAhead(8 * moveDirection);
	}

	// Methods

	private boolean isTargetUpdated() {
		return (getTime() - lastTargetUpdate) < TARGET_UPDATED_LIMIT;
	}

	private void calculateTarget(){
		targetDistance = myPosition.distance(targetPosition);
		targetAngle = myPosition.look(targetPosition);
		double bulletDelay = DUtils.bulletDelay(targetDistance, POWER);
		targetShootPosition = targetPosition.move(targetSpeed, bulletDelay);
		targetShootPosition = targetShootPosition.limit(getBattleFieldWidth(), getBattleFieldHeight());
	}

	private void invertMoveDirection() {
		moveDirection = -moveDirection;
	}

	private void updateTarget(ScannedRobotEvent event) {
		lastTargetUpdate = getTime();
		targetName = event.getName();

		double tarAngle = Utils.normalAbsoluteAngleDegrees(getHeading() + event.getBearing());
		double distance = event.getDistance();
		targetPosition = myPosition.add(Vec2D.fromAngle(tarAngle).mul(distance));

		double tarVelocity = event.getVelocity();
		double tarHeading = event.getHeading() % 360;
		targetSpeed = Vec2D.fromAngle(tarHeading).mul(tarVelocity);
		calculateTarget();
	}

	// Events

	public void onScannedRobot(ScannedRobotEvent event) {
		if (!isTargetUpdated() || targetName.equals(event.getName()) || event.getDistance() < targetDistance) {
			updateTarget(event);
		}
	}

	public void onHitByBullet(HitByBulletEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = DUtils.calculateRadar(radarAngle, hitAngle, OVERSCAN);
		setTurnRadarRight(radarDelta);
	}

	public void onHitRobot(HitRobotEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = DUtils.calculateRadar(radarAngle, hitAngle, OVERSCAN);
		setTurnRadarRight(radarDelta);
	}

	public void onHitWall(HitWallEvent event) {
		invertMoveDirection();
	}

	public void onWin(WinEvent event) {
		setAllColors(Color.RED);
	}

	public void onSkippedTurn(SkippedTurnEvent event) {
		out.println("ALERT: skipped " + event.getSkippedTurn() + "turns");
	}

	public void onPaint(Graphics2D graphics) {
		if (isTargetUpdated()) {
			DUtils.drawPoint(graphics, Color.RED, targetPosition, 10);
			DUtils.drawPoint(graphics, Color.ORANGE, targetPosition.add(targetSpeed), 10);
			DUtils.drawPoint(graphics, Color.YELLOW, targetShootPosition, 10);
			DUtils.drawAngle(graphics, Color.YELLOW, myPosition, getGunHeading(), 1000);
			DUtils.drawCircle(graphics, Color.GRAY, myPosition, NEAR);
			DUtils.drawCircle(graphics, Color.YELLOW, myPosition, targetDistance);
			DUtils.drawCircle(graphics, Color.GRAY, myPosition, FAR);
		}
	}

	private static class Vec2D {

		public final double x, y;

		public Vec2D(double x, double y) {
			this.x = x;
			this.y = y;
		}

		public static Vec2D fromAngle(double angle) {
			angle = Math.toRadians(angle);
			return new Vec2D(Math.sin(angle), Math.cos(angle));
		}

		public Vec2D add(Vec2D other) {
			return new Vec2D(x + other.x, y + other.y);
		}

		public Vec2D sub(Vec2D other) {
			return new Vec2D(x - other.x, y - other.y);
		}

		public Vec2D mul(double scalar) {
			return new Vec2D(x * scalar, y * scalar);
		}

		public double distance(Vec2D other) {
			return Math.hypot(x - other.x, y - other.y);
		}

		public Vec2D limit(double width, double height) {
			return new Vec2D(DUtils.constrain(x, 0, width), DUtils.constrain(y, 0, height));
		}

		public Vec2D move(Vec2D speed, double time) {
			return add(speed.mul(time));
		}

		public double angle() {
			return (360 + Math.toDegrees(Math.atan2(x, y))) % 360;
		}

		public double look(Vec2D other) {
			return other.sub(this).angle();
		}

		@Override
		public String toString() {
			return "x: " + this.x + ", y: " + this.y;
		}

	}

	private static class DUtils {

		public static double bulletDelay(double distance, double power) {
			return distance / (20 - 3 * power);
		}

		public static double constrain(double value, double low, double high) {
			if (value < low) {
				return low;
			}
			if (high < value) {
				return high;
			}
			return value;
		}

		public static double calculateRadar(double radarAngle, double targetAngle, double overscan) {
			double radarDelta = Utils.normalRelativeAngleDegrees(targetAngle - radarAngle);
			return radarDelta + Math.signum(radarDelta) * overscan;
		}

		public static void drawPoint(Graphics2D graphics, Color color, Vec2D position, double size) {
			var x = position.x - size / 2;
			var y = position.y - size / 2;
			graphics.setColor(color);
			graphics.fillRect((int) x, (int) y, (int) size, (int) size);
		}

		public static void drawLine(Graphics2D graphics, Color color, Vec2D from, Vec2D to) {
			graphics.setColor(color);
			graphics.drawLine((int) from.x, (int) from.y, (int) to.x, (int) to.y);
		}

		public static void drawAngle(Graphics2D graphics, Color color, Vec2D from, double angle, double distance) {
			Vec2D to = from.move(Vec2D.fromAngle(angle), distance);
			drawLine(graphics, color, from, to);
		}

		public static void drawCircle(Graphics2D graphics, Color color, Vec2D center, double radius) {
			graphics.setColor(color);
			int x = (int) (center.x - radius);
			int y = (int) (center.y - radius);
			int size = (int) (radius * 2);
			graphics.drawArc(x, y, size, size, 0, 360);
		}

	}

}
