package diefesson;

import robocode.*;
import java.awt.Color;
import java.awt.Graphics2D;

/**
 * TerminatorUnit - a robot by Diefesson de Sousa Silva
 */
public class TerminatorUnit extends AdvancedRobot {

	private final long LAST_SEEN_LIMIT = 10;
	private final double RADAR_LIMIT = 30;
	private final double GUN_LIMIT = 20;
	private final double MOVE_SPEED = 8;
	private final double TURN_SPEED = 10;
	private final double POWER = 1;
	private final double RADAR_OVERSCAN = 1.1;

	private Vec2D myPosition = null;
	private double moveDirection = 1;
	private Vec2D targetPosition = null;
	private Vec2D targetSpeed = null;
	private Vec2D targetShootPosition = null;
	private long lastSeen = 9999;

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
			update(deltaTime);
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

	private void update(long deltaTime) {
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

	private void collect(long deltaTime) {
		lastSeen += deltaTime;
		myPosition = new Vec2D(getX(), getY());
		if (isInfoNew()) {
			double distance = myPosition.distance(targetPosition);
			double bulletDelay = Util.bulletDelay(distance, POWER);
			targetShootPosition = targetPosition.move(targetSpeed, bulletDelay);
			targetShootPosition = targetShootPosition.limit(getBattleFieldWidth(), getBattleFieldHeight());
		}
	}

	private void predict(long deltaTime) {
		if (isInfoNew()) {
			targetPosition = targetPosition.move(targetSpeed, deltaTime);
		}
	}

	private void updateRadar() {
		if (isInfoNew()) {
			double radarAngle = getRadarHeading();
			double targetAngle = myPosition.look(targetPosition);
			double radarDelta = Util.angleDelta(radarAngle, targetAngle);
			radarDelta = Util.limit(radarDelta * RADAR_OVERSCAN, RADAR_LIMIT);
			setTurnRadarRight(radarDelta);
		} else {
			setTurnRadarRight(RADAR_LIMIT);
		}
	}

	private void updateGun() {
		if (isInfoNew()) {
			double gunAngle = getGunHeading();
			double targetShootAngle = myPosition.look(targetShootPosition);
			double gunDelta = Util.angleDelta(gunAngle, targetShootAngle);
			gunDelta = Util.limit(gunDelta, GUN_LIMIT);
			setTurnGunRight(gunDelta);
			if (getGunHeat() == 0) {
				fire(POWER);
			}
		}
	}

	private void updateMove() {
		if (isInfoNew()) {
			double targetAngle = myPosition.look(targetPosition);
			double sideAngle = getHeading() + 90;
			double deltaAngle = Util.angleDelta(sideAngle, targetAngle);
			setTurnRight(deltaAngle);
		}
		setAhead(8 * moveDirection);
	}

	// Methods

	private boolean isInfoNew() {
		return lastSeen < LAST_SEEN_LIMIT;
	}

	private void invertMoveDirection() {
		moveDirection = -moveDirection;
	}

	// Events

	public void onScannedRobot(ScannedRobotEvent event) {
		lastSeen = 0;
		double tarAngle = (getHeading() + event.getBearing()) % 360;
		targetPosition = myPosition.add(Vec2D.fromAngle(tarAngle).mul(event.getDistance()));

		double tarVelocity = event.getVelocity();
		double tarHeading = event.getHeading() % 360;
		targetSpeed = Vec2D.fromAngle(tarHeading).mul(tarVelocity);
	}

	public void onHitByBullet(HitByBulletEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = Util.angleDelta(radarAngle, hitAngle);
		turnRadarRight(radarDelta * RADAR_OVERSCAN);
	}

	public void onHitRobot(HitRobotEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = Util.angleDelta(radarAngle, hitAngle);
		turnRadarRight(radarDelta * RADAR_OVERSCAN);
	}

	public void onHitWall(HitWallEvent event) {
		invertMoveDirection();
	}

	public void onWin(WinEvent event) {
		setAllColors(Color.RED);
	}

	public void onPaint(Graphics2D graphics) {
		if (isInfoNew()) {
			Util.drawPoint(graphics, Color.RED, targetPosition, 10);
			Util.drawPoint(graphics, Color.ORANGE, targetPosition.add(targetSpeed), 10);
			Util.drawPoint(graphics, Color.YELLOW, targetShootPosition, 10);
			Util.drawAngle(graphics, Color.YELLOW, myPosition, getGunHeading(), 1000);
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
			return new Vec2D(Util.constrain(x, 0, width), Util.constrain(y, 0, height));
		}

		public Vec2D move(Vec2D speed, double time) {
			return add(speed.mul(time));
		}

		public double magnitude() {
			return Math.sqrt(x * x + y * y);
		}

		public Vec2D normal() {
			double magnitude = magnitude();
			return new Vec2D(x / magnitude, y / magnitude);
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

	private static class Util {

		public static double bulletDelay(double distance, double power) {
			return distance / (20 - 3 * power);
		}

		public static double limit(double value, double limit) {
			return constrain(value, -limit, limit);
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

		public static double angleDelta(double from, double to) {
			double delta = (to - from);
			if (-180 > delta || delta > 180) {
				delta = from - to;
			}
			return delta;
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

	}

}
