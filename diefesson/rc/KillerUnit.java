package diefesson.rc;

import robocode.*;
import java.awt.Color;
import java.awt.Graphics2D;

/**
 * KillerUnit - a robot by Diefesson de Sousa Silva
 */
public class KillerUnit extends Robot {

	private final long LAST_SEEN_LIMIT = 10;
	private final double RADAR_LIMIT = 30;
	private final double GUN_LIMIT = 20;
	private final double MOVE = 25;
	private final double POWER = 1;
	private final double EXPECTED_DELAY = 8; // TODO replace by smarter strategy
	private final double HIT_OVERSCAN = 1.1;

	private Vec2D myPos = null;
	private double moveDirection = 1;
	private Vec2D targetPosition = null;
	private Vec2D targetSpeed = null;
	private long lastSeen = 9999;

	// Init and update

	public void run() {
		init();
		long lastTime = getTime();
		long time;
		long delta;
		while (true) {
			time = getTime();
			delta = time - lastTime;
			lastTime = time;
			out.println("time: " + time + ", delta: " + delta);
			update(delta);
		}
	}

	private void init() {
		setColors(Color.BLACK, Color.RED, Color.YELLOW);
		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);
	}

	private void update(long delta) {
		collect(delta);
		predict(delta);
		updateRadar(delta);
		updateGun(delta);
		updateMove(delta);
	}

	private void collect(long delta) {
		lastSeen += delta;
		myPos = new Vec2D(getX(), getY());
	}

	private void predict(long delta) {
		if (isInfoNew()) {
			targetPosition = targetPosition.move(targetSpeed, delta);
		}
	}

	private void updateRadar(long delta) {
		if (isInfoNew()) {
			double radarAngle = getRadarHeading();
			double tarAngle = myPos.look(targetPosition);
			double radarDelta = Util.angleDelta(radarAngle, tarAngle);
			radarDelta = Util.limit(radarDelta, RADAR_LIMIT);
			turnRadarRight(radarDelta);
		} else {
			turnRadarRight(RADAR_LIMIT);
		}
	}

	private void updateGun(long delta) {
		if (isInfoNew()) {
			double gunAngle = getGunHeading();
			double targetShootAngle = myPos.look(targetPosition.move(targetSpeed, EXPECTED_DELAY));
			double gunDelta = Util.angleDelta(gunAngle, targetShootAngle);
			gunDelta = Util.limit(gunDelta, GUN_LIMIT);
			turnGunRight(gunDelta);
			if (getGunHeat() == 0) {
				fire(POWER);
			}
		}
	}

	private void updateMove(long delta) {
		// TODO: need RateControlRobot
	}

	// Methods

	private boolean isInfoNew() {
		return lastSeen < LAST_SEEN_LIMIT;
	}

	private void invertMoveDirection() {
		moveDirection = -moveDirection;
	}

	private void dodge() {
		ahead(MOVE * moveDirection);
	}

	// Events

	public void onScannedRobot(ScannedRobotEvent event) {
		lastSeen = 0;
		double tarAngle = (getHeading() + event.getBearing()) % 360;
		targetPosition = myPos.add(Vec2D.fromAngle(tarAngle).mul(event.getDistance()));

		double tarVelocity = event.getVelocity();
		double tarHeading = event.getHeading() % 360;
		targetSpeed = Vec2D.fromAngle(tarHeading).mul(tarVelocity);
	}

	public void onBulletHitBullet(BulletHitBulletEvent event) {
		dodge();
	}

	public void onHitByBullet(HitByBulletEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = Util.angleDelta(radarAngle, hitAngle);
		turnRadarRight(radarDelta * HIT_OVERSCAN);
		dodge();
	}

	public void onHitWall(HitWallEvent e) {
		invertMoveDirection();
		dodge();
	}

	public void onPaint(Graphics2D graphics) {
		if (isInfoNew()) {
			Util.drawPoint(graphics, Color.RED, targetPosition, 10);
			Util.drawPoint(graphics, Color.YELLOW, targetPosition.add(targetSpeed), 10);
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

		public static double limit(double value, double limit) {
			if (value < (-limit)) {
				return -limit;
			}
			if (value > limit) {
				return limit;
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

		public static void drawAngle(Graphics2D graphics, Color color, Vec2D from, Vec2D to) {
			graphics.setColor(color);
			graphics.drawLine((int) from.x, (int) from.y, (int) to.x, (int) to.y);
		}

	}

}
