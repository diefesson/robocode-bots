package diefesson;

import java.awt.Color;
import java.awt.Graphics2D;

import robocode.*;
import robocode.util.Utils;

/**
 * Weirdmagedon - a robot by Diefesson de Sousa Silva
 */
public class WeirdmagedonV1 extends AdvancedRobot {

	// Config - target abandon
	private static final long LAST_SCAN_LIMIT = 10;

	// Config - firepower
	private static final double SHORT_POWER = 3;
	private static final double POWER = 2;
	private static final double LONG_POWER = 1;
	private static final double SHORT_RANGE = 200;
	private static final double LONG_RANGE = 300;

	// Config - radar
	private static final double OVERSCAN = 30;
	private static final boolean LOCK = true;

	// Config - movement
	private static final double CORRECTION_RATE = 45;
	private static final double NEAR = 100;
	private static final double FAR = 250;

	// Config - movement prediction
	private static final int PREDICTION_COUNT = 35;

	// State
	private double moveDirection = 1;

	private String targetName;
	private Vec2D targetPosition;
	private double targetSpeed;
	private double targetAngularSpeed;
	private double targetPriorHeading;
	private double targetHeading;
	private long priorScanTime;
	private long scanTime = -9999;
	private Vec2D[] predictions = new Vec2D[PREDICTION_COUNT];
	private Vec2D shootPosition;
	private double targetDistance;
	private double targetAngle;
	private Vec2D myPosition;

	// Init and update

	@Override
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
			targetAngle = myPosition.lookAngle(targetPosition);
			targetAngularSpeed = (targetHeading - targetPriorHeading) / (scanTime - priorScanTime);
			targetDistance = myPosition.distance(targetPosition);
			DUtils.createPredictions(predictions, targetPosition, targetHeading, targetSpeed, targetAngularSpeed);
			for (int i = 0; i < predictions.length; i++) {
				predictions[i] = predictions[i].constrain(0, 0, getBattleFieldWidth(), getBattleFieldHeight());
			}
			shootPosition = DUtils.calculateShootPosition(predictions, myPosition, getPower());
		}
	}

	private void predict() {
		if (isTargetUpdated()) {
			targetPosition = predictions[0];
		}
	}

	private void updateRadar() {
		if (getRadarTurnRemaining() > 0) {
			return;
		}
		if (LOCK && isTargetUpdated()) {
			double radarAngle = getRadarHeading();
			double radarDelta = DUtils.calculateRadar(radarAngle, targetAngle, OVERSCAN);
			setTurnRadarRight(radarDelta);
		} else {
			setTurnRadarRight(360);
		}
	}

	private void updateGun() {
		if (isTargetUpdated()) {
			double gunAngle = getGunHeading();
			double shootAngle = myPosition.lookAngle(shootPosition);
			double gunDelta = Utils.normalRelativeAngleDegrees(shootAngle - gunAngle);
			setTurnGunRight(gunDelta);
			if (getGunHeat() == 0) {
				setFire(getPower());
			}
		}
	}

	private void updateMove() {
		if (isTargetUpdated()) {
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

	private double getPower() {
		if (targetDistance < SHORT_RANGE) {
			return SHORT_POWER;
		}
		if (targetDistance > LONG_RANGE) {
			return LONG_POWER;
		}
		return POWER;
	}

	private boolean isTargetUpdated() {
		return (getTime() - scanTime) < LAST_SCAN_LIMIT;
	}

	private void invertMoveDirection() {
		moveDirection = -moveDirection;
	}

	private void setTarget(ScannedRobotEvent event) {
		targetName = event.getName();

		priorScanTime = event.getTime() - 1;
		scanTime = event.getTime();

		targetPriorHeading = targetHeading = event.getHeading();

		double tarAngle = Utils.normalAbsoluteAngleDegrees(getHeading() + event.getBearing());
		double distance = event.getDistance();
		targetPosition = myPosition.add(Vec2D.fromAngle(tarAngle).mul(distance));

		targetSpeed = event.getVelocity();
		calculate();
	}

	private void updateTarget(ScannedRobotEvent event) {
		priorScanTime = scanTime;
		scanTime = event.getTime();

		targetPriorHeading = targetHeading;
		targetHeading = event.getHeading();

		double tarAngle = Utils.normalAbsoluteAngleDegrees(getHeading() + event.getBearing());
		double distance = event.getDistance();
		targetPosition = myPosition.add(Vec2D.fromAngle(tarAngle).mul(distance));

		targetSpeed = event.getVelocity();
		calculate();
	}

	// Events

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		if (!isTargetUpdated()) {
			setTarget(event);
		} else if (targetName.equals(event.getName())) {
			updateTarget(event);
		} else if (event.getDistance() < targetDistance) {
			setTarget(event);
		}
	}

	@Override
	public void onHitByBullet(HitByBulletEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = DUtils.calculateRadar(radarAngle, hitAngle, OVERSCAN);
		setTurnRadarRight(radarDelta);
		invertMoveDirection();
	}

	@Override
	public void onHitRobot(HitRobotEvent event) {
		double hitAngle = (getHeading() + event.getBearing()) % 360;
		double radarAngle = getRadarHeading();
		double radarDelta = DUtils.calculateRadar(radarAngle, hitAngle, OVERSCAN);
		setTurnRadarRight(radarDelta);
		invertMoveDirection();
	}

	@Override
	public void onHitWall(HitWallEvent event) {
		invertMoveDirection();
	}

	@Override
	public void onWin(WinEvent event) {
		setAllColors(Color.RED);
	}

	@Override
	public void onSkippedTurn(SkippedTurnEvent event) {
		out.println("ALERT: skipped " + event.getSkippedTurn() + "turns");
	}

	@Override
	public void onPaint(Graphics2D graphics) {
		if (isTargetUpdated()) {
			DUtils.drawPoint(graphics, Color.RED, targetPosition, 10);
			for (Vec2D p : predictions) {
				DUtils.drawPoint(graphics, Color.YELLOW, p, 5);
			}
			DUtils.drawPoint(graphics, Color.RED, shootPosition, 10);
			DUtils.drawAngle(graphics, Color.YELLOW, myPosition, getGunHeading(), 1000);
			DUtils.drawCircle(graphics, Color.GRAY, myPosition, NEAR);
			DUtils.drawCircle(graphics, Color.GRAY, myPosition, FAR);
			DUtils.drawCircle(graphics, Color.LIGHT_GRAY, myPosition, SHORT_RANGE);
			DUtils.drawCircle(graphics, Color.LIGHT_GRAY, myPosition, LONG_RANGE);
			DUtils.drawCircle(graphics, Color.YELLOW, myPosition, targetDistance);
		}
	}

	public static class Vec2D {

		public final double x;
		public final double y;

		public Vec2D(double x, double y) {
			this.x = x;
			this.y = y;
		}

		public static Vec2D fromAngle(double angle) {
			angle = Math.toRadians(angle);
			return new Vec2D(Math.sin(angle), Math.cos(angle));
		}

		public double magnitude() {
			return Math.hypot(x, y);
		}

		public Vec2D normal() {
			double magnitude = magnitude();
			return new Vec2D(x / magnitude, y / magnitude);
		}

		public double angle() {
			return (360 + Math.toDegrees(Math.atan2(x, y))) % 360;
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
			return other.sub(this).magnitude();
		}

		public Vec2D constrain(double minX, double minY, double maxX, double maxY) {
			return new Vec2D(DUtils.constrain(x, minX, maxX), DUtils.constrain(y, minY, maxY));
		}

		public Vec2D move(Vec2D speed, double time) {
			return add(speed.mul(time));
		}

		public Vec2D look(Vec2D other) {
			return other.sub(this).normal();
		}

		public double lookAngle(Vec2D other) {
			return other.sub(this).angle();
		}

		@Override
		public String toString() {
			return "x: " + this.x + ", y: " + this.y;
		}

	}

	public static class DUtils {

		private DUtils() {
		}

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

		public static void createPredictions(
				Vec2D[] predictions,
				Vec2D startPosition,
				double startAngle,
				double speed,
				double angularSpeed) {

			Vec2D position = startPosition;
			double angle = startAngle;
			for (int i = 0; i < predictions.length; i++) {
				Vec2D delta = Vec2D.fromAngle(angle).mul(speed);
				position = position.add(delta);
				predictions[i] = position;
				angle += angularSpeed;
			}
		}

		public static Vec2D calculateShootPosition(Vec2D[] predictions, Vec2D origin, double power) {
			double speed = Rules.getBulletSpeed(power);
			Vec2D bestPosition = null;
			double bestDistance = Double.MAX_VALUE;
			for (int i = 0; i < predictions.length; i++) {
				Vec2D prediction = predictions[i];
				Vec2D bulletPosition = origin.add(origin.look(prediction).mul(speed).mul((double) i + 1));
				double distance = bulletPosition.distance(prediction);
				if (distance < bestDistance) {
					bestPosition = prediction;
					bestDistance = distance;
				}
			}
			return bestPosition;
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
