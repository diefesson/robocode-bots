package diefesson;

import java.awt.Color;
import java.awt.Graphics2D;
import java.io.IOException;
import java.io.Serializable;
import java.util.Set;
import java.util.HashSet;
import java.util.Iterator;
import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;

import robocode.*;
import robocode.util.Utils;

/**
 * Weirdmagedon - a robot by Diefesson de Sousa Silva
 */
public class WeirdmagedonV3 extends TeamRobot {

    // Config - target abandon
    private static final long LAST_SCAN_LIMIT = 10;

    // Config - firepower
    private static final double SHORT_POWER = 3;
    private static final double MID_POWER = 2;
    private static final double LONG_POWER = 1;
    private static final double SHORT_RANGE = 200;
    private static final double LONG_RANGE = 300;

    // Config - radar
    private static final boolean LOCK = true;
    private static final double LOCK_OVERSCAN = 30;
    private static final double REACTION_OVERSCAN = 60;

    // Config - movement
    private static final double NEAR = 150;
    private static final double FAR = 250;
    private static final double ARENA_LIMIT = 40;

    // Config - forces
    private static final double MOVE_FORCE = 0.2;
    private static final double RETHREAT_FORCE = 0.8;
    private static final double ADVANCE_FORCE = 0.6;
    private static final double WALL_FORCE = 1;
    private static final double WAVE_FORCE = 1;
    private static final double FORCE_WEIGHT = 1.0;

    // Config - movement prediction
    private static final int PREDICTION_COUNT = 35;

    // Config - dodging
    private static final int DODGING_COUNT = 9;
    private static final double DODGING_SPREAD = Rules.MAX_VELOCITY * 0.6;

    // Config - painting
    private static final boolean PAINT_RANGES = true;
    private static final boolean PAINT_TARGETING = true;
    private static final boolean PAINT_WAVES = true;
    private static final boolean PAINT_FORCES = true;

    // Info - collected
    private String targetName;
    private Vec2D targetPosition;
    private double targetSpeed;
    private double targetPriorHeading;
    private double targetHeading;
    private double targetPriorEnergy;
    private double targetEnergy;
    private long priorScanTime;
    private long scanTime = -9999;

    // Info - calculated
    private long time;
    private Vec2D myPosition;
    private Vec2D myPriorPosition;
    private Vec2D targetPriorPosition;
    private double myPriorHeading;
    private double myHeading;
    private Vec2D[] myPredictions = new Vec2D[PREDICTION_COUNT];
    private Vec2D[] targetPredictions = new Vec2D[PREDICTION_COUNT];
    private double targetAngularSpeed;
    private double targetEnergyDelta;
    private Vec2D shootPosition;
    private double targetDistance;
    private double targetAngle;
    private List<Wave> waves = new ArrayList<>();
    private List<Vec2D> forces = new ArrayList<>();
    private Vec2D[] safeCandidates = new Vec2D[DODGING_COUNT];
    private Vec2D resultingForce;
    private Vec2D movement = Vec2D.ZERO;
    
    // Info - arena
    private Rectangle arena;
    private Rectangle arenaLimits;

    // Info - messaging
    private Set<Message> messageBuffer = new HashSet<>();

    // Init and update

    @Override
    public void run() {
        init();
        while (true) {
            loop();
            execute();
        }
    }

    private void init() {
        setColors();
        setAdjusts();
        arena = new Rectangle(18, 18, getBattleFieldWidth() - 18, getBattleFieldHeight() - 18);
        Arrays.fill(safeCandidates, Vec2D.ZERO);
        arenaLimits = new Rectangle(
                arena.xS + ARENA_LIMIT,
                arena.yS + ARENA_LIMIT,
                arena.xF - ARENA_LIMIT,
                arena.yF - ARENA_LIMIT);
    }

    private void setColors() {
        setBodyColor(Color.DARK_GRAY);
        setGunColor(Color.GRAY);
        setRadarColor(Color.RED);
        setBulletColor(Color.ORANGE);
        setScanColor(Color.RED);
    }

    private void setAdjusts() {
        setAdjustRadarForGunTurn(true);
        setAdjustRadarForRobotTurn(true);
        setAdjustGunForRobotTurn(true);
    }

    private void loop() {
        dispatchMessages();
        calculate();
        update();
        predict();
    }

    private void dispatchMessages() {
        for (Message m : messageBuffer) {
            dispatchMessage(m);
        }
        messageBuffer.clear();
    }

    private void dispatchMessage(Message m) {
        if (m instanceof ScanMessage sm) {
            onScanMessage(sm);
        }
    }

    private void calculate() {
        time = getTime();
        calculateMe();
        calculateTarget();
        calculateWaves();
        calculateForces();
    }

    private void calculateMe() {
        myPriorPosition = myPosition;
        myPosition = new Vec2D(getX(), getY());
        myPriorHeading = myHeading;
        myHeading = getHeading();
        double myAngularSpeed = myHeading - myPriorHeading;
        DUtils.createPredictions(myPredictions, myPosition, getHeading(), getVelocity(), myAngularSpeed);
        Vec2D.constrainAll(myPredictions, arena);
    }

    private void calculateTarget() {
        if (isTargetUpdated()) {
            targetAngle = myPosition.lookAngle(targetPosition);
            targetDistance = myPosition.distance(targetPosition);
            targetAngularSpeed = (targetHeading - targetPriorHeading) / (scanTime - priorScanTime);
            targetEnergyDelta = (targetEnergy - targetPriorEnergy);
            DUtils.createPredictions(targetPredictions, targetPosition, targetHeading, targetSpeed, targetAngularSpeed);
            Vec2D.constrainAll(targetPredictions, arena);
            shootPosition = DUtils.calculateShootPosition(targetPredictions, myPosition, getPower());
        }
    }

    private void calculateWaves() {
        Iterator<Wave> wavesIter = waves.iterator();
        while (wavesIter.hasNext()) {
            Wave wave = wavesIter.next();
            if (wave.isPast(myPosition, time)) {
                wavesIter.remove();
            }
        }

        if (isTargetUpdated() && DUtils.isConstrained(targetEnergyDelta, -3, -1)) {
            double headsOnAngle = targetPriorPosition.lookAngle(myPriorPosition);
            Vec2D circularShootPosition = DUtils.calculateShootPosition(myPredictions, targetPriorPosition,
                    targetEnergyDelta);
            double circularAngle = targetPosition.lookAngle(circularShootPosition);
            waves.add(
                    new Wave(targetPriorPosition, headsOnAngle, circularAngle, Rules.getBulletSpeed(-targetEnergyDelta),
                            time - 2.0));
        }
    }

    private void calculateForces() {
        forces.clear();
        calculateWallForces();
        calculateWavesForces();
        calculateMoveForces();
        calculateResultingForce();
    }

    private void calculateWallForces() {
        if (myPosition.x < arenaLimits.xS) {
            forces.add(Vec2D.RIGHT.mul(WALL_FORCE));
        } else if (arenaLimits.xF < myPosition.x) {
            forces.add(Vec2D.LEFT.mul(WALL_FORCE));
        }
        if (myPosition.y < arenaLimits.yS) {
            forces.add(Vec2D.UP.mul(WALL_FORCE));
        } else if (arenaLimits.yF < myPosition.y) {
            forces.add(Vec2D.DOWN.mul(WALL_FORCE));
        }
    }

    private void calculateWavesForces() {
        Wave wave = nearestWave();
        if (wave != null) {
            double distance = wave.distance(myPosition, time);
            double remainingTime = distance / wave.speed;
            double impactTime = time + remainingTime;
            Vec2D dodgingDelta = Vec2D.fromAngle(myPosition.lookAngle(wave.origin) + 90).mul(remainingTime)
                    .mul(DODGING_SPREAD);
            Vec2D rightPosition = myPosition.add(dodgingDelta);
            Vec2D leftPosition = myPosition.add(dodgingDelta.neg());
            Vec2D.fillInterpolations(safeCandidates, rightPosition, leftPosition);
            Vec2D.toCircleSurface(safeCandidates, wave.origin, wave.radius(impactTime));
            Vec2D.constrainAll(safeCandidates, arenaLimits);
            Vec2D safest = findSafestPosition(safeCandidates, wave, impactTime);
            forces.add(myPosition.look(safest).mul(WAVE_FORCE).orZero());
        }
    }

    private void calculateMoveForces() {
        if (isTargetUpdated()) {
            if (targetDistance < NEAR) {
                forces.add(targetPosition.look(myPosition).mul(RETHREAT_FORCE));
            } else if (FAR < targetDistance) {
                forces.add(myPosition.look(targetPosition).mul(ADVANCE_FORCE));
            }
            Vec2D sideForce = Vec2D.fromAngle(targetAngle + 90);
            if (sideForce.dot(movement) < 0) {
                sideForce = sideForce.neg();
            }
            if (waves.isEmpty()) {
                forces.add(sideForce.mul(MOVE_FORCE));
            }
        }
    }

    private void calculateResultingForce() {
        if (forces.isEmpty()) {
            resultingForce = movement;
        } else {
            resultingForce = Vec2D.ZERO;
            for (Vec2D force : forces) {
                resultingForce = resultingForce.add(force);
            }
        }
        movement = movement.lerp(resultingForce, FORCE_WEIGHT);
    }

    private void update() {
        updateRadar();
        updateGun();
        updateMove();
    }

    private void updateRadar() {
        if (getRadarTurnRemaining() > 0) {
            return;
        }
        if (LOCK && isTargetUpdated()) {
            look(targetAngle, LOCK_OVERSCAN);
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

    private void predict() {
        if (isTargetUpdated()) {
            targetPriorPosition = targetPosition;
            targetPosition = targetPredictions[0];
        }
    }

    private void updateMove() {
        double movementAngle = movement.angle();
        double frontAngle = getHeading();
        double frontDelta = Utils.normalRelativeAngleDegrees(movementAngle - frontAngle);
        double backAngle = (getHeading() + 180) % 360;
        double backDelta = Utils.normalRelativeAngleDegrees(movementAngle - backAngle);
        double side = (Math.abs(backDelta) < Math.abs(frontDelta)) ? -1 : 1;
        double delta = (side < 0) ? backDelta : frontDelta;
        setTurnRight(delta);
        setAhead(20 * side);
    }

    // Methods

    private double getPower() {
        if (targetDistance < SHORT_RANGE) {
            return SHORT_POWER;
        }
        if (targetDistance > LONG_RANGE) {
            return LONG_POWER;
        }
        return MID_POWER;
    }

    public Wave nearestWave() {
        if (waves.isEmpty()) {
            return null;
        }
        Wave nearest = waves.get(0);
        double distance = nearest.distance(myPosition, time);
        for (int i = 1; i < waves.size(); i++) {
            Wave w = waves.get(i);
            double d = w.distance(myPosition, time);
            if (d < distance) {
                nearest = w;
                distance = d;
            }
        }
        return nearest;
    }

    public Vec2D findSafestPosition(Vec2D[] positions, Wave wave, double time) {
        Vec2D headsOnPosition = wave.headsOnPosition(time);
        Vec2D circularPosition = wave.circularPosition(time);
        Vec2D safest = null;
        double totalDistance = Double.NEGATIVE_INFINITY;
        for (Vec2D p : positions) {
            double headsOnDistance = headsOnPosition.distance(p);
            double circularDistance = circularPosition.distance(p);
            double td = headsOnDistance + circularDistance;
            if (headsOnDistance < 18 || circularDistance < 18) {
                td -= 100;
            }
            if (td > totalDistance) {
                safest = p;
                totalDistance = td;
            }
        }
        return safest;
    }

    public void look(double targetAngle, double overscan) {
        double radarDelta = Utils.normalRelativeAngleDegrees(targetAngle - getRadarHeading());
        setTurnRadarRight(radarDelta + Math.signum(radarDelta) * overscan);
    }

    private boolean isTargetUpdated() {
        return (time - scanTime) < LAST_SCAN_LIMIT;
    }

    private void setTarget(ScanMessage message) {
        targetName = message.name;
        priorScanTime = message.time - 1;
        scanTime = message.time;
        targetPriorHeading = targetHeading = message.heading;
        targetPriorPosition = targetPosition = message.position;
        targetSpeed = message.speed;
        targetPriorEnergy = targetEnergy = message.energy;
        calculateTarget();
    }

    private void updateTarget(ScanMessage message) {
        priorScanTime = scanTime;
        scanTime = message.time;
        targetPriorHeading = targetHeading;
        targetHeading = message.heading;
        targetPosition = message.position;
        targetSpeed = message.speed;
        targetPriorEnergy = targetEnergy;
        targetEnergy = message.energy;
        calculateTarget();
    }

    private void broadcast(Message message) {
        // Prevents delay for local info
        dispatchMessage(message);
        try {
            broadcastMessage(message);
        } catch (IOException exception) {
            out.println("ALERT: error sending message");
        }
    }

    // Message events

    private void onScanMessage(ScanMessage message) {
        if (!isTargetUpdated()) {
            setTarget(message);
        } else if (targetName.equals(message.name) && scanTime < message.time) {
            updateTarget(message);
        } else if (myPosition.distance(message.position) < targetDistance) {
            setTarget(message);
        }
    }

    // Events

    @Override
    public void onScannedRobot(ScannedRobotEvent event) {
        if (isTeammate(event.getName())) {
            return;
        }
        double tarAngle = Utils.normalAbsoluteAngleDegrees(getHeading() + event.getBearing());
        double distance = event.getDistance();
        Vec2D position = myPosition.add(Vec2D.fromAngle(tarAngle).mul(distance));
        ScanMessage message = new ScanMessage(
                event.getTime(),
                event.getName(),
                event.getHeading(),
                event.getVelocity(),
                event.getEnergy(),
                position);
        broadcast(message);
    }

    @Override
    public void onMessageReceived(MessageEvent event) {
        messageBuffer.add((Message) event.getMessage());
    }

    @Override
    public void onHitByBullet(HitByBulletEvent event) {
        double hitAngle = (getHeading() + event.getBearing()) % 360;
        look(hitAngle, REACTION_OVERSCAN);
    }

    @Override
    public void onHitRobot(HitRobotEvent event) {
        double hitAngle = (getHeading() + event.getBearing()) % 360;
        look(hitAngle, REACTION_OVERSCAN);
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
        if (PAINT_RANGES) {
            DUtils.drawCircle(graphics, Color.GRAY, myPosition, NEAR);
            DUtils.drawCircle(graphics, Color.GRAY, myPosition, FAR);
            DUtils.drawCircle(graphics, Color.LIGHT_GRAY, myPosition, SHORT_RANGE);
            DUtils.drawCircle(graphics, Color.LIGHT_GRAY, myPosition, LONG_RANGE);
        }
        if (isTargetUpdated() && PAINT_TARGETING) {
            DUtils.drawPoint(graphics, Color.RED, targetPosition, 10);
            for (Vec2D p : targetPredictions) {
                DUtils.drawPoint(graphics, Color.YELLOW, p, 5);
            }
            DUtils.drawPoint(graphics, Color.RED, shootPosition, 10);
            DUtils.drawAngle(graphics, Color.YELLOW, myPosition, getGunHeading(), 1000);

            DUtils.drawCircle(graphics, Color.YELLOW, myPosition, targetDistance);
        }
        if (PAINT_WAVES && !waves.isEmpty()) {
            for (Vec2D candidate : safeCandidates) {
                DUtils.drawPoint(graphics, Color.WHITE, candidate, 5);
            }
            for (Wave wave : waves) {
                DUtils.drawCircle(graphics, Color.BLUE, wave.origin, wave.radius(time));
                DUtils.drawPoint(graphics, Color.GREEN, wave.headsOnPosition(time), 10);
                DUtils.drawPoint(graphics, Color.RED, wave.circularPosition(time), 10);
            }
        }
        if (PAINT_FORCES) {
            DUtils.drawRectangle(graphics, Color.blue, arenaLimits);
            for (Vec2D force : forces) {
                DUtils.drawAngle(graphics, Color.GRAY, myPosition, force.angle(), force.magnitude() * 200);
            }
            DUtils.drawAngle(graphics, Color.WHITE, myPosition, resultingForce.angle(),
                    resultingForce.magnitude() * 200);
            DUtils.drawAngle(graphics, Color.BLUE, myPosition, movement.angle(), movement.magnitude() * 200);
        }
    }

    public static class Wave {

        public final Vec2D origin;
        public final double headsOnAngle;
        public final double circularAngle;
        public final double speed;
        public final double startTime;

        public Wave(Vec2D origin, double headsOnAngle, double circularAngle, double speed, double startTime) {
            this.origin = origin;
            this.headsOnAngle = headsOnAngle;
            this.circularAngle = circularAngle;
            this.speed = speed;
            this.startTime = startTime;
        }

        public double deltaTime(double time) {
            return time - startTime;
        }

        public double radius(double time) {
            return speed * deltaTime(time);
        }

        public double distance(Vec2D position, double time) {
            return origin.distance(position) - radius(time);
        }

        public boolean isPast(Vec2D position, double time) {
            return origin.distance(position) < radius(time);
        }

        public Vec2D headsOnPosition(double time) {
            return origin.add(Vec2D.fromAngle(headsOnAngle).mul(radius(time)));
        }

        public Vec2D circularPosition(double time) {
            return origin.add(Vec2D.fromAngle(circularAngle).mul(radius(time)));
        }

    }

    public static class Message implements Serializable {

        public final long time;

        protected Message(long time) {
            this.time = time;
        }

    }

    public static class ScanMessage extends Message {

        public final String name;
        public final double heading;
        public final double speed;
        public final double energy;
        public final Vec2D position;

        public ScanMessage(long time, String name, double heading, double speed, double energy, Vec2D position) {
            super(time);
            this.name = name;
            this.heading = heading;
            this.speed = speed;
            this.energy = energy;
            this.position = position;
        }

        @Override
        public int hashCode() {
            return name.hashCode();
        }

        @Override
        public boolean equals(Object other) {
            if (other instanceof ScanMessage otherSM) {
                return this.name.equals(otherSM.name);
            }
            return false;
        }

    }

    public static class Rectangle {

        public final double xS;
        public final double yS;
        public final double xF;
        public final double yF;
        public final double width;
        public final double height;

        public Rectangle(double xS, double yS, double xF, double yF) {
            this.xS = xS;
            this.yS = yS;
            this.xF = xF;
            this.yF = yF;
            this.width = xF - xS;
            this.height = yF - yS;
        }

    }

    public static class Vec2D implements Serializable {

        public static final Vec2D ZERO = new Vec2D(0, 0);
        public static final Vec2D UP = new Vec2D(0, 1);
        public static final Vec2D DOWN = new Vec2D(0, -1);
        public static final Vec2D LEFT = new Vec2D(-1, 0);
        public static final Vec2D RIGHT = new Vec2D(1, 0);

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

        public Vec2D orZero() {
            if (!Double.isFinite(x) || !Double.isFinite(y)) {
                return ZERO;
            } else {
                return this;
            }
        }

        public double magnitude() {
            return Math.hypot(x, y);
        }

        public Vec2D normal() {
            double magnitude = magnitude();
            return new Vec2D(x / magnitude, y / magnitude);
        }

        public Vec2D neg() {
            return new Vec2D(-x, -y);
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

        public double dot(Vec2D other) {
            return x * other.x + y * other.y;
        }

        public Vec2D constrain(Rectangle rectangle) {
            return constrain(rectangle.xS, rectangle.yS, rectangle.xF, rectangle.yF);
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

        public Vec2D lerp(Vec2D to, double weight) {
            return new Vec2D(DUtils.lerp(x, to.x, weight), DUtils.lerp(y, to.y, weight));
        }

        @Override
        public String toString() {
            return "x: " + this.x + ", y: " + this.y;
        }

        public static void constrainAll(Vec2D[] positions, Rectangle rectangle) {
            for (int i = 0; i < positions.length; i++) {
                positions[i] = positions[i].constrain(rectangle);
            }
        }

        public static void fillInterpolations(Vec2D[] interpolations, Vec2D from, Vec2D to) {
            int count = interpolations.length - 1;
            for (int i = 0; i <= count; i++) {
                interpolations[i] = from.lerp(to, (double) i / count);
            }
        }

        public static void toCircleSurface(Vec2D[] points, Vec2D origin, double radius) {
            for (int i = 0; i < points.length; i++) {
                double angle = origin.lookAngle(points[i]);
                points[i] = origin.add(Vec2D.fromAngle(angle).mul(radius));
            }
        }

    }

    public static class DUtils {

        private DUtils() {
        }

        public static double bulletDelay(double distance, double power) {
            return distance / (20 - 3 * power);
        }

        public static boolean isConstrained(double value, double low, double high) {
            return low <= value && value <= high;
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

        public static double lerp(double from, double to, double weight) {
            return from * (1 - weight) + to * weight;
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

        public static boolean iminentWallImpact(Vec2D position, double heading, double direction, double limit,
                double width, double height) {
            Vec2D speed2D = Vec2D.fromAngle(heading).mul(direction);
            return (speed2D.x < 0 && position.x < limit ||
                    speed2D.y < 0 && position.y < limit ||
                    0 < speed2D.x && width - limit < position.x ||
                    0 < speed2D.y && height - limit < position.y);
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

        public static void drawRectangle(Graphics2D graphics, Color color, Rectangle rectangle) {
            graphics.setColor(color);
            graphics.drawRect((int) rectangle.xS, (int) rectangle.yS, (int) rectangle.width, (int) rectangle.height);
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
