package org.firstinspires.ftc.teamcode.utilities.math.linearalgebra;

import java.util.function.DoubleBinaryOperator;
import java.util.function.DoubleConsumer;
import java.util.function.Function;

public class Pose {
    private double x;
    private double y;
    private double heading;

    public Pose() {
        this(0, 0, 0);
    }

    public Pose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose(Pose other) {
        this.x = other.x;
        this.y = other.y;
        this.heading = other.heading;
    }

    public double getX() {
        return x;
    }

    public void setX(double x) {
        this.x = x;
    }

    public double getY() {
        return y;
    }

    public void setY(double y) {
        this.y = y;
    }

    public double getHeading() {
        return heading;
    }

    public void setHeading(double heading) {
        this.heading = heading;
    }

    public void add(Pose other) {
        this.setX(this.getX() + other.getX());
        this.setY(this.getY() + other.getY());
        this.setHeading(this.getHeading() + other.getHeading());
    }
    public Pose rotated(double angle) {
        double x = this.getX();
        double y = this.getY();

        this.setX(y * Math.cos(angle) - x * Math.sin(angle));
        this.setY(y * Math.sin(angle) + x * Math.cos(angle));
        this.setHeading(this.getHeading() + angle);

        return this;
    }

    public Pose times(double other) {
        this.setX(this.getX() * other);
        this.setY(this.getY() * other);

        return this;
    }

    public Pose abs() {
        this.setX(Math.abs(this.getX()));
        this.setY(Math.abs(this.getY()));
        this.setHeading(Math.abs(this.getHeading()));

        return this;
    }

    public Pose map(Function<Double, Double> func) {
        this.setX(func.apply(this.getX()));
        this.setY(func.apply(this.getY()));
        this.setHeading(func.apply(this.getHeading()));

        return this;
    }

    public boolean lessThan(Pose other) {
        return (this.getX() < other.getX()) && (this.getY() < other.getY()) && (this.getHeading() < other.getHeading());
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public double distance(Pose other) {

        return new Pose(
                x - other.x,
                y - other.y,
                heading - other.heading
        ).magnitude();

    }
}
