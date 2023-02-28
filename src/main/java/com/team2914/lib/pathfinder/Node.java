package com.team2914.lib.pathfinder;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;

public class Node {
    private boolean isObstacle;
    private Translation2d pos;
    private int[] gridPos;
    private double cost;
    private double gScore;
    private Node parent;
    private List<Node> neighbors;

    public Node(boolean isObstacle, Translation2d pos, int[] gridPos) {
        this.isObstacle = isObstacle;
        this.pos = pos;
        this.gridPos = gridPos;
        this.cost = Double.MAX_VALUE;
        this.gScore = Double.MAX_VALUE;
        this.parent = null;
        this.neighbors = new ArrayList<>();
    }

    public double distance(Node n) {
        return pos.getDistance(n.getPos());
    }

    public double distance(Translation2d _pos) {
        return pos.getDistance(_pos);
    }

    public Translation2d getPos() {
        return this.pos;
    }

    public List<Node> getNeighbors() {
        return this.neighbors;
    }

    public boolean getIsObstacle() {
        return this.isObstacle;
    }

    public int[] getGridPos() {
        return this.gridPos;
    }

    public double getCost() {
        return this.cost;
    }

    public double getGScore() {
        return this.gScore;
    }

    public Node getParent() {
        return this.parent;
    }

    public void setGScore(double g) {
        this.gScore = g;
    }

    public void setCost(double cost) {
        this.cost = cost;
    }

    public void setParent(Node parent) {
        this.parent = parent;
    }

    public void setObstacle(boolean obstacle) {
        this.isObstacle = obstacle;
    }
}
