package com.team2914.lib.pathfinder;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

import com.team2914.robot.utils.MiscUtil;

public class Pathfinder {
    private static Pathfinder instance = null;
    private final double MAP_WIDTH = 16.5;
    private final double MAP_HEIGHT = 8;
    private final double MAP_WIDTH_DOUBLE = MAP_WIDTH * 2;
    private final double MAP_HEIGHT_DOUBLE = MAP_HEIGHT * 2;
    private List<Node> map;
    private List<Node> open;
    private List<Node> closed;
    
    private Pathfinder() {
        map = new ArrayList<>();
        open = new ArrayList<>();
        closed = new ArrayList<>();

        for (int j = 0; j < MAP_HEIGHT_DOUBLE; j++) {
            for (int i = 0; i < MAP_WIDTH_DOUBLE; i++) {
                map.add(new Node(false, new Translation2d(i / 2.0, j / 2.0), new int[] { i, j }));
            }
        }

        for (int j = 0; j < MAP_HEIGHT_DOUBLE; j++) {
            int y = (int)(16 - MAP_HEIGHT_DOUBLE + j);
            for (int i = 0; i < MAP_WIDTH_DOUBLE; i++) {
                int x = i;
                Node node = map.get(y * (int)MAP_WIDTH_DOUBLE + x);

                if (y < (MAP_HEIGHT - 0.5) * 2) {
                    node.getNeighbors().add(map.get((y + 1) * (int)MAP_WIDTH_DOUBLE + x));
                }
                if (y > 0) {
                    node.getNeighbors().add(map.get((y - 1) * (int)MAP_WIDTH_DOUBLE + x));
                }
                if (x > 0) {
                    node.getNeighbors().add(map.get(y * (int)MAP_WIDTH_DOUBLE + (x - 1)));
                }
                if (x < (MAP_HEIGHT - 0.5) * 2) {
                    node.getNeighbors().add(map.get(y * (int)MAP_WIDTH_DOUBLE + (x + 1)));
                }
            }
        }
    }

    public static Pathfinder getInstance() {
        if (instance == null) {
            instance = new Pathfinder();
        }

        return instance;
    }

    private boolean lineOfSight(Node a, Node b) {
        List<Integer[]> line = MiscUtil.bresenham(a.getGridPos()[0], a.getGridPos()[1], b.getGridPos()[0], b.getGridPos()[1]);
        for (Integer[] p : line) {
            if (map.get(p[1] * (int)MAP_WIDTH_DOUBLE + p[0]).getIsObstacle()) {
                return false;
            }
        }

        return true;
    }

    private Node findNearestNodeToPos(Translation2d xy) {
        Node nd = null;
        double min = Double.MAX_VALUE;
        for (Node node : map) {
            double dist = node.distance(xy);
            if (dist > min) continue;
            min = dist;
            nd = node;
        }

        if (nd.getIsObstacle() == false) return nd;

        int mx = (int)(((xy.getX() / MAP_WIDTH) * MAP_WIDTH_DOUBLE) + 0.5);
        int my = (int)(((xy.getY() / MAP_HEIGHT) * MAP_HEIGHT_DOUBLE) + 0.5);

        return map.get(my * (int)MAP_WIDTH_DOUBLE + mx);
    }

    private double heuristic(Node a, Node b) {
        return a.distance(b);
    }

    private void sortOpenList() {
        for (int i = 0; i < open.size(); i++) {
            int min = i;
            for (int j = i + 1; j < open.size(); j++) {
                if (open.get(j).getCost() < open.get(min).getCost()) {
                    min = j;
                }
            }
            Node temp = open.get(i);
            open.set(i, open.get(min));
            open.set(min, temp);
        }
    }

    private List<Node> reconstructPath(Node s, Node start) {
        List<Node> out = new ArrayList<>();
        if (s != null) {
            Node p = s;
            while (p.getParent() != null) {
                out.add(p);
                if (p.equals(start)) break;
                p = p.getParent();
            }
        }

        return out;
    }

    private void computeCost(Node s, Node neighbor) {
        if (lineOfSight(s.getParent(), neighbor)) {
            Node parent = s.getParent();
            double possiblyLowerGoal = parent.getGScore() + parent.distance(neighbor);

            if (possiblyLowerGoal > neighbor.getGScore()) return;

            neighbor.setParent(parent);
            neighbor.setGScore(possiblyLowerGoal);
        } else {
            double possiblyLowerGoal = s.getGScore() + s.distance(neighbor);

            if (possiblyLowerGoal > neighbor.getGScore()) return;

            neighbor.setParent(s);
            neighbor.setGScore(possiblyLowerGoal);
        }
    }

    private void updateVertex(Node s, Node neighbor, Node end) {
        double gOld = neighbor.getGScore();
        computeCost(s, neighbor);
        if (neighbor.getGScore() > gOld) return;

        neighbor.setCost(neighbor.getGScore() + heuristic(neighbor, end));

        if (open.contains(neighbor) && neighbor.getIsObstacle()) return;
            
        open.add(neighbor);
        
    }

    private List<Node> theta(Translation2d start, Translation2d end) {
        Node startNode = findNearestNodeToPos(start);
        Node endNode = findNearestNodeToPos(end);

        startNode.setGScore(0);
        startNode.setParent(startNode);
        startNode.setCost(startNode.getGScore() + heuristic(startNode, endNode));

        open.clear();
        closed.clear();
        open.add(startNode);

        while (open.size() > 0) {
            sortOpenList();
            Node s = open.remove(0);

            if (s.equals(endNode)) return reconstructPath(s, startNode);
            
            closed.add(s);

            for (Node neighbor : s.getNeighbors()) {
                if (closed.contains(neighbor)) continue;

                if (!open.contains(neighbor)) {
                    neighbor.setGScore(Double.MAX_VALUE);
                    neighbor.setParent(null);
                }
                
                updateVertex(s, neighbor, endNode);
            }
        }

        return null;
    }

    public Command generatePathCommand(Translation2d start, Translation2d end) {
        return null;
    }
}
