package frc.robot.lib.PathFinder;

import java.util.ArrayList;

public class Node {
    public double x = 0;
    public double y = 0;
    public double g = -1;
    public double h = -1;
    public double f = -1;
    public Node prevNode = null;
    ArrayList<Node> Neighbors;

    public Node(Node node) {
        this.x = node.x;
        this.y = node.y;
        this.g = node.g;
        this.h = node.h;
        f = node.f;
        prevNode = node.prevNode;
    }

    public Node(double x, double y, ArrayList<Node> Neighbors) {
        this.y = y;
        this.x = x;
        this.Neighbors = Neighbors;
    }

    public Node(double x, double y) {
        this.y = y;
        this.x = x;
    }

    @Override
    public String toString() {
        return "(" + this.x + ", " + this.y + ")";
    }

}