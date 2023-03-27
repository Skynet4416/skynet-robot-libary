package frc.robot.lib.PathFinder;

import java.util.ArrayList;
import java.util.Arrays;

public class Graph {
    ArrayList<Node> Nodes;
    ArrayList<Node> OpenNodes = new ArrayList<Node>();
    ArrayList<Node> ClosedNodes = new ArrayList<Node>();

    public Graph(ArrayList<Node> Nodes) {
        this.Nodes = Nodes;
    }

    private double Distance2D(Node A, Node B) {
        return Math.sqrt(Math.pow(A.x - B.x, 2) + Math.pow(A.y - B.y, 2));
    }

    public ArrayList<Node> aStar(Node Start, Node End) {
        ArrayList<Node> found_path = new ArrayList<Node>();
        ArrayList<Node> reverse_path = new ArrayList<Node>();
        reverse_path.add(End);

        OpenNodes.clear();
        ClosedNodes.clear();
        Node current_node;

        // CALCULATING HEURISTIC DISTANCES
        for (int i = 0; i < Nodes.size(); i++) {
            Nodes.get(i).h = this.Distance2D(Nodes.get(i), End);
        }

        OpenNodes.add(Start);
        Start.g = 0;
        Start.f = Start.g + Start.h;

        while (OpenNodes.size() > 0) {
            current_node = Start;

            // PICK CURRENT NODE
            current_node = OpenNodes.get(0);
            double smallest_f = current_node.f;
            for (int i = 0; i < OpenNodes.size(); i++) {
                if (OpenNodes.get(i).f < smallest_f) {
                    current_node = OpenNodes.get(i);
                    smallest_f = current_node.f;
                }
            }

            // UPDATE NEIGHBORS
            for (int i = 0; i < current_node.Neighbors.size(); i++) {
                double alt_g = current_node.g
                        + Distance2D(current_node, current_node.Neighbors.get(i));
                double alt_f = current_node.Neighbors.get(i).g + current_node.Neighbors.get(i).h;

                if (alt_f < current_node.Neighbors.get(i).f || current_node.Neighbors.get(i).f == -1) {
                    current_node.Neighbors.get(i).g = alt_g;
                    current_node.Neighbors.get(i).f = alt_f;
                    current_node.Neighbors.get(i).prevNode = current_node;
                }
                if (!ClosedNodes.contains(current_node.Neighbors.get(i)))
                    OpenNodes.add(current_node.Neighbors.get(i));
            }

            ClosedNodes.add(current_node);
            OpenNodes.remove(current_node);

            // TRACE BACK PATH
            if (OpenNodes.contains(End)) {
                System.out.println("PATH FOUND!");
                while (!reverse_path.contains(Start)) {
                    reverse_path.add(reverse_path.get(reverse_path.size() - 1).prevNode);
                }
                for (int i = 0; i < reverse_path.size(); i++) {
                    found_path.add(reverse_path.get(reverse_path.size() - i - 1));
                }
                return found_path;
            }
        }

        System.out.println("NO PATH FOUND!");
        return found_path;
    }

    public static void main(String[] args) {
        Node p1 = new Node(2, 0.5);
        Node p2 = new Node(2, 2.5);
        Node p3 = new Node(2, 5);
        Node p4 = new Node(5, 5);
        Node p5 = new Node(7, 2);
        Node p6 = new Node(4, 0.5);
        Node p7 = new Node(8, 3);

        p1.Neighbors = new ArrayList<Node>(Arrays.asList(p2, p6));
        p2.Neighbors = new ArrayList<Node>(Arrays.asList(p1, p3));
        p3.Neighbors = new ArrayList<Node>(Arrays.asList(p2, p4));
        p4.Neighbors = new ArrayList<Node>(Arrays.asList(p3, p7, p5));
        p5.Neighbors = new ArrayList<Node>(Arrays.asList(p4, p7, p6));
        p6.Neighbors = new ArrayList<Node>(Arrays.asList(p2, p5));
        p7.Neighbors = new ArrayList<Node>(Arrays.asList(p4, p5));

        Graph graph = new Graph(new ArrayList<Node>(Arrays.asList(p1, p2, p3, p4, p5, p6, p7)));
        var path = graph.aStar(p3, p6);
        System.out.println(path);
    }
}