package frc.robot.lib.PathFinder;

import edu.wpi.first.math.geometry.Translation2d;

public class AABB {
    public Translation2d topLeft;
    public Translation2d bottomRight;
    public Translation2d bottomLeft;
    public Translation2d topRight;

    public AABB(Translation2d topLeft, Translation2d topRight) {
        this.topLeft = topLeft;
        this.topRight = topRight;
        this.bottomLeft = new Translation2d(topLeft.getX(), topRight.getY());
        this.bottomRight = new Translation2d(topRight.getX(), topLeft.getY());
    }

    public boolean check_line(Translation2d line_start, Translation2d line_end) {
        boolean retVal;
        retVal = Intersects(line_start, line_end, topLeft, bottomLeft);
        retVal = retVal || Intersects(line_start, line_end, bottomLeft, bottomRight);
        retVal = retVal || Intersects(line_start, line_end, bottomRight, topRight);
        retVal = retVal || Intersects(line_start, line_end, topRight, topLeft);
        return retVal;
    }

    // a1 is line1 start, a2 is line1 end, b1 is line2 start, b2 is line2 end
    private boolean Intersects(Translation2d a1, Translation2d a2, Translation2d b1, Translation2d b2) {
        Translation2d b = a2.minus(a1);
        Translation2d d = b2.minus(b1);
        double bDotDPerp = b.getX() * d.getY() - b.getY() * d.getX();

        // if b dot d == 0, it means the lines are parallel so have infinite
        // intersection points
        if (bDotDPerp == 0)
            return false;

        Translation2d c = b1.minus(a1);
        double t = (c.getX() * d.getY() - c.getY() * d.getX()) / bDotDPerp;
        if (t < 0 || t > 1)
            return false;

        double u = (c.getX() * b.getY() - c.getY() * b.getX()) / bDotDPerp;
        if (u < 0 || u > 1)
            return false;

        return true;
    }
}
