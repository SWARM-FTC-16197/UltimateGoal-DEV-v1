package org.firstinspires.ftc.teamcode.Miscellaneous;

public class Normalizer {
    public static double normalize(double angle) {
        double TAU = 360;

        angle = angle % TAU;

        angle = (angle + TAU) % TAU;

        if (angle > 180) {
            angle -= TAU;
        }

        return angle;
    }
}
