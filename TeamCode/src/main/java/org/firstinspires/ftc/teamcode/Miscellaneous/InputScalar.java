package org.firstinspires.ftc.teamcode.Miscellaneous;

public class InputScalar {

    public enum ScaleType{
        NONE, CUBE, CUBEOVERABS, FIFTH
    }

    public static ScaleType scaleType = ScaleType.NONE;
    public static double multiplier = 1;

    public static double scale(double input){
        if(input == 0) return 0;
        if(scaleType == ScaleType.CUBE) input = Math.pow(input, 3);
        if(scaleType == ScaleType.CUBEOVERABS) input = Math.pow(input, 3) / Math.abs(input);
        if(scaleType == ScaleType.FIFTH) input = Math.pow(input, 5);
        input *= multiplier;
        return input;
    }

    public static void setScaleType(ScaleType type){
        scaleType = type;
    }

    public static void setMultiplier(double m){
        multiplier = m;
    }


}
