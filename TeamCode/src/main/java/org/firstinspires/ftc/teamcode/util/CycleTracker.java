package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.util.Timing;

import java.io.File;
import java.util.HashMap;
import com.arcrobotics.ftclib.util.Timing.Timer;
import java.util.concurrent.TimeUnit;


public class CycleTracker {
    protected static StatCalculator stat = new StatCalculator();
    protected static Timing.Timer timer = new Timer(120, TimeUnit.SECONDS);
//    Timing.Timer
    // private static int high = 0 , low = 0;
    private File file;
    private static HashMap<String, Double> data;
    private static double cycle =0,
        mean = 0,
        fastest = 0,
        slowest = 0,
        high = 0,
        low = 0;

    public CycleTracker(){
        file = new File("/home/lvuser/Logs");//Add in data like time of match, what type it is from DRIVER STATION
        timer.start();
    }
    
    public static void trackCycle(int num){
        double cycleTime = timer.elapsedTime();
        stat.addNumber(cycleTime);
        cycle = stat.getSizeDouble();
        mean = stat.getMean();
        slowest = stat.getLowestValue();
        fastest = stat.getHighestValue();
        // switch (getPosition()){
        //     case HIGH:
        //         high.set(high.get()+1);
        //     case LOW:
        //         low.set(low.get()+1);
        switch (num){
            case 1:
                high = (high+1);
            case 2:
                low = (low+1);
        }    // }
    }

    public static void printOut(String string, double num){
        data.put(string, num);
    }
    public static void printAllData(){
//        printOut(DriverStation.getMatchType().toString(), DriverStation.getMatchNumber());

        for (int i = 0; i < stat.getSizeInt(); i++) {
            printOut("Cycle "+ i +":", stat.getNum(i));
        }
        printOut("High: ", high);
        printOut("Low: ", low);
        printOut("Total Cycles: ", cycle);
        printOut("Mean", mean);
        printOut("Fastest: ", fastest);
        printOut("SLowest", slowest);
//        printOut("Fastest: ", stat.getHighestValue());
//        printOut("SLowest", stat.getLowestValue());
    }
}
