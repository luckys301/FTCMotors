package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.HashMap;


public class CycleTracker {//TODO:Remove Static
    protected StatCalculator stat = new StatCalculator();
    protected ElapsedTime timer = new ElapsedTime(0);
    private final Telemetry tl;
//    private final File file = new File("/home/lvuser/Logs");
//    private final PrintStream stream;

//        try {
//            stream = new PrintStream(file);
//        } catch (FileNotFoundException e) {
//            throw new RuntimeException(e);
//        }

    private HashMap<String, Double> data;
    private double cycle =0,
        mean = 0,
        fastest = 0,
        slowest = 0,
        high = 0,
        low = 0,
        elapsedTime = 0;

    public CycleTracker(Telemetry tl) {
        timer.reset();
        this.tl = tl;
    }
    public void resetTimer(){
        timer.reset();
    }

    /*
    public double getCycle(){ return cycle;}
    public double getMean(){ return mean;}
    public double getFastest(){ return fastest;}
    public double getSlowest(){ return slowest;}
    public double getHigh(){ return high;}
    public double getLow(){ return low;}
    public double getElapsedTime(){ return elapsedTime;}
    */



    public void trackCycle(int num){
        double cycleTime = timer.seconds();
        stat.addNumber(cycleTime);
        cycle = stat.getSizeDouble();
        mean = stat.getMean();
        slowest = stat.getHighestValue();
        fastest = stat.getLowestValue();
        elapsedTime = stat.getSum();
        // switch (getPosition()){
        //     case HIGH:
        //         high.set(high.get()+1);
        //         break;
        //     case LOW:
        //         low.set(low.get()+1);
        //         break;
        switch (num){
            case 1:
                high = (high+1);
                break;
            case 2:
                low = (low+1);
                break;
        }    // }
        timer.reset();
    }

//    public  void printOut(String string, double num){
//        data.put(string, num);
//    }
//    public  void printAllData(){
////        printOut(DriverStation.getMatchType().toString(), DriverStation.getMatchNumber());
//
//        for (int i = 0; i < stat.getSizeInt(); i++) {
//            printOutStream("Cycle "+ i +":", stat.getNum(i));
//        }
//        printOut("High: ", high);
//        printOut("Low: ", low);
//        printOut("Total Cycles: ", cycle);
//        printOut("Mean", mean);
//        printOut("Fastest: ", fastest);
//        printOut("SLowest", slowest);
////        printOut("Fastest: ", stat.getHighestValue());
////        printOut("SLowest", stat.getLowestValue());
//    }
//    public  void printOutStream(String string, double num){
//        stream.print(string + num);
////        data.put(string, num);
//    }
    public void cyclePeriodic(){
//        tl.addData("Elapsed Time", getElapsedTime());
//        tl.addData("Mean", getMean());
//        tl.addData("Cycle", getCycle());
//
//        tl.addData("high", getHigh());
//        tl.addData("low", getLow());

//        tl.addData("fast", getFastest());
//        tl.addData("slow", getSlowest());

        tl.addData("Elapsed Time", elapsedTime);
        tl.addData("Mean", mean);
        tl.addData("Cycle", cycle);

        tl.addData("high", high);
        tl.addData("low", low);

        tl.addData("fast", fastest);
        tl.addData("slow", slowest);
    }
}
