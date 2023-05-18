package org.firstinspires.ftc.teamcode.util.container;

public class Value<T> {
    public volatile T value;
    public Value(T value){
        this.value =value;
    }
}
