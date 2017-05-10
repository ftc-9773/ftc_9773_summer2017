/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;

import java.util.List;

/**
 * Created by pranavburugula on 3/5/2017.
 */

public class StateMachine {
    private List<String> states = null;
    private int stateIndex = 0;

    public StateMachine(List<String> states){
        this.states = states;
    }

    public String getCurState(){
        return states.get(stateIndex);
    }

    public void switchState(String newState){
        if (states.contains(newState)){
            stateIndex = states.indexOf(newState);
        } else {
            DbgLog.error("ftc9773: Invalid state");
        }
    }

    public void nextState(){
        stateIndex = (stateIndex+1 ==states.size())? 0 : stateIndex++;
    }

    public void prevState(){
        stateIndex = (stateIndex == 0)? states.size()-1 : stateIndex--;
    }
}
