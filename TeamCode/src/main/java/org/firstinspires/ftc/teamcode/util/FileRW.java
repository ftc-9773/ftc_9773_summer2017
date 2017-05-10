/*
 * Copyright (c) 2017 Robocracy 9773
 */

package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.ftccommon.DbgLog;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

/**
 * Created by pranavb on 9/23/16.
 */

public class FileRW {
    File file = null;
    FileReader fileReader = null;
    FileWriter fileWriter = null;
    BufferedReader bufferedReader = null;
    BufferedWriter bufferedWriter = null;
    String fileName;

    public FileRW(String fileName, boolean write){
        this.fileName = fileName;
        try{
            this.file = new File(fileName);
            if(write) {
                file.createNewFile();
                this.fileWriter = new FileWriter(fileName);
            } else if(!write){
                this.fileReader = new FileReader(fileName);
            }
        }
        catch (IOException e){
            DbgLog.error("ftc9773: An Exception was caught: %s", e.getMessage());
        }
        if (write) {
            this.bufferedWriter = new BufferedWriter(fileWriter);
        } else if(!write){
            this.bufferedReader = new BufferedReader(fileReader);
        }
    }

    public void fileWrite(String data){
        try{
            bufferedWriter.write(data);
            bufferedWriter.newLine();
        }
        catch(IOException e){
            DbgLog.error("ftc9773: An Exception was caught: %s", e.getMessage());
        }
    }

    public String getNextLine(){
        String data = null;
        try{
            data = bufferedReader.readLine();
        }
        catch(FileNotFoundException ex) {
            DbgLog.error(String.format(
                    "ftc9773: Unable to open file '%s'", fileName));
        }
        catch (IOException e){
            DbgLog.error(String.format("ftc9773: An IOException was caught : %s", e.getMessage()));
        }

        return data;
    }

    public void close(){
        try {
            if(bufferedWriter!=null) {
                bufferedWriter.flush();
                bufferedWriter.close();
            }
            if(bufferedReader!=null)
                bufferedReader.close();
            rmIfZeroSize(); // Delete the file if size is '0';  this happens when we press "init"
                            // button repititively without pressing "play".
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public void rmIfZeroSize() {
        try {
            if (file.length() <= 0) {
                file.delete();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    public static String getTimeStampedFileName(String fileName) {
        String timestamp = new SimpleDateFormat("yyyy.MM.dd.HH.mm.ss").format(new Date());
        String newFileName = fileName.concat(timestamp);
        return (newFileName);
    }
}
