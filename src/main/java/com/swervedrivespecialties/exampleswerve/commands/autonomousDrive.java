package com.swervedrivespecialties.exampleswerve.commands;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class autonomousDrive extends Command {
    private int i = 0;
    List<Double> forwardAr = new ArrayList<>();
    List<Double> strafeAr = new ArrayList<>();
    List<Double> rotationAr = new ArrayList<>();

    public autonomousDrive() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        Scanner inputStream = null;
        try {
            String fileName = "INSERT FILE HERE";
            File file = new File(fileName);

            inputStream = new Scanner(file);
            while (inputStream.hasNext()) {
                String data = inputStream.next();
                String[] arr = data.split(",");

                forwardAr.add(Double.parseDouble(arr[0]));
                strafeAr.add(Double.parseDouble(arr[1]));
                rotationAr.add(Double.parseDouble(arr[2]));
            }

        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (inputStream != null) {
                inputStream.close();
            }
        }
    }

    @Override
    protected void execute() {

        double forward = forwardAr.get(i);
        forward = Math.copySign(Math.pow(forward, 2.0), forward);
        double strafe = strafeAr.get(i);
        strafe = Math.copySign(Math.pow(strafe, 2.0), strafe);
        double rotation = rotationAr.get(i);
        rotation = Math.copySign(Math.pow(rotation, 2.0), rotation);

        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, false);
    }

    @Override
    protected boolean isFinished() {
        if (i == forwardAr.size() - 1) {
            return true;
        } else {
            i++;
            return false;
        }
    }

}