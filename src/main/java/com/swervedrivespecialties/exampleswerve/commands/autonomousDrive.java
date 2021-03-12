package com.swervedrivespecialties.exampleswerve.commands;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.geometry.Translation2d;

/**
 * Command class for sending values in autonomous to
 * {@link com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem#drive(Translation2d, double, boolean)}
 */
public class autonomousDrive extends Command {
    private int i = 0;
    private int x = 1;
    // Create arrays for each value
    List<Double> forwardAr = new ArrayList<>();
    List<Double> strafeAr = new ArrayList<>();
    List<Double> rotationAr = new ArrayList<>();

    public autonomousDrive() {
        requires(DrivetrainSubsystem.getInstance());
    }

    @Override
    protected void initialize() {
        Scanner inputStream = null;
        try { // Read the CSV file from the string below
            String fileName = "movement values 9.csv";
            Path filePath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
            File file = new File(filePath.toString());
            inputStream = new Scanner(file);
            while (inputStream.hasNext()) {
                String data = inputStream.next();
                String[] arr = data.split(",");
                // Add values to arrays
                forwardAr.add(Double.parseDouble(arr[0]));
                strafeAr.add(Double.parseDouble(arr[1]));
                rotationAr.add(Double.parseDouble(arr[2]));
            }
            // Mandatory exception handling
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
        // Set each variable to a value from their array
        double forward = forwardAr.get(i);
        double strafe = strafeAr.get(i);
        double rotation = rotationAr.get(i);
        // Send values to drive method in DrivetrainSubsystem
        DrivetrainSubsystem.getInstance().drive(new Translation2d(forward, strafe), rotation, false);
    }

    @Override
    protected boolean isFinished() {
        if (i == forwardAr.size() - 1) { // Calls the execute method depending on the array size
            return true; // (Arrays should be same length)
        } else {
            if (x == 6){
                x = 0;
                i++;
            } else {
               x++; 
            }
            return false;
        }
    }

}