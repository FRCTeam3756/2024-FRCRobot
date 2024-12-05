// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.AIConstants;
import org.json.JSONObject;
import org.json.JSONArray;

public class DSVisionSubsystem extends SubsystemBase {
    private final NetworkTable table;
    private final NetworkTableEntry dataEntry;

    private double[] driveCommand = new double[]{0, 0};
    // private double speakerCommand = 0.0;
    // private boolean intakeCommand = false;
    // private boolean ampCommand = false;
    // private String warnings = null;

    public DSVisionSubsystem() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
    
        inst.setServer(AIConstants.ROBOT_IP_ADDRESS);
        table = inst.getTable(AIConstants.NETWORK_TABLE_NAME);
        dataEntry = table.getEntry(AIConstants.DATA_ENTRY_NAME);
        
    }

    @Override
    public void periodic() {
        updateDataFromNetworkTables();
    }

    private void updateDataFromNetworkTables() {
        String jsonString = dataEntry.getString("{}");
        if (jsonString.trim().isEmpty()) {
            System.out.println("Received empty JSON string.");
            return;
        }

        try {
            if (jsonString.trim().startsWith("[")) {
                JSONArray driveArray = new JSONArray(jsonString);
    
                if (driveArray.length() >= 2) {
                    driveCommand[0] = driveArray.getDouble(0);
                    driveCommand[1] = driveArray.getDouble(1);
                } else {
                    System.out.println("Received array with insufficient data.");
                }
    
            } else if (jsonString.trim().startsWith("{")) {
                JSONObject jsonObject = new JSONObject(jsonString);
                System.out.print(jsonObject);
    
                if (jsonObject.has("commands")) {
                    JSONObject commands = jsonObject.getJSONObject("commands");
                    JSONArray driveArray = commands.getJSONArray("drive");
    
                    driveCommand[0] = driveArray.getDouble(0);
                    driveCommand[1] = driveArray.getDouble(1);
                    
                    // intakeCommand = commands.getBoolean("intake");
                    // speakerCommand = commands.getDouble("speaker");
                    // ampCommand = commands.getBoolean("amp");

                    // warnings = jsonObject.isNull("warnings") ? null : jsonObject.getString("warnings");
                }
            } else {
                System.out.println("Received invalid JSON format: " + jsonString);
            }
        } catch (Exception e) {
            System.out.println("Error parsing JSON: " + e.getMessage());
        }
    }

    public double[] getDriveDirections() {
        return driveCommand;
    }

    public boolean getIntakeCommand() {
        if (driveCommand[0] < AIConstants.ENABLE_INTAKE_DISTANCE) {
            return true;
        } else {
            return false;
        }
    }

    // public boolean getAmpCommand() {
    //     return ampCommand;
    // }

    // public double getSpeakerCommand() {
    //     return speakerCommand;
    // }

    // public String checkWarnings() {
    //     return warnings;
    // }
}
