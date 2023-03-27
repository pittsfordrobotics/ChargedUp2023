// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team3181.frc2023.subsystems.objectivetracker;

import com.team3181.frc2023.Constants.RobotConstants;
import com.team3181.frc2023.FieldConstants.AutoDrivePoints;
import com.team3181.frc2023.subsystems.swerve.Swerve;
import com.team3181.lib.math.GeomUtil;
import com.team3181.lib.util.VirtualSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import org.littletonrobotics.junction.Logger;

public class ObjectiveTracker extends VirtualSubsystem {
  private final NodeSelectorIO selectorIO;
  private final NodeSelectorIOInputsAutoLogged selectorInputs = new NodeSelectorIOInputsAutoLogged();
  private final Objective objective = new Objective();
  private boolean manualSelection = true;
  private final boolean linkMode = false;
  private boolean[] filled = new boolean[27];

  public static class Objective {
    public int nodeRow; // The row of the selected target node
    public NodeLevel nodeLevel; // The level of the selected target node

    public Objective(int nodeRow, NodeLevel nodeLevel) {
      this.nodeRow = nodeRow;
      this.nodeLevel = nodeLevel;
    }

    public Objective() {
      this.nodeRow = 0;
      this.nodeLevel = NodeLevel.MID;
    }
  }

  private final static ObjectiveTracker INSTANCE = new ObjectiveTracker(RobotConstants.NODE_SELECTOR);

  public static ObjectiveTracker getInstance() {
    return INSTANCE;
  }

  private ObjectiveTracker(NodeSelectorIO selectorIO) {
    this.selectorIO = selectorIO;
  }

  @Override
  public void periodic() {
    selectorIO.updateInputs(selectorInputs);
    Logger.getInstance().processInputs("NodeSelector", selectorInputs);

    // Read updates from node selector
    if (selectorInputs.selected != -1) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        objective.nodeRow = 8 - ((int) selectorInputs.selected % 9);
      } else {
        objective.nodeRow = (int) selectorInputs.selected % 9;
      }
      if (selectorInputs.selected < 9) {
        objective.nodeLevel = NodeLevel.HYBRID;
      } else if (selectorInputs.selected < 18) {
        objective.nodeLevel = NodeLevel.MID;
      } else {
        objective.nodeLevel = NodeLevel.HIGH;
      }
      selectorInputs.selected = -1;
    }

    // Read updates from node filled
    if (selectorInputs.filled.length != 0) {
      if (DriverStation.getAlliance() == Alliance.Blue) {
        for (int i = 8; i >= 0; i--) {
          filled[8-i] = selectorInputs.filled[i];
        }
        for (int i = 17; i >= 9; i--) {
          filled[26-i] = selectorInputs.filled[i];
        }
        for (int i = 26; i >= 18; i--) {
          filled[44-i] = selectorInputs.filled[i];
        }
      }
      else {
        filled = selectorInputs.filled;
      }
      selectorInputs.filled = new boolean[]{};
    }

    // updates manualSelection
    if (selectorInputs.active != -1) {
      manualSelection = selectorInputs.active == 1;
      selectorInputs.active = -1;
    }

    // automatically chooses optimal place to place game piece
    if (!manualSelection && !DriverStation.isAutonomous()) {
      double topDist = GeomUtil.distance(new Pose2d(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_TOP_INNER, DriverStation.getAlliance()).getPosition(), new Rotation2d()), Swerve.getInstance().getPose());
      double bottomDist = GeomUtil.distance(new Pose2d(AutoDrivePoints.pathPointFlipper(AutoDrivePoints.COMMUNITY_BOTTOM_INNER, DriverStation.getAlliance()).getPosition(), new Rotation2d()), Swerve.getInstance().getPose());

      boolean objectiveFound = false;
      if (topDist < bottomDist) {
        if (linkMode) {
          for (int i = 18 + 2; i < 27; i++) {
            if (filled[i - 1] && filled[i - 2] && !filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            } else if (filled[i] && filled[i - 1] && !filled[i - 2]) {
              objective.nodeRow = (i - 2) % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            } else if (filled[i] && filled[i - 2] && !filled[i - 1]) {
              objective.nodeRow = (i - 1) % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            }
          }
        }
        if (!objectiveFound) {
          for (int i = 18; i < 27; i++) {
            if (!filled[i]) {
                objective.nodeRow = i % 9;
                objective.nodeLevel = NodeLevel.HIGH;
                objectiveFound = true;
                break;
            }
          }
        }
        if (!objectiveFound && linkMode) {
          for (int i = 0 + 2; i < 9; i++) {
            if (filled[i-1] && filled[i-2] && !filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              objectiveFound = true;
              break;
            }
            else if (filled[i] && filled[i-1] && !filled[i-2]) {
              objective.nodeRow = (i-2) % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              objectiveFound = true;
              break;
            }
            else if (filled[i] && filled[i-2] && !filled[i-1]) {
              objective.nodeRow = (i-1) % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              objectiveFound = true;
              break;
            }
          }
        }
        if (!objectiveFound) {
          for (int i = 0; i < 9; i++) {
            if (!filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              objectiveFound = true;
              break;
            }
          }
        }
        if (!objectiveFound) {
          for (int i = 9; i < 18; i++) {
            if (!filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.MID;
              objectiveFound = true;
              break;
            }
          }
        }
      }
      else {
        if (linkMode) {
          for (int i = 26 - 2; i >= 18; i--) {
            if (filled[i + 1] && filled[i + 2] && !filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            } else if (filled[i] && filled[i + 1] && !filled[i + 2]) {
              objective.nodeRow = (i + 2) % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            } else if (filled[i] && filled[i + 2] && !filled[i + 1]) {
              objective.nodeRow = (i + 1) % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            }
          }
        }
        if (!objectiveFound) {
          for (int i = 26; i >= 18; i--) {
            if (!filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HIGH;
              objectiveFound = true;
              break;
            }
          }
        }
        if (!objectiveFound && linkMode) {
          for (int i = 8 - 2; i >= 0; i--) {
            if (filled[i+1] && filled[i+2] && !filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              break;
            }
            else if (filled[i] && filled[i+1] && !filled[i+2]) {
              objective.nodeRow = (i+2) % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              break;
            }
            else if (filled[i] && filled[i+2] && !filled[i+1]) {
              objective.nodeRow = (i+1) % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              break;
            }
          }
        }
        if (!objectiveFound) {
          for (int i = 8; i >= 0; i--) {
            if (!filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.HYBRID;
              objectiveFound = true;
              break;
            }
          }
        }
        if (!objectiveFound) {
          for (int i = 17; i >= 9; i--) {
            if (!filled[i]) {
              objective.nodeRow = i % 9;
              objective.nodeLevel = NodeLevel.MID;
              objectiveFound = true;
              break;
            }
          }
        }
      }
    }

    // Send current data to selector
    {
      selectorIO.setSelected(objectiveToSelected());
      boolean[] tempFilled = new boolean[27];
      if (DriverStation.getAlliance() == Alliance.Blue) {
        for (int i = 8; i >= 0; i--) {
          tempFilled[8-i] = filled[i];
        }
        for (int i = 17; i >= 9; i--) {
          tempFilled[26-i] = filled[i];
        }
        for (int i = 26; i >= 18; i--) {
          tempFilled[44-i] = filled[i];
        }
      } else {
        tempFilled = filled;
      }
      selectorIO.setFilled(tempFilled);
      selectorIO.setActive(manualSelection ? 1 : 0);
    }

    // Send current node as text
    {
      String text = "";
      switch (objective.nodeLevel) {
        case HYBRID -> text += "HYBRID";
        case MID -> text += "MID";
        case HIGH -> text += "HIGH";
      }
      text += ", ";
      if (objective.nodeRow < 3) {
        text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
      } else if (objective.nodeRow < 6) {
        text += "CO-OP";
      } else {
        text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
      }
      text += " grid, ";
      if (objective.nodeRow == 1 || objective.nodeRow == 4 || objective.nodeRow == 7) {
        text += objective.nodeLevel == NodeLevel.HYBRID ? "CENTER" : "CUBE";
      } else if (objective.nodeRow == 0 || objective.nodeRow == 3 || objective.nodeRow == 6) {
        text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
        text += objective.nodeLevel == NodeLevel.HYBRID ? "" : " CONE";
      } else {
        text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
        text += objective.nodeLevel == NodeLevel.HYBRID ? "" : " CONE";
      }
      text += " node";
//      SmartDashboard.putString("Selected Node", text);
    }

    // Log state
    Logger.getInstance().recordOutput("ObjectiveTracker/Node Level", objective.nodeLevel.toString());
    Logger.getInstance().recordOutput("ObjectiveTracker/Node Row", objective.nodeRow);
  }

  public Objective getObjective() {
    return objective;
  }

  /** Shifts the selected node in the selector by one position. */
  public void shiftNode(Direction direction) {
    manualSelection = true;
    switch (direction) {
      case LEFT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          if (objective.nodeRow < 8) {
            objective.nodeRow += 1;
          }
        } else {
          if (objective.nodeRow > 0) {
            objective.nodeRow -= 1;
          }
        }
        break;

      case RIGHT:
        if (DriverStation.getAlliance() == Alliance.Blue) {
          if (objective.nodeRow > 0) {
            objective.nodeRow -= 1;
          }
        } else {
          if (objective.nodeRow < 8) {
            objective.nodeRow += 1;
          }
        }
        break;

      case UP:
        switch (objective.nodeLevel) {
          case HYBRID -> {}
          case MID -> objective.nodeLevel = NodeLevel.HYBRID;
          case HIGH -> objective.nodeLevel = NodeLevel.MID;
        }
        break;

      case DOWN:
        switch (objective.nodeLevel) {
          case HYBRID -> objective.nodeLevel = NodeLevel.MID;
          case MID -> objective.nodeLevel = NodeLevel.HIGH;
          case HIGH -> {}
        }
        break;
    }
  }

  /** Toggle whether current cell is filled */
  public void toggleFilled() {
    int selected = objectiveToSelected();
    filled[selected] = !filled[selected];
  }

  private int objectiveToSelected() {
    int selected;
    if (DriverStation.getAlliance() == Alliance.Blue) {
      selected = 8 - objective.nodeRow;
    } else {
      selected = objective.nodeRow;
    }
    switch (objective.nodeLevel) {
      case HYBRID -> selected += 0;
      case MID -> selected += 9;
      case HIGH -> selected += 18;
    }
    return selected;
  }

  /** Toggle whether automation is enabled */
  public void toggleActive() {
    manualSelection = !manualSelection;
  }

  /** Set automation enabled */
  public void setAutomated() {
    manualSelection = false;
  }

  /** Use automation to update objective */
  public void updateObjective(Objective objective) {
    this.objective.nodeRow = objective.nodeRow;
    this.objective.nodeLevel = objective.nodeLevel;
  }

  /** Command factory to shift the selected node in the selector by one position. */
  public Command shiftNodeCommand(Direction direction) {
    return new InstantCommand(() -> shiftNode(direction))
        .andThen(
            Commands.waitSeconds(0.3),
            Commands.repeatingSequence(
                new InstantCommand(() -> shiftNode(direction)), new WaitCommand(0.1)))
        .ignoringDisable(true);
  }

  public enum NodeLevel {
    HYBRID,
    MID,
    HIGH
  }

  public enum Direction {
    LEFT,
    RIGHT,
    UP,
    DOWN
  }
}