// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team3181.frc2023.subsystems.objectivetracker;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Filesystem;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;
import java.nio.file.Paths;

public class NodeSelectorIOServer implements NodeSelectorIO {
  private final IntegerPublisher currentNodePublisher;
  private final IntegerSubscriber currentNodeSubscriber;
  private final BooleanArrayPublisher filledNodePublisher;
  private final BooleanArraySubscriber filledNodeSubscriber;
  private final IntegerPublisher activePublisher;
  private final IntegerSubscriber activeSubscriber;

  public NodeSelectorIOServer() {
    // Create publisher and subscriber
    var table = NetworkTableInstance.getDefault().getTable("nodeselector");
    currentNodePublisher = table.getIntegerTopic("current_node_to_dashboard").publish();
    currentNodeSubscriber = table.getIntegerTopic("current_node_to_robot").subscribe(-1);
    filledNodePublisher = table.getBooleanArrayTopic("filled_node_to_dashboard").publish();
    filledNodeSubscriber = table.getBooleanArrayTopic("filled_node_to_robot").subscribe(new boolean[]{});
    activePublisher = table.getIntegerTopic("active_to_dashboard").publish();
    activeSubscriber = table.getIntegerTopic("active_to_robot").subscribe(-1);

    // Start server
    var app =
        Javalin.create(
            config -> {
              config.staticFiles.add(
                  Paths.get(
                                  Filesystem.getDeployDirectory().getAbsolutePath(),
                          "nodeselector")
                      .toString(),
                  Location.EXTERNAL);
            });
    app.start(5800);
  }

  @Override
  public void updateInputs(NodeSelectorIOInputs inputs) {
    for (var value : currentNodeSubscriber.readQueueValues()) {
      inputs.selected = value;
    }
    for (var value : filledNodeSubscriber.readQueueValues()) {
      inputs.filled = value;
    }
    for (var value : activeSubscriber.readQueueValues()) {
      inputs.active = value;
    }
  }

  @Override
  public void setSelected(long selected) {
    currentNodePublisher.set(selected);
  }

  @Override
  public void setFilled(boolean[] selected) {
    filledNodePublisher.set(selected);
  }

  @Override
  public void setActive(long active) {
    activePublisher.set(active);
  }
}