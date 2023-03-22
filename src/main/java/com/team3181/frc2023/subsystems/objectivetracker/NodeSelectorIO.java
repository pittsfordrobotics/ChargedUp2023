// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package com.team3181.frc2023.subsystems.objectivetracker;

import org.littletonrobotics.junction.AutoLog;

public interface NodeSelectorIO {
  @AutoLog
  class NodeSelectorIOInputs {
    public long selected = -1;
    public boolean[] filled = {};
    public long active = -1;
  }

  default void updateInputs(NodeSelectorIOInputs inputs) {}

  default void setSelected(long selected) {}

  default void setFilled(boolean[] selected) {}

  default void setActive(long active) {}
}