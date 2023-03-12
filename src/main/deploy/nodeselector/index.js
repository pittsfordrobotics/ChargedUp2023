// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

import { NT4_Client } from "./NT4.js";

const currentNodeToDashboardTopic = "/nodeselector/current_node_to_dashboard";
const currentNodeToRobotTopic = "/nodeselector/current_node_to_robot";
const filledNodeToDashboardTopic = "/nodeselector/filled_node_to_dashboard";
const filledNodeToRobotTopic = "/nodeselector/filled_node_to_robot";
const activeToDashboardTopic = "/nodeselector/active_to_dashboard";
const activeToRobotTopic = "/nodeselector/active_to_robot";
var timer;
const timeout = 250;
var active = true;
var lastIndex = 0;

function setActive(index) {
  if (index !== null) {
      Array.from(document.getElementsByClassName("active")).forEach((element) => {
        element.classList.remove("active");
      });
    Array.from(document.getElementsByClassName("robot")).forEach((element) => {
      element.classList.remove("robot");
    });
      if (active) {
        document.getElementsByTagName("td")[index].classList.add("active");
      }
      else {
        document.getElementsByTagName("td")[index].classList.add("robot");
      }
  }
}

function toggleFilled(index) {
  timer = null;
  if (index !== null) {
    let arr = new Array(27).fill(false);
    for (let i = 0; i < 27; i++) {
      if (i === index) {
        arr[i] = !document.getElementsByTagName("td")[i].classList.contains("filled");
      }
      else {
        arr[i] = document.getElementsByTagName("td")[i].classList.contains("filled");
      }
    }
    client.addSample(filledNodeToRobotTopic, arr);
    client.addSample(currentNodeToRobotTopic, lastIndex);
  }
}

function setFilled(index) {
  if (index !== null && index.length !== 0) {
    for (let i = 0; i < 27; i++) {
      if (index[i]) {
        document.getElementsByTagName("td")[i].classList.add("filled");
      } else {
        document.getElementsByTagName("td")[i].classList.remove("filled");
      }
    }
  }
}

let client = new NT4_Client(
  window.location.hostname,
  "NodeSelector",
  (topic) => {
    // Topic announce
  },
  (topic) => {
    // Topic unannounce
  },
  (topic, timestamp, value) => {
    // New data
    if (topic.name === currentNodeToDashboardTopic) {
      document.body.style.backgroundColor = "black";
      setActive(value);
      lastIndex = value;
    }
    if (topic.name === filledNodeToDashboardTopic) {
      document.body.style.backgroundColor = "black";
      setFilled(value);
    }
    if (topic.name === activeToDashboardTopic) {
      document.body.style.backgroundColor = "black";
      active = (value === 1);
      setActive(lastIndex);
    }
  },
  () => {
    // Connected
  },
  () => {
    // Disconnected
    document.body.style.backgroundColor = "red";
    setActive(null);
    setFilled(null);
    active = true;
  }
);

window.addEventListener("load", () => {
  // Start NT connection
  client.subscribe([currentNodeToDashboardTopic, filledNodeToDashboardTopic, activeToDashboardTopic], false, false, 0.02);
  client.publishTopic(currentNodeToRobotTopic, "int");
  client.publishTopic(filledNodeToRobotTopic, "boolean[]");
  client.publishTopic(activeToRobotTopic, "int");
  client.connect();

  addEventListener("contextmenu", (event) => {
    event.preventDefault();
  });

  // Add click listeners
  Array.from(document.getElementsByTagName("td")).forEach((cell, index) => {
    cell.addEventListener("mousedown", () => {
        timer = setTimeout(function() { toggleFilled(index) }, timeout);
    });
    cell.addEventListener("mouseup", () => {
      if (timer) {
        clearTimeout(timer);
        timer = null;
        if (document.getElementsByTagName("td")[index].classList.contains("active")) {
          client.addSample(activeToRobotTopic, 0);
        }
        else {
          client.addSample(activeToRobotTopic, 1);
          client.addSample(currentNodeToRobotTopic, index);
        }
      }
    });
    // cell.addEventListener("mousemove", () => {
    //   if (timer) {
    //     clearTimeout(timer);
    //     timer = null;
    //   }
    // });
    // cell.addEventListener("click", () => {
    //   if (document.getElementsByTagName("td")[index].classList.contains("active")) {
    //     client.addSample(activeToRobotTopic, 0);
    //   }
    //   else {
    //     client.addSample(activeToRobotTopic, 1);
    //     client.addSample(currentNodeToRobotTopic, index);
    //   }
    // });
    // cell.addEventListener("contextmenu", (event) => {
    //   event.preventDefault();
    //   if (document.getElementsByTagName("td")[index].classList.contains("active")) {
    //     client.addSample(activeToRobotTopic, 0);
    //   }
    //   else {
    //     client.addSample(activeToRobotTopic, 1);
    //     client.addSample(currentNodeToRobotTopic, index);
    //   }
    // });
  });

  // Add touch listeners
  ["touchstart"].forEach((eventString) => {
    document.body.addEventListener(eventString, (event) => {
      event.preventDefault();
      if (event.touches.length > 0) {
        let x = event.touches[0].clientX;
        let y = event.touches[0].clientY;
        Array.from(document.getElementsByTagName("td")).forEach(
          (cell, index) => {
            let rect = cell.getBoundingClientRect();
            if (
              x >= rect.left &&
              x <= rect.right &&
              y >= rect.top &&
              y <= rect.bottom
            ) {
              timer = setTimeout(function() { toggleFilled(index) }, timeout);
              if (document.getElementsByTagName("td")[index].classList.contains("active")) {
                client.addSample(activeToRobotTopic, 0);
              }
              else {
                client.addSample(activeToRobotTopic, 1);
                client.addSample(currentNodeToRobotTopic, index);
              }
            }
          }
        );
      }
    });
  });

  ["touchend", "touchmove"].forEach((eventString) => {
    document.body.addEventListener(eventString, (event) => {
      if (timer){
        clearTimeout(timer);
      }
    });
  });

});