package com.team3181.frc2023.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** Vision subsystem hardware interface. */
public interface VisionIO {
    @AutoLog
    class VisionIOInputs {
        public double captureTimestamp = 0.0;
        public double[] cornerX = new double[] {};
        public double[] cornerY = new double[] {};
        public boolean hasTarget = false;
        public boolean connected = false;
        public double vAngle = 0.0;
        public double hAngle = 0.0;
    }

    enum Pipelines {
        APRIL_TAGS(0), RETROREFLECTOR(1);

        private final int num;
        Pipelines(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum LED {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        private final int num;

        LED(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    enum CameraMode {
        VISION_PROCESSING(0), DRIVER_CAMERA(1);

        private final int num;

        CameraMode(int num) {
            this.num = num;
        }

        public int getNum() {
            return num;
        }
    }

    /** Updates the set of loggable inputs. */
    default void updateInputs(VisionIOInputs inputs) {}

    /** Enabled or disabled vision LEDs. */
    default void setLEDs(LED led) {}

    /** Enabled or disabled vision LEDs. */
    default void setCameraModes(CameraMode mode) {}

    /** Sets the pipeline number. */
    default void setPipeline(Pipelines pipeline) {}
}