package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

abstract public class testItem {
        private String description;

        protected testItem(String description) {
                this.description = description;
        }

        public String getDescription() {
                return description;
        }

        abstract public void run(boolean on, Telemetry telemetry);
}