/*
 * Copyright (C) 2026 Windham Windup
 *
 * This program is free software: you can redistribute it and/or modify it under the terms of the
 * GNU General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without
 * even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with this program. If
 * not, see <https://www.gnu.org/licenses/>.
 */

package frc.lib.io.objectdetection;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/**
 * Standardized interface for ObjectDetection-IO used in FRC. This interface is often implemented
 * through an ML or HSV Color pipeline. Currently factored for PhotonVision only.
 */
public interface ObjectDetectionIO {

    /**
     * Class defining data type for Object Detection updateInputs method (i.e. data structure the
     * camera will place its results into). Fields of this class will be populated downstream by the
     * implemented IO layer (e.g. ObjectDetectionIOPhotonVision), used in calculations at the device
     * layer, and used for decisions at the subsystem layer.
     */
    public class ObjectDetectionIOInputs implements LoggableInputs {
        /** Whether the camera is connected. */
        public boolean connected = false;

        /** Raw unread frame payloads from the detector since last update. */
        public byte[][] rawResults = new byte[0][];

        /** Capture timestamps for unread results, in microseconds. */
        public long[] captureTimestampsUs = new long[0];

        /** Publish timestamps for unread results, in microseconds. */
        public long[] publishTimestampsUs = new long[0];

        @Override
        public void toLog(LogTable table) {
            table.put("Connected", connected);
            table.put("ResultsLength", rawResults.length);
            table.put("CaptureTimestampsUs", captureTimestampsUs);
            table.put("PublishTimestampsUs", publishTimestampsUs);
            for (int i = 0; i < rawResults.length; i++) {
                table.put("RawResult/" + i, rawResults[i]);
            }
        }

        @Override
        public void fromLog(LogTable table) {
            connected = table.get("Connected", false);
            int resultsLength = table.get("ResultsLength", 0);
            rawResults = new byte[resultsLength][];
            captureTimestampsUs = table.get("CaptureTimestampsUs", new long[0]);
            publishTimestampsUs = table.get("PublishTimestampsUs", new long[0]);
            for (int i = 0; i < resultsLength; i++) {
                rawResults[i] = table.get("RawResult/" + i, new byte[0]);
            }
        }
    }

    /**
     * Updates the provided ObjectDetectionIOInputs with the latest camera readings. If the camera
     * is not connected, the ObjectDetectionIOInput fields remain empty.
     *
     * @param inputs The structure to populate with updated target detection data
     */
    public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
