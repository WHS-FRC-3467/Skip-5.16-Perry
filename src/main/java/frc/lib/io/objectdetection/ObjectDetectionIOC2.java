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

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.networktables.TimestampedRaw;
import edu.wpi.first.util.WPIUtilJNI;

import java.util.ArrayList;

/**
 * Real hardware implementation of {@link ObjectDetectionIO} using c2 object-detection output.
 *
 * <p>Reads flatbuffer detection frames as raw bytes so decoding can happen in the device layer.
 */
public class ObjectDetectionIOC2 implements ObjectDetectionIO {
    private static final String DEFAULT_DEVICE_ID = "dsv1";
    private static final String DEFAULT_TABLE_NAME = "video1_yolo";
    private static final String FLATBUFFER_TYPE = "objectdetections_fb";
    private static final int DEFAULT_POLL_STORAGE_DEPTH = 32;
    private static final long DISCONNECT_TIMEOUT_US = 500_000L;

    /** Shared configuration for c2 object detection output. */
    public static record C2Config(String deviceId, String tableName, int pollStorageDepth) {}

    private final NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
    private final RawSubscriber detectionSubscriber;

    public ObjectDetectionIOC2() {
        this(defaults());
    }

    public ObjectDetectionIOC2(C2Config config) {
        C2Config validatedConfig = validateConfig(config);
        NetworkTable detectionTable =
                ntInstance.getTable(
                        "/" + validatedConfig.deviceId() + "/" + validatedConfig.tableName());
        detectionSubscriber =
                detectionTable
                        .getRawTopic("detections")
                        .subscribe(
                                FLATBUFFER_TYPE,
                                new byte[0],
                                PubSubOption.sendAll(true),
                                PubSubOption.keepDuplicates(true),
                                PubSubOption.pollStorage(validatedConfig.pollStorageDepth()));
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        long nowUs = WPIUtilJNI.now();
        long lastChangeUs = detectionSubscriber.getLastChange();

        inputs.connected =
                ntInstance.isConnected()
                        && detectionSubscriber.exists()
                        && lastChangeUs > 0
                        && nowUs - lastChangeUs <= DISCONNECT_TIMEOUT_US;

        TimestampedRaw[] unreadFrames = detectionSubscriber.readQueue();
        if (unreadFrames.length == 0) {
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        ArrayList<byte[]> results = new ArrayList<>(unreadFrames.length);
        ArrayList<Long> captureTimestampsUs = new ArrayList<>(unreadFrames.length);
        ArrayList<Long> publishTimestampsUs = new ArrayList<>(unreadFrames.length);
        for (TimestampedRaw unreadFrame : unreadFrames) {
            if (unreadFrame != null && unreadFrame.value != null && unreadFrame.value.length > 0) {
                results.add(unreadFrame.value);
                captureTimestampsUs.add(unreadFrame.timestamp);
                publishTimestampsUs.add(
                        unreadFrame.serverTime != 0
                                ? unreadFrame.serverTime
                                : unreadFrame.timestamp);
            }
        }

        inputs.rawResults = results.toArray(byte[][]::new);
        inputs.captureTimestampsUs =
                captureTimestampsUs.stream().mapToLong(Long::longValue).toArray();
        inputs.publishTimestampsUs =
                publishTimestampsUs.stream().mapToLong(Long::longValue).toArray();
    }

    public static C2Config defaults() {
        return new C2Config(DEFAULT_DEVICE_ID, DEFAULT_TABLE_NAME, DEFAULT_POLL_STORAGE_DEPTH);
    }

    private static C2Config validateConfig(C2Config config) {
        if (config == null) {
            throw new IllegalArgumentException("c2Config cannot be null");
        }
        if (config.deviceId() == null || config.deviceId().isBlank()) {
            throw new IllegalArgumentException("c2Config.deviceId cannot be blank");
        }
        if (config.tableName() == null || config.tableName().isBlank()) {
            throw new IllegalArgumentException("c2Config.tableName cannot be blank");
        }
        if (config.pollStorageDepth() <= 0) {
            throw new IllegalArgumentException("c2Config.pollStorageDepth must be positive");
        }
        return config;
    }
}
