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

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import org.photonvision.PhotonCamera;
import org.photonvision.common.dataflow.structures.Packet;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;

/**
 * An ObjectDetectionIO implementation that uses a camera connected to hardware running the
 * PhotonVision library to detect objects.
 */
public class ObjectDetectionIOPhotonVision implements ObjectDetectionIO {
    private static final byte[] PHOTON_RESULT_MAGIC = new byte[] {'P', 'H', 'O', 'T', 'O', 'N', 1};

    protected final PhotonCamera camera;
    protected final String cameraName;
    private final Alert disconnectedAlert;

    /**
     * Constructs a {@link ObjectDetectionIOPhotonVision} object with the specified camera name.
     *
     * @param cameraName The name of the camera
     */
    public ObjectDetectionIOPhotonVision(String cameraName) {
        // CameraName is the name of the NetworkTable that PhotonVision is broadcasting information
        // over.
        // The name of the NetworkTable should be the same as the camera’s nickname (from the
        // PhotonVision UI).
        camera = new PhotonCamera(cameraName);
        disconnectedAlert =
                new Alert(
                        "PhotonVision Camera " + cameraName + " is not connected.",
                        AlertType.kError);
        this.cameraName = cameraName;
    }

    @Override
    public void updateInputs(ObjectDetectionIOInputs inputs) {
        /* Verify PhotonVision hardware is connected. */
        inputs.connected = camera.isConnected();
        if (!inputs.connected) {
            disconnectedAlert.set(true);
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }
        /* Update results. */
        disconnectedAlert.set(false);
        // PhotonVision container containing all information about stored targets from camera.
        // List retrieved via .getAllUnreadResults() is FIFO, max size 20, and each call clears
        // the queue. Call once per loop().
        List<PhotonPipelineResult> result = camera.getAllUnreadResults();
        if (result.isEmpty()) {
            inputs.rawResults = new byte[0][];
            inputs.captureTimestampsUs = new long[0];
            inputs.publishTimestampsUs = new long[0];
            return;
        }

        inputs.rawResults =
                result.stream()
                        .map(ObjectDetectionIOPhotonVision::packPhotonResult)
                        .toArray(byte[][]::new);
        inputs.captureTimestampsUs =
                result.stream()
                        .mapToLong(unread -> unread.metadata.captureTimestampMicros)
                        .toArray();
        inputs.publishTimestampsUs =
                result.stream()
                        .mapToLong(unread -> unread.metadata.publishTimestampMicros)
                        .toArray();
    }

    public static byte[] getPhotonResultMagic() {
        byte[] copy = new byte[PHOTON_RESULT_MAGIC.length];
        System.arraycopy(PHOTON_RESULT_MAGIC, 0, copy, 0, PHOTON_RESULT_MAGIC.length);
        return copy;
    }

    private static byte[] packPhotonResult(PhotonPipelineResult result) {
        Packet packet = new Packet(512);
        PhotonPipelineResult.photonStruct.pack(packet, result);
        byte[] packedResult = packet.getWrittenDataCopy();
        byte[] rawResult = new byte[PHOTON_RESULT_MAGIC.length + packedResult.length];
        System.arraycopy(PHOTON_RESULT_MAGIC, 0, rawResult, 0, PHOTON_RESULT_MAGIC.length);
        System.arraycopy(
                packedResult, 0, rawResult, PHOTON_RESULT_MAGIC.length, packedResult.length);
        return rawResult;
    }
}
