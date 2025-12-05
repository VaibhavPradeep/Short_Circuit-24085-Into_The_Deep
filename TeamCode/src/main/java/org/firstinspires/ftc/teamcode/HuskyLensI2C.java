package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.I2cAddr;

public class HuskyLensI2C {

    public enum ArtifactColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public enum MotifType {
        GPP,
        PGP,
        PPG,
        UNKNOWN
    }

    private I2cDeviceSynch device;

    // Default I2C address for HuskyLens
    private static final I2cAddr HUSKY_ADDR = I2cAddr.create7bit(0x32);

    // Command to request blocks
    private static final byte[] REQUEST_BLOCKS = {
            (byte)0x55, (byte)0xAA, // header
            (byte)0x11,             // command type: request blocks
            (byte)0x02,             // data length
            (byte)0x2C, (byte)0x00  // payload = get blocks
    };

    public HuskyLensI2C(HardwareMap hwMap, String name) {
//        device = new I2cDeviceSynchImpl(hwMap.i2cDeviceSynch.get(name), HUSKY_ADDR, false);
        device.engage();
    }

    /**
     * Send command to HuskyLens to request blocks and return raw frame.
     */
    private byte[] readFrame() {
        device.write(REQUEST_BLOCKS);
        sleep(30); // give HuskyLens time to reply

        return device.read(32);  // read 32 bytes (enough for several blocks)
    }

    /**
     * Parse AprilTag ID from HuskyLens data.
     */
    public int getAprilTagId() {
        byte[] frame = readFrame();

        // HuskyLens protocol: blocks start with 0x55 0xAA and include ID at fixed position
        for (int i = 0; i < frame.length - 5; i++) {
            if ((frame[i] & 0xFF) == 0x55 && (frame[i+1] & 0xFF) == 0xAA) {
                int id = frame[i+5] & 0xFF;
                // AprilTag IDs are typically > 20 for this game
                if (id == 21 || id == 22 || id == 23) return id;
            }
        }
        return -1;
    }

    /**
     * Convert AprilTag ID to motif.
     */
    public MotifType motifFromTagId(int id) {
        switch (id) {
            case 21: return MotifType.GPP;
            case 22: return MotifType.PGP;
            case 23: return MotifType.PPG;
            default: return MotifType.UNKNOWN;
        }
    }

    /**
     * Read artifact color labels from HuskyLens (color recognition mode).
     */
    public ArtifactColor getArtifactColor() {
        byte[] frame = readFrame();

        // Look inside frame for labels:
        // frame[i+5] holds ID (color ID > 0)
        // frame[i+6..] holds further data which may include label index

        for (int i = 0; i < frame.length - 5; i++) {
            if ((frame[i] & 0xFF) == 0x55 && (frame[i+1] & 0xFF) == 0xAA) {
                int id = frame[i+5] & 0xFF;

                // Your training should label:
                //   Green artifacts   -> ID 1
                //   Purple artifacts  -> ID 2

                if (id == 1) return ArtifactColor.GREEN;
                if (id == 2) return ArtifactColor.PURPLE;
            }
        }

        return ArtifactColor.UNKNOWN;
    }

    private void sleep(long ms) {
        try { Thread.sleep(ms); } catch (Exception ignored) {}
    }
}
