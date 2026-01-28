# Software Requirements Specification: ECU Bridge Rework

## 1. Introduction
The ECU Bridge is a critical component that facilitates communication between the high-level ROS 2 control system (running on a Raspberry Pi or similar) and the low-level Chassis Controller (ECU, running on BeagleBone Blue or similar). This document describes the requirements for a redesigned ECU Bridge that uses a Serial interface and implement a robust caching and periodic update mechanism.

## 2. System Architecture
The new architecture consists of three main layers:
1.  **Serial Transport**: Manages low-level serial port operations (opening, closing, reading, and writing bytes).
2.  **ECU Protocol**: Implements the communication protocol, including framing, CRC verification, and command serialization/deserialization.
3.  **ECU Bridge**: The top-level component that manages the state cache, handles periodic polling of sensors (IMU, Encoders), and periodic updates of actuators (Motors).

## 3. Communication Requirements

### 3.1 Serial Interface
- **SR.1.1 Configurable Port**: The bridge shall support configurable serial port paths (e.g., `/dev/ttyAMA0`) and baud rates (defaulting to 115200).
- **SR.1.2 Robust Reading**: The transport shall implement a "start byte" search mechanism (using `0xAA` as defined in `ecu_sw_bb`) to synchronize with the byte stream.

### 3.2 Protocol Layer
- **SR.2.1 Protocol Compliance**: The protocol implementation shall strictly follow the specifications defined in [ecu_sw_bb/docs/protocol.md](../../../../../ecu_sw_bb/docs/protocol.md).
- **SR.2.2 Framing and Integrity**: The implementation shall handle message framing, CRC16 calculation, and verification as specified in the protocol document.
- **SR.2.3 Command Set**: All commands (API version, motor control, encoder retrieval, and IMU data) shall be supported according to the defined command IDs and payload structures.

## 4. Functional Requirements

### 4.1 Data Caching
- **FR.1.1 State Cache**: The ECU Bridge shall maintain a local cache for:
    - Motor Speeds (Setpoints)
    - Encoder Values
    - IMU Data (Accelerometer, Gyroscope, Magnetometer, Orientation)
    - API Version
- **FR.1.2 Thread Safety**: Access to the cache shall be thread-safe to allow concurrent access from periodic tasks and ROS 2 interface calls.

### 4.2 Periodic Tasks
- **FR.2.1 Polling Mechanism**: The ECU Bridge shall implement periodic polling of ECU data.
- **FR.2.2 Configurable Periods**: Each parameter group shall have a configurable update period:
    - Default IMU update rate: 50Hz (20ms)
    - Default Encoder update rate: 20Hz (50ms)
    - Default Motor setpoint update rate: 10Hz (100ms)
- **FR.2.3 Async Execution**: Periodic tasks shall run in their own thread(s) to avoid blocking the main ROS 2 executor.

### 4.3 Error Handling
- **ER.1.1 Connection Monitoring**: The system shall detect serial communication timeouts or CRC errors.
- **ER.1.2 Reconnection**: Upon failure, the bridge shall attempt to re-establish the serial connection.
- **ER.1.3 Stale Data**: The cache shall track data validity; if communication is lost for more than a threshold time, data shall be marked as invalid.
