# N3uron ESP32 Modbus & Ignition Perspective Integration

## Project Overview

This project demonstrates a complete **Industrial IoT setup** using:

- **ESP32** as a Modbus TCP server controlling relays and LEDs.  
- **n3uron MIDBUS** acting as a Modbus TCP client and OPC UA client/server gateway.  
- **Ignition Perspective** as the SCADA/HMI platform for monitoring and control.  

The system enables **real-time reading and writing** of sensor data and relay/LED states via OPC UA tags, with a dashboard interface for operators.

---

**Data Flow:**

1. **ESP32 Modbus TCP server** exposes sensor readings and control registers (uint16).  
2. **n3uron** subscribes to ESP32 Modbus registers and exposes them as **OPC UA nodes**.  
3. **Ignition Designer** connects to the OPC UA server (either system provider or n3uron provider).  
4. **OPC Tags** in Ignition map to Modbus registers for reading and writing values.  
5. **Expression Tags** handle bit-level monitoring (e.g., relay/LED status bits).  
6. **Perspective dashboard** binds switches to scripts that update the OPC tag (16-bit register).  
7. **Relay/LED control** is performed by updating the correct bits in the uint16 register.

---




