# System Architecture

ESP32 → Modbus TCP → N3uron Gateway → OPC UA → Ignition SCADA

## ESP32
- Reads sensors
- Publishes registers via Modbus TCP server

## N3uron
- Acts as protocol hub
- Converts Modbus → OPC UA
- Handles connection resilience

## Ignition SCADA
- Tag historian
- Dashboards
- Alarms
