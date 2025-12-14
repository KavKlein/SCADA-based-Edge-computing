# Modbus Register Map

| Register| Description                    | Type        |
|---------|---------------------------------|-------------|
| 30001   | Temperature                     | Input Reg   |
| 30002   | Humidity                        | Input Reg   |
| 30003   | Oil level                       | Input Reg   |
| 30004   | DI Status(bits)                 | Input Reg   |
| 30005   | Analog input 1                  | Input Reg   |
| 30006   | Analog input 2                  | Input Reg   |
| 30007   | Module Status                   | Input Reg   |
| 40001   | Digital in/output(Relay states) | Holding Reg |
| 40002   | Digital in/output(LED Control)  | Holding Reg |
| 40003   | "Alarm Acknowledged"            | Holding Reg |
| 40004   | Module Mode                     | Holding Reg |
| 40005   | Temp Setpont                    | Holding Reg |
| 40006   | Hysterisis Setpoint             | Holding Reg |


*Input Registers updated by ESP32 firmware and Holding Registers can be updated by both ESP32 and Ignition Dashboard
