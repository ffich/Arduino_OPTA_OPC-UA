# ⚡ Arduino_OPTA_OPC-UA

## 🗂 Overview

This project implements an **🖧 OPC-UA server** on an **🔌 Arduino OPTA** platform, complemented by a **🌐 Node-RED client** that serves as a control panel for the OPTA-based system. It enables real-time control and monitoring of:

- ⚙ **OPTA relays**
- 🔘 **Digital inputs** (e.g., buttons)
- 🌡 **Modbus-connected temperature and humidity sensor** via the OPTA’s Modbus port

The repository was created to demonstrate seamless industrial device integration via OPC-UA for an interactive demo—showcased at the **🤖 Automate 2025** exhibition in Detroit.

---

## 📡 What Is OPC-UA?

**OPC Unified Architecture (OPC-UA)** is a versatile, platform-agnostic communication protocol widely used in **🏭 industrial automation**.  
Key attributes:

- 🔒 **Secure by design** – authentication, encryption, and integrity checks.
- 📈 **Scalable and robust** – from embedded devices to enterprise systems.
- 🔄 **Interoperable** – information-model approach for structured data exchange.
- 💻 **Platform-independent** – standardized, service-oriented architecture.

Leveraging OPC-UA in this project allows smooth integration between **Arduino OPTA hardware** and higher-level systems like **Node-RED**, creating a unified, interoperable control and monitoring interface.

---

## 📂 Repository Structure

├── opta_opcua_server/

│ └── Arduino sketch implementing the OPC-UA server on OPTA

└── node-red-opc-ua-client/

└── Node-RED flow implementing the client control panel

### 🔌 `opta_opcua_server/`

Contains the Arduino sketch (C++) that:

- 🖧 Hosts an OPC-UA server on the OPTA platform
- ⚙ Controls relay outputs
- 🔘 Reads digital input states (buttons)
- 🌡 Reads temperature & humidity from a Modbus sensor

### 🖥 `node-red-opc-ua-client/`

Contains the Node-RED flow that:

- 📡 Connects as an OPC-UA client to the Arduino OPTA server
- 🎛 Provides a dashboard to:
  - ⚙ Toggle relays
  - 🔘 Monitor button states
  - 🌡 Display sensor readings in real time

---

## 🚀 Getting Started

1. **📥 Flash** the Arduino OPTA with the OPC-UA server sketch in `opta_opcua_server/`
2. **📤 Import** the Node-RED flow from `node-red-opc-ua-client/`
3. **🔧 Configure** Node-RED to connect to the correct OPC-UA endpoint
4. **🎛 Control & Monitor**: Use the dashboard to toggle relays, read buttons, and check sensor data

---

## 📜 License

Distributed under the **MIT License**. See the [LICENSE](LICENSE) file for details.

---

## 💡 Summary

This repository demonstrates how **OPC-UA** can bridge embedded devices and user-friendly interfaces, with an **Arduino OPTA** and **Node-RED** creating a complete industrial control and monitoring solution.
