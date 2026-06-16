# Demand-Responsive Transport (DRT) Fleet Optimization Engine

An advanced ride-pooling and taxi dispatch simulator built utilizing Python **TraCI** and the **SUMO (Simulation of Urban MObility)** framework.

## 🚀 Core Heuristics Implemented
* **Virtual Waiting Room Buffer:** Manages sudden spikes in passenger demand within a localized local memory cache to guarantee 100% C++ simulation engine uptime.
* **Dynamic Capacity Gatekeeping:** Implements algorithmic workload balancing by capping active itineraries to prevent market hoarding across the fleet.
* **Real Network Physics Costing:** Utilizes real-time routing calculation arrays (`findIntermodalRoute`) to evaluate true travel delays in seconds rather than standard straight-line distance formulas.
* **Eco-Metrics Telemetry Tracker:** Pulls instantaneous vehicle fuel consumption and environmental footprint statistics ($CO_2$) directly out of live runtime steps.

## 🛠️ Setup & Execution
1. Install [SUMO](https://eclipse.dev/sumo/).
2. Set your system environment variable `SUMO_HOME`.
3. Execute the core simulation routing pipeline:
   ```bash
   python run_all.py