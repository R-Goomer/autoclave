| Phase / Scenario | Sensor Inputs to Inject | Expected State (`currentState`) | Active Relays (GPIO = LOW / ON) | Inactive Relays (GPIO = HIGH / OFF) |
| --- | --- | --- | --- | --- |
| **1. Standby / Empty** | Water: `< 1850` (e.g., 1778)<br>

<br>Temp: Any<br>

<br>Pressure: Any | `STATE_CHECK_WATER` | *None* | All Relays OFF |
| **2. Cycle Start / Lock** | Water: `> 1850` (e.g., 2500)<br>

<br>Temp: ~25°C<br>

<br>Pressure: ~0 PSI | `STATE_LOCK_DOOR` | **R6: Door Closer** (GPIO 19) | R1, R2, R3, R4, R5 |
| **3. Air Purging** | Water: `> 1850`<br>

<br>Temp: `< 90°C`<br>

<br>Time: `< 30s` | `STATE_PURGE` | **R3: Exhaust Valve** (GPIO 16)<br>

<br>**R5: Heater** (GPIO 18)<br>

<br>**R6: Door Closer** (GPIO 19) | R1, R2, R4 |
| **4. Pressurizing** | Water: `> 1850`<br>

<br>Temp: `90°C – 120°C`<br>

<br>Pressure: `< 15 PSI` | `STATE_HEATING` | **R1: Steam Inlet** (GPIO 7)<br>

<br>**R5: Heater** (GPIO 18)<br>

<br>**R6: Door Closer** (GPIO 19) | R2, R3, R4 |
| **5a. Sterilizing (Heating)** | Water: `> 1850`<br>

<br>Temp: `< 121°C` (e.g., 120.5°C)<br>

<br>Pressure: `≥ 15 PSI` | `STATE_STERILIZING` | **R5: Heater** (GPIO 18)<br>

<br>**R6: Door Closer** (GPIO 19) | R1, R2, R3, R4 |
| **5b. Sterilizing (At Temp)** | Water: `> 1850`<br>

<br>Temp: `≥ 121°C`<br>

<br>Pressure: `≥ 15 PSI` | `STATE_STERILIZING` | **R6: Door Closer** (GPIO 19) | R1, R2, R3, R4, **R5 (Heater cuts off)** |
| **6. Venting / Draining** | Hold time finishes (15m elapsed)<br>

<br>Pressure: `> 1.0 PSI` | `STATE_EXHAUST` | **R2: Drain Valve** (GPIO 15)<br>

<br>**R3: Exhaust Valve** (GPIO 16)<br>

<br>**R6: Door Closer** (GPIO 19) | R1, R4, R5 |
| **7. Cycle Complete** | Pressure: `≤ 1.0 PSI` | `STATE_COMPLETE` $\rightarrow$ `STATE_IDLE` | *None* (Door unlocks) | All Relays OFF |
