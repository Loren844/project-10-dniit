# PBL 6: Automation & Motor Control of the SCARA Robot (TIA Portal)

This document details the "Automation" (PLC) part of the goods sorting project with a SCARA robot.

## Summary of the Architecture

In this project, the Siemens S7-1200 PLC acts as the "muscles" of the system, while the Ubuntu PC (ROS2) acts as the "brain".

1. **ROS2 (Ubuntu)** processes vision (YOLO), computes inverse kinematics (IK), and determines the precise joint angles.
2. **Snap7 (Python)** sends these angle setpoints and the robot state directly into the PLC memory over an Ethernet cable (IP: `192.168.0.10`).
3. **TIA Portal V17 (S7-1200)** reads these setpoints, generates acceleration ramps (S-curves), and sends pulse trains (PTO) to the motor drivers (HB860C) to execute the motion.

---

## 1. Hardware Configuration

The PLC drives the HB860C drivers using its fast outputs via the **PTO (Pulse Train Output)** method in "Pulse A and Direction B" mode.

* **Axis 1 (Shoulder — Rotation):** PTO1 / PWM1 generator
  * Pulse (PU+): `%Q0.0`
  * Direction (DR+): `%Q0.1`

* **Axis 2 (Z — Vertical Translation):** PTO2 / PWM2 generator
  * Pulse (PU+): `%Q0.2`
  * Direction (DR+): `%Q0.3`

* **Axis 3 (Elbow — Rotation):** PTO3 / PWM3 generator
  * Pulse (PU+): `%Q0.4`
  * Direction (DR+): `%Q0.5`

* **Conveyor Belt Motor:** Standard output
  * Command: `%Q0.6` (or another available output)

*(Note: The 24 V PLC signals are connected directly to the HB860C drivers; documentation confirms tolerance up to 28 V on their control inputs.)*

---

## 2. Software Configuration (Technology Objects)

To avoid low-level pulse programming, **3 Technology Objects (TO_PositioningAxis)** have been configured in TIA Portal:

* `Axe_Epaule` (Shoulder Axis): Associated with PTO1. Mechanical unit: **Degrees (°)**.
* `Axe_Z` (Z Axis): Associated with PTO2. Mechanical unit: **Millimetres (mm)**.
* `Axe_Coude` (Elbow Axis): Associated with PTO3. Mechanical unit: **Degrees (°)**.

These objects autonomously manage velocities, accelerations, and mechanical limit enforcement.

---

## 3. Communication Bridge (Data Block — DB1)

`DB1` is the critical exchange point between Python (Snap7) and the S7-1200.
**The order and type of variables are strict** to avoid memory overlap caused by byte-array writes.

| Variable name | Type | Size | Snap7 offset | Role |
| --- | --- | --- | --- | --- |
| `Cible_Theta1` | Real | 4 bytes | 0 | Shoulder target angle computed by ROS2 |
| `Cible_Z` | Real | 4 bytes | 4 | Target height computed by ROS2 |
| `Cible_Theta3` | Real | 4 bytes | 8 | Elbow target angle computed by ROS2 |
| `Etat_Robot` | DInt | 4 bytes | 12 | Cycle phase (0=Tracking, 1=Pick, 2=Transport, etc.) |
| `Ancien_Etat` | DInt | 4 bytes | 16 | Internal memory to detect state changes |
| `Motor_On` | Bool | 1 bit | N/A (Local) | Master enable (triggered by operator) |
| `Go_Movement` | Bool | 1 bit | N/A (Local) | Internal movement execution trigger |
| `Init_Position` | Bool | 1 bit | N/A (Local) | Homing command |
| `Cmd_Tapis` | Bool | 1 bit | N/A (Local) | Conveyor belt enable/disable |

---

## 4. Main Program Logic (OB1 — SCL)

The adopted strategy is **"Stop and Pick"**.

1. The belt brings the object into view.
2. When the AI validates the target (error < 50 mm), the robot state changes.
3. The PLC immediately stops the belt (`Cmd_Tapis`).
4. The PLC generates an extended pulse (`%MW10`) to guarantee triggering of the `MC_MoveAbsolute` blocks.
5. The robot executes its trajectory. Once the object is deposited, the state returns to 0 and the belt restarts.

**SCL code embedded in OB1:**

```pascal
"DB1".Cmd_Tapis := "DB1".Motor_On AND ("DB1".Etat_Robot = 0);

IF "DB1".Etat_Robot <> "DB1".Ancien_Etat THEN
    %MW10 := 250;
    "DB1".Ancien_Etat := "DB1".Etat_Robot;
END_IF;

IF %MW10 > 0 THEN
    "DB1".Go_Movement := 1;
    %MW10 := %MW10 - 1;
ELSE
    "DB1".Go_Movement := 0;
END_IF;

"MC_Power_DB_Epaule"(Axis := "Axe_Epaule", Enable := "DB1".Motor_On, StartMode := 1);
"MC_Power_DB_Z"(Axis := "Axe_Z", Enable := "DB1".Motor_On, StartMode := 1);
"MC_Power_DB_Coude"(Axis := "Axe_Coude", Enable := "DB1".Motor_On, StartMode := 1);

"MC_Home_DB_Epaule"(Axis := "Axe_Epaule", Execute := "DB1".Init_Position, Position := 0.0, Mode := 0);
"MC_Home_DB_Z"(Axis := "Axe_Z", Execute := "DB1".Init_Position, Position := 0.0, Mode := 0);
"MC_Home_DB_Coude"(Axis := "Axe_Coude", Execute := "DB1".Init_Position, Position := 0.0, Mode := 0);

"MC_MoveAbsolute_DB_Epaule"(Axis := "Axe_Epaule", Execute := "DB1".Go_Movement, Position := "DB1".Cible_Theta1, Velocity := 50.0);
"MC_MoveAbsolute_DB_Z"(Axis := "Axe_Z", Execute := "DB1".Go_Movement, Position := "DB1".Cible_Z, Velocity := 20.0);
"MC_MoveAbsolute_DB_Coude"(Axis := "Axe_Coude", Execute := "DB1".Go_Movement, Position := "DB1".Cible_Theta3, Velocity := 50.0);
```

---

## 5. Commissioning Procedure

To start the machine safely (Hardware-in-the-Loop / Physical):

1. Start the ROS2 node on Ubuntu (`ros2 launch scara_visual_servoing gazebo_sim.launch.py` or the hardware launch file). The terminal should display `CONNECTED TO PLC`.
2. Open TIA Portal, go **online**, and open the **Watch table** for DB1.
3. Set `Motor_On` to **1** (the belt starts if no object is under the camera).
4. Set `Init_Position` to **1** then back to **0**. Verify in the diagnostics that all 3 axes are "Homed" (zero point defined).
5. The system is now 100% autonomous.
