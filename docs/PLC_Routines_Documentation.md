# PLC Software Documentation — Fire Suppression Control System (Oil Tank Farm)

**Source:** `all_plc.txt` (aggregated routine listings of six PLCs)
**Scope:** General description of program routines, their functions, key variables, and differences between PLCs. The shared function library (`lib.txt`) is out of scope for this document.

---

## Chapter 1. General System Description

### 1.1 Purpose and Composition

The software runs on six PLCs that jointly control the automatic fire-suppression system of an oil tank farm:

| PLC | Tag | Role | Valves controlled | Other objects |
|-----|-----|------|-------------------|---------------|
| APT 1.1.1 | `APT111` | Valve cabinet, block 1 | 48 (Э-001…Э-048) | 2 discrete signals (door, power input) |
| APT 1.1.2 | `APT112` | Valve cabinet, block 2 | 48 (Э-049…Э-096) | 2 discrete signals |
| APT 1.1.3 | `APT113` | Valve cabinet, block 3 | 48 (Э-097…Э-144) | 2 discrete signals |
| APT 1.1.4 | `APT114` | Valve cabinet, block 4 | 48 (Э-145…Э-192) | 2 discrete signals |
| APT 2.3 | `APT23` | Fire-detection / distribution panel | 11 (Э-193…Э-203) | 47 discretes, 43 fire zones |
| APT 5 | `APT5` | Pump station (ПНС) | 10 (Э-1…Э-10; Э-1…Э-6 via ШУПН) | 7 pumps (incl. БДП), 35 analogs, 3 ШУПН controllers |

Each PLC uses hot-standby CPU redundancy (primary/backup CPU pair, dual PSU modules), which is reflected in every diagnostic section of the code (`__SYSVA_FO_ISPRIMARY`, `__SYSVA_FO_ISACTIVE`).

### 1.2 Communication Architecture

Two independent communication layers are used:

1. **Modbus RTU (serial field buses)** — each valve cabinet talks to COBA electric valve actuators over two serial ports:
   * Port 1: valve actuators (read FC03 registers 40004–40007, write FC16 registers 40002–40003).
   * Port 2: additional valves (APT111–114) or discrete I/O modules / pump controls (APT23, APT5).

2. **Modbus TCP (peer-to-peer PLC network)** — PLCs exchange fire-zone state and valve/pump commands over **two physically separate Ethernet subnets**: `192.168.0.x` (primary) and `192.168.1.x` (backup). Every peer connection is configured as a request pair (primary IP enabled, backup IP disabled). If a request fails continuously for 3 seconds (`Timer < 3000` accumulated with `CYCLE_TIME`), the routine disables the failing request and enables its twin, achieving automatic cable/interface failover. Connection health flags (`CONxxx_yyy`) are published to the HMI.

TCP peer topology:

```
                 APT 2.3 (fire zones source, regs 1001..1143)
                /      |       \        \
               /       |        \        \
        APT111     APT112    APT113    APT114      (chain + hub reads)
            \         |        |        /
             \        |        |       /
              APT 5 ──┴── ШУПН 1/2/3 (192.168.1.125..127, Modbus TCP)
```

* **APT111** ↔ APT5, APT112 (neighbor), APT23 (read fires / write valves).
* **APT112** ↔ APT111, APT113, APT23.
* **APT113** ↔ APT112, APT114, APT23.
* **APT114** ↔ APT113, APT23.
* **APT23** ↔ APT114, APT5 (heartbeat polling only; it is the TCP *server* exposing fire data).
* **APT5** ↔ APT111, APT23 (reads 43 fire states, writes valve/pump states), plus three ШУПН pump-control cabinets as clients.

Shared register map convention between PLCs (holding registers):

| Region | Content |
|--------|---------|
| 1…43 | Fire state per zone (`gFS[i].PRM.FIRE`) |
| 1001+… | Fire matrix published by APT23 (`exchFF`, offset 1001) |
| 1101+… | Valve demand matrix (`exchFV`, offset 1101) |
| 1201+… | Pump-control-cabinet demand matrix (`exchFP`, offset 1201) |
| 2000 | Heartbeat word (single-register read used as link test) |

### 1.3 Common Program Flow

All six PLCs execute the same sequence of routines per scan cycle (IEC 61131-3 Structured Text, one `PROGRAM` per routine, numbered by task order):

| # | Routine | Function |
|---|---------|----------|
| 00 | `FB_HMI_EXCH` | Data-exchange function block invoked twice per cycle (input/output phases) |
| 01 | `_NNN_LinkInput` | Hardware inputs → process image |
| 02 | `_NNN_HMI_in` | Pull operator commands from HMI |
| 03 | `_NNN_Inst` | Instruction handling (local/auto/manual arbitration) |
| 04 | `_NNN_Automatic` | Automatic fire-scenario logic (core application logic) |
| 05 | `_NNN_RTU_Master` | Modbus RTU master — field devices |
| 06 | `_NNN_TCP_Master` | Modbus TCP master — peer PLCs |
| 07 | `_NNN_HMI_out` | Push process image to HMI |
| 08 | `_NNN_LinkOutput` | Process image → hardware outputs |
| 10 | `_NNN_End` | First-cycle termination flag |

(`NNN` = `111`, `112`, `113`, `114`, `23`, `5`.) Note: in APT5 the *Automatic* routine is numbered 03 and *Inst* is 04 — execution order is swapped relative to the other PLCs, but the roles are identical.

### 1.4 Global Data Model

Routines communicate through global structures (declared outside the listing):

* `gVLV[1..n]` — valve objects: `.in.LOCAL`, `.in.REMOTE` (mode feedback), `.in.IS_ON` / `.in.IS_OFF` (end-position state), command bits.
* `gDI[1..n]` — monitored discrete signals: `.IN`, `.MDL` (source-module error tag), `.PRM.PV`.
* `gAI[1..35]` — analog measurements (APT5 only): `.IN`, `.MDL`.
* `gDO[1..n]` — commanded discrete outputs (annunciator lamps/sirens): `.OUT`, `.PRM.PV.0`, `.MDL`.
* `gACT[1..7]` — motor/pump actuators (APT5): `.in.IS_ON`, `.in.IS_OFF`, `.in.FLT`, `.out.TO_ON`.
* `gFS[1..43]` — fire-zone objects: `.PRM.FIRE` (fire flag), `.PRM.VLVS` (valve demand bitmask), `.PRM.PCC` (pump demand bitmask).
* `CONxxx_yyy` — TCP link-health flags per peer.

---

## Chapter 2. Routine Descriptions

### 2.1 FB_HMI_EXCH — HMI/SCADA Exchange Block

**Function.** Maps all global arrays into a fixed HMI memory window using helper calls, so the operator screen always sees a consistent snapshot. It is executed twice per cycle: first from `_NNN_HMI_in` (reading commands) and then from `_NNN_HMI_out` (writing state). Additionally publishes:

* `exchV(i)` — valve i image; `exchD(i)` — discrete i; `exchT(i)` — analog i; `exchP(i)` — pump i; `exchC(i)` — ШУПН i; `exchO(i)` — discrete output i.
* `exchWord(w)` — link-connection flags (`CONxxx_yyy`).
* `exchF(i)`, `exchFF(i)`, `exchFV(i)`, `exchFP(i)` — fire-zone state and inter-PLC matrices (APT23).
* Hardware diagnostics: `exchMdlPSU`, `exchMdlCPU` (with primary/active flags from `__SYSVA_FO_IS*`), `exchMdlDI/DO/AI` per I/O module.
* `gI` (window base index) and toggled heartbeat bit `gD` (`IF First THEN gD := NOT gD`).

**Variables.** Loop counters `i`, `w`; `CPU_pri[]`, `CPU_act[]`; direct references to module tags (`_111_A2_DI`, `_023_G2_1_PSU`, …).

**Differences.**

| PLC | Exchanged objects |
|-----|-------------------|
| APT111–114 | 48 valves, 2 discretes, 2 connection words, full module diagnostics |
| APT23 | 11 valves, 47 discretes, 43 fire zones (`exchF`), 3 interconnection matrices at `gI:=1001/1101/1201` (each 43 registers), 2 connection words |
| APT5 | 10 valves, 3 ШУПН, 7 pumps, 35 analogs, 35 discretes, 3 discrete outputs, 5 connection words, AI-module diagnostics |

### 2.2 Routine 01 — _NNN_LinkInput

**Function.** Direct assignment of physical input-module channels to the process image. Unused channels are assigned to a throwaway variable `_dis`. Module fault tags are captured with `MdlErr()` before reading channels of modules that carry meaningful signals.

**Key variables.** `_111_A2_DI[n].input` style channel tags; targets `gVLV[i].in.LOCAL/REMOTE`, `gDI[i].IN/.MDL`, `gACT[7].in.*`, `gAI[i].IN/.MDL`.

**Differences.**

* **APT111–114** — near-identical: 16 channels per DI module mapped as LOCAL/REMOTE pairs of 8 valves each (modules A2, A3, A6, A7); the tail of A7 carries door state and power-input presence (`gDI[1..2]`).
* **APT23** — mostly unused inputs; LOCAL/REMOTE feedback for valves Э-193…Э-203 comes from module A3, power/door discretes from A3/A4. No valve end-position inputs (positions come over Modbus RTU).
* **APT5** — the richest mapping: level-switch contacts LS01.1/.2/.3 and LS02.1/.2/.3 (min/norm/max in fire-water tanks ПР1/ПР2), БДП run/fault (`gACT[7]`), motor-winding overheat contacts for six pumps (N1.1…N3.2), LOCAL/REMOTE for valves Э-7…Э-10, and 35 raw analog channels across four AI modules (A5–A8).

### 2.3 Routine 02 — _NNN_HMI_in

**Function.** Single call `hmiExch();` — transfers operator commands from the HMI window into globals. **Identical in all six PLCs.**

### 2.4 Routine 03/04 — _NNN_Inst

**Function.** Per-object instruction processing: applies manual/local/auto mode arbitration, edge detection and command latching for each object type via library helpers `_gVLV(i)`, `_gDI(i)`, `_gACT(i)`, `_gAI(i)`, `_gDO(i)`.

**Key variables.** Loop counter `i`; object index ranges.

**Differences.**

| PLC | Objects processed |
|-----|-------------------|
| APT111–114 | `_gVLV` × 48, `_gDI` × 2 |
| APT23 | `_gVLV` × 11, `_gDI` × 47 |
| APT5 | `_gVLV` × 4 (only Э-7…Э-10), `_gACT` × 1 (БДП), `_gAI` × 35, `_gDI` × 35, `_gDO` × 3 |

### 2.5 Routine 04/03 — _NNN_Automatic (Core Logic)

**Function.** Implements the fire scenarios: for each fire zone, a table of valve numbers is loaded into arrays `v[1..10]` (valves to **open**) and `vr[1..4]` (valves to **reverse/close**), then dispatched with `vlv(g := zone, v := v, vr := vr)`, where `g` is the scenario/zone index. Individual valve auto-behaviour is set with `VlvAut(v := n, fs1 := x, fs2 := y)` (valve `n` opens on fire scenario `fs1`, closes on `fs2`).

**Key variables.** `i` (zone index), `v[]`, `vr[]`, `b` (result of `VlvAut`), `gFS[].PRM.*`, `gDO[].PRM.PV.0`, `Timer`.

**Differences.**

* **APT111–114** — eight tank-fire scenarios each ("Пожар на резервуаре N/1…N/8"), with tank numbering offset per block (1/1–1/8, 2/1–2/8, …). Followed by 48 `VlvAut` calls assigning every valve a "normally open on own-tank fire / close on neighbor-tank fire" pair (e.g., `VlvAut(v:=5, fs1:=1, fs2:=5)`).
* **APT23** — first runs the fire-zone processor `_gFS(g := i, fire := i)` for all 43 zones (tanks 1/1–1/32, general fire, tank-farm, ПНС, platforms, trestle positions 3.1-н1…н5, СНТП). Then six local scenarios: pump-station fire operates valve 11; five trestle fires (zones 38–42) open/cross-close valve pairs 1–10 in a chain pattern (segment isolation). Ends with annunciator logic: `gDO[1]` = "fire-protection system fault" from `gDI[47]`.
* **APT5** — no `VlvAut`. Uses `pcc(g, pcc1, pcc2, foam, pmp10, pmp11, pmp20, pmp21)` to select pump-control cabinets (ШУПН 1–3), foam line, and up to four duty/standby pumps per scenario, combined with `vlv()` for local valves. Seven scenarios (general fire, tank fire, ПНС fire, platform fire, trestle fire, СНТП fire). Also drives sound-light annunciators: low fire-water tank level (`gDO[1]`, `gDO[2]`) and a timed horn pulse (`gDO[3]`, `Timer` vs. `30000`/`3000 ms`).

### 2.6 Routine 05 — _NNN_RTU_Master

**Function.** Modbus RTU master for serial field devices. On the first cycle (`IF NOT First OR sInit`) it builds request descriptors: slave ID, function code (FC03 read / FC16 write), start address and length, packed into request groups of 16 (`sMaxReqs := 16`) across up to 8 controllers per port (`_Ctrl_1`, `_Ctrl_2`), with running `sOffset` into holding-register buffers `_HR_1`, `_HR_2`. Every cycle it walks the request list (`sReqNo`, `sCtrlNo` rollover) and decodes results through per-device service blocks: `sCOBA` / `sCOBA_VLV` (valve actuator: status regs 40004–40007, command regs 40002–40003) and `sCOBA_DIS` (multi-channel discrete module). Request status from `_Diag_*[...].status_request` feeds actuator fault state.

**Key variables.** `cfg_SID/FUN/ADR/LEN` (+ `cfg2_*` for the second port), `sMaxReqs`, `sMaxCtrls`, `sOffset`, `sReqNo`, `sCtrlNo`, `_Control_1/_Control_2`, `_HR_1/_HR_2`, `i`, `j`.

**Differences.**

| PLC | Port 1 | Port 2 | Service block |
|-----|--------|--------|---------------|
| APT111–114 | 24 COBA valves | 24 COBA valves (48 total) | `sCOBA` |
| APT23 | 11 COBA valves | 4 × 12-channel COBA discrete modules (last module truncated to 8 channels via `EXIT`) | `sCOBA_VLV`, `sCOBA_DIS` |
| APT5 | 4 COBA valves | 1 × 6-channel COBA discrete module | `sCOBA_VLV`, `sCOBA_DIS` |

Timeout per request: 1000 ms, `repeat_over_scan := TRUE` in all PLCs.

### 2.7 Routine 06 — _NNN_TCP_Master

**Function.** Modbus TCP master for inter-PLC data exchange. First cycle configures request pairs via `rq01…rqNN(enable, IP, port 502, unit, FC, register, length, offset in `_HR`, timeout 1000)` — enabled request on subnet `192.168.0.x`, disabled twin on `192.168.1.x`. Every cycle monitors `_Diag[0].RequestsAndStats[n].status_request`; on persistent failure (>3000 ms accumulated from `CYCLE_TIME`) swaps to the backup path and clears the connection flag `CONxxx_yyy := 0`; success sets `:= 1`. After the transport layer, the routine maps exchanged registers onto the process image.

**Key variables.** `_Ctrl[0].Requests[]`, `_Diag[]`, `Timer[]`, `sCtrlNo`, `sReqNo`, `CONxxx_yyy`, `_HR[0].regs[]`, `gFS[]`.

**Differences.**

| PLC | Peers polled | Data mapping after exchange |
|-----|--------------|------------------------------|
| APT111 | APT5 (heartbeat @2000), APT112 (heartbeat), APT23 (read 43 fires @1001, write 8 valve bytes @1101+8·0) | `gFS[1..8].PRM.FIRE := _HR[i]`; publish `gFS[1..8].PRM.VLVS → _HR[i+100]` |
| APT112 | APT111, APT113 (heartbeats), APT23 (write slot @1101+8·1) | same pattern, own 8-tank slice |
| APT113 | APT112, APT114 (heartbeats), APT23 (write slot @1101+8·2) | same pattern |
| APT114 | APT113 (heartbeat), APT23 (read fires + write slot @1101+8·3) | same pattern |
| APT23 | APT114, APT5 (heartbeats only) | none — APT23 acts as server; matrices exported via `FB_HMI_EXCH` regions 1001/1101/1201 |
| APT5 | APT111 (heartbeat), APT23 (read 43 fires @1001; write valves @1101+32 and pumps @1201+32), ШУПН 1–3 (192.168.1.125/126/127: FC03 12 regs + FC13 1 reg each, no failover pair) | `gFS[1..43].PRM.FIRE`; publishes `gFS[33..40].PRM.VLVS/PCC`; decodes each ШУПН frame (offset 500+j·13) through `_gPCC1` (cabinet state/alarms, `.oStart/.oReset` commands written back), `_gPCC2` × 2 (pumps), `_gPCC3` × 2 (valves Э-1…Э-6), and copies 5 protection-relay bits per cabinet into `gDI[21..35]` |

APT5's TCP routine is by far the largest: single-subnet connections to ШУПН have no redundant twin (flag set directly from request status), unlike the dual-path PLC peers.

### 2.8 Routine 07 — _NNN_HMI_out

**Function.** Single call `hmiExch();` pushing the updated image to the HMI. **Identical in all six PLCs.**

### 2.9 Routine 08 — _NNN_LinkOutput

**Function.** Writes computed states to physical output modules: cabinet position lamps (open/closed LED pairs per valve) and group annunciators. Output-module faults are tagged via `MdlErr()`.

**Key variables.** `_NNN_AX_DO[n].output`, `gVLV[i].in.IS_ON/IS_OFF`, `gDO[i].OUT/.MDL`, `gACT[1].out.TO_ON`, `_dis`.

**Differences.**

| PLC | Outputs |
|-----|---------|
| APT111–114 | 48 lamp pairs across DO modules A4/A5/A8 (2 bits per valve); module A9 fully spare (`_dis`) |
| APT23 | 11 lamp pairs on A5 (note: Э-197…Э-200 pairs are wired reversed — comments swap open/closed); A5 bits 30/31 = "fire-system fault" and "general fault" annunciators (`gDO[1..2]`) |
| APT5 | A4: БДП start command (`gACT[1].out.TO_ON`), three sound-light annunciators HL01–HL03 (`gDO[1..3]`), lamp pairs for valves Э-7…Э-10; remaining channels spare |

### 2.10 Routine 10 — _NNN_End

**Function.** Sets `First := TRUE` at the end of the very first scan, unlocking all "first cycle" initialization branches in other routines. **Identical in all six PLCs.**

---

## Chapter 3. Summary of Inter-PLC Differences

1. **APT111–APT114** are template copies. They differ only in: valve/tank number ranges (Э-001–048, Э-049–096, Э-097–144, Э-145–192; tanks 1/x, 2/x, 3/x, 4/x), TCP neighbors (chain 111↔112↔113↔114 plus hub links), and their write-slot index into APT23's valve matrix (`1101 + 8·block#`).
2. **APT23** is unique as the fire-detection hub: it processes 43 fire zones (`_gFS`), owns the inter-PLC data matrices (fires/valves/pumps at offsets 1001/1101/1201), controls trestle-segmentation and pump-station-fire valve scenarios, and polls discrete COBA modules instead of a second valve bus. Its LinkInput is sparse because most of its signals arrive over the fieldbus.
3. **APT5** deviates most: it adds analog instrumentation (35 channels: tank level/temperature, header pressures, bearing/winding temperatures), pump-group logic (`pcc` scenarios selecting ШУПН cabinets, pumps and foam line), Modbus TCP *client* role toward three ШУПН cabinets, the БДП automatic fire-water pump, timed annunciator logic, and executes Inst after Automatic (routine order 03/04 swapped).
4. All PLCs share the identical skeleton: HMI exchange block, first-cycle configuration pattern guarded by `First`/`sInit`, 3-second dual-network TCP failover, 1000 ms RTU request timeouts, and hot-standby CPU diagnostics.
