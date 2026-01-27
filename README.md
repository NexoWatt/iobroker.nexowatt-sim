# NexoWatt EMS Simulator (Testadapter)

Dieser Adapter stellt **virtuelle Geräte** bereit, damit du dein **NexoWatt EMS** ohne physische Hardware realistisch testen kannst.


**Neu ab v0.3.1:** Die Schnell-Buttons unter `scenario.buttons.*` sind in **Deutsch** benannt (Name-Spalte) und enthalten eine Kurzbeschreibung.
Enthaltene Simulationen:

- **Netz / Hausanschluss** (Limit, Basislast, Import/Export)
- **PV-Anlage** (Tagesprofil, Wetterfaktor)
- **PV-Override** (für Schnelltests: PV-Power unabhängig von Tageszeit erzwingen)
- **Speicher** (200 kW / 200 kWh default, SoC-Physik, Lade-/Entladeleistung)
- **Wärmepumpe**
- **BHKW**
- **Generator**
- **Ladeinfrastruktur (EVCS)**: bis **50 Ladepunkte** (AC 11/22 kW + DC bis 400 kW) inkl. **Vehicle-SoC**, Fahrzeugkapazität, Ziel-SoC, Abfahrtszeit

> Ziel: Dein EMS soll wie im Feld arbeiten – es schreibt Setpoints (z.B. Lade-Limits) und der Simulator reagiert wie echte Geräte.

---

## 1) Quick Start (GitHub → NexoWatt Admin installieren)

1. Repository als ZIP exportieren oder in dein GitHub pushen.
2. Im **NexoWatt Admin** den Adapter/ die App aus GitHub installieren (Custom-URL / Entwickler-Repositorium).
3. Instanz starten.
4. In den States findest du die simulierten Geräte unter:

`nexowatt-sim.<instanz>.…`

---

## 1.1) Test-Szenarien (One-Click)

Für schnelle, reproduzierbare End-to-End-Tests enthält der Adapter **vordefinierte Szenarien**. Diese lassen sich ohne Neustart der Instanz **per Klick** ausführen.

Du findest sie in den States unter:

- `scenario.buttons.*` → **ein Klick = Szenario ausführen** (Timeline-Szenarien starten automatisch)
- `scenario.*` → Status/Monitoring (active, running, phase, elapsed_s, status)
- `scenario.catalog_json` → Liste aller verfügbaren Szenarien (IDs + Beschreibung)

### Bedienung in der Praxis

1. Öffne im Admin die **States** oder **Objekte** der Instanz.
2. Navigiere zu `nexowatt-sim.<instanz>.scenario.buttons`.
3. Setze das gewünschte Button-State kurz auf `true`.

Szenario-Beispiele (Auszug):

- `scenario.buttons.suite_smoke_all` → **Schneller Smoke-Test** (mehrere Stages nacheinander)
- `scenario.buttons.lm_6cars_deadline_0615` → 6 Fahrzeuge, Deadline 06:15
- `scenario.buttons.lm_50ports_deadline_0615` → **alle Ports**, Deadline 06:15
- `scenario.buttons.pv_cloud_ramp_timeline` → PV-„Wolken“ (Override-Rampe)
- `scenario.buttons.tariff_extremes_timeline` → **negative/hohe** Strompreise (Tarif-Extremtest)
- `scenario.buttons.grid_14a_limit_timeline` → Netzlimit-Reduktion (14a-Style)
- `scenario.buttons.faults_evcs_faulted_5` → 5 Ladepunkte **Faulted**
- `scenario.buttons.fuzz_10min_medium` → 10 Minuten „Chaos-Test“ (stabile Reproduzierbarkeit via randomSeed)

Zusätzlich kannst du über `scenario.selected` + `scenario.ctrl.apply/start/stop/reset` steuern.

---

## 2) Standard-Test: „40 kW Hausanschluss, mehrere Ladepunkte, Ziel: 06:15 voll“

### 2.1 Grundparameter setzen

- `grid.limit_kw` → **40**
- `grid.base_load_kw` → z.B. **8**
- (optional) `pv.installed_kwp` → z.B. **200**
- (optional) `storage.soc_pct` → Start-SoC

### 2.2 Fahrzeuge anstecken & Ziele setzen

Beispiel für 6 Fahrzeuge:

- `evcs.c01.ctrl.plugged = true`
- `evcs.c01.vehicle.soc_pct = 20`
- `evcs.c01.vehicle.capacity_kwh = 60`
- `evcs.c01.vehicle.target_soc_pct = 100`
- `evcs.c01.vehicle.departure_time = "06:15"`

Analog für `c02…c06`.

### 2.3 EMS-Regelung testen

Dein EMS schreibt typischerweise:

- `evcs.cXX.ctrl.enabled`
- `evcs.cXX.ctrl.limit_kw` (oder vergleichbare Limits/Setpoints)
- `storage.ctrl.power_set_kw` (Charge/Discharge)
- etc.

Der Simulator setzt dann:

- `evcs.cXX.meas.power_kw`
- `evcs.cXX.vehicle.soc_pct` (steigt)
- `grid.power_kw` (Import/Export)
- `storage.soc_pct`, `storage.power_kw`

---

## 3) State-Struktur (Auszug)

### Netz

- `grid.limit_kw` (rw)
- `grid.base_load_kw` (rw)
- `grid.power_kw` (r) – Netzwirkleistung (+ Import, − Export)
- `grid.over_limit` (r)

### PV

- `pv.installed_kwp` (rw)
- `pv.weather_factor` (rw)
- `pv.power_kw` (r)

PV-Override (für Schnelltests):

- `pv.override.enabled` (rw)
- `pv.override.power_kw` (rw)

### Speicher

- `storage.capacity_kwh` (rw)
- `storage.max_charge_kw` (rw)
- `storage.max_discharge_kw` (rw)
- `storage.soc_pct` (rw) – manuell setzen möglich
- `storage.ctrl.enabled` (rw)
- `storage.ctrl.power_set_kw` (rw) – **+** Entladen / **−** Laden
- `storage.power_kw` (r)

### EVCS

- `evcs.count` (r)
- `evcs.total_power_kw` (r)

Pro Ladepunkt `evcs.cXX…`:

- `meta.max_kw` (r)
- `meta.type` (r) – ac / dc
- `ctrl.enabled` (rw)
- `ctrl.limit_kw` (rw)
- `ctrl.plugged` (rw)
- `vehicle.soc_pct` (rw) – manueller Override möglich
- `vehicle.capacity_kwh` (rw)
- `vehicle.max_charge_kw` (rw)
- `vehicle.target_soc_pct` (rw)
- `vehicle.departure_time` (rw, HH:MM)
- `meas.status` (r)
- `meas.power_kw` (r)
- `meas.energy_kwh` (r)

---

## 4) Hinweise / Design-Entscheidungen

- **Tapering bei DC**: Ab ca. 80% SoC reduziert der Simulator die DC-Ladeleistung (vereinfachtes Modell).
- **Limit-Validierung**: Setpoints werden auf Gerätelimits geklemmt (z.B. 400 kW Charger bleibt 400 kW max).
- **Deterministische Tests**: Über `randomSeed` bleiben „zufällige“ Schwankungen reproduzierbar.

---

## 5) Support / Anpassungen

Wenn du mir sagst, **welche State-Namen** euer EMS exakt erwartet (Prefix/Datapoint-Schema), kann ich einen **Kompatibilitätsmodus** einbauen, der zusätzlich Alias-States in eurem Format erzeugt.
