'use strict';

/*
 * NexoWatt EMS Simulator (Testadapter)
 * ------------------------------------------------------------
 * Virtual plant that mimics PV, storage, loads and EV chargers.
 * The EMS writes setpoints to *.ctrl.* states; this adapter
 * produces measurements under *.meas.* and other read states.
 */

const utils = require('@iobroker/adapter-core');

function clamp(value, min, max) {
    const v = Number(value);
    if (Number.isNaN(v)) return min;
    return Math.min(max, Math.max(min, v));
}

function pad2(n) {
    return String(n).padStart(2, '0');
}

/**
 * Deterministic pseudo-random generator (LCG).
 * Good enough for reproducible simulation noise.
 */
class LcgRandom {
    constructor(seed = 1) {
        this._state = (Number(seed) >>> 0) || 1;
        this._spareNormal = null;
    }
    next() {
        // Numerical Recipes LCG
        this._state = (1664525 * this._state + 1013904223) >>> 0;
        return this._state / 0x100000000;
    }
    normal(mu = 0, sigma = 1) {
        // Box-Muller with spare
        if (this._spareNormal !== null) {
            const z = this._spareNormal;
            this._spareNormal = null;
            return mu + sigma * z;
        }
        let u = 0;
        let v = 0;
        while (u === 0) u = this.next();
        while (v === 0) v = this.next();
        const mag = Math.sqrt(-2.0 * Math.log(u));
        const z0 = mag * Math.cos(2.0 * Math.PI * v);
        const z1 = mag * Math.sin(2.0 * Math.PI * v);
        this._spareNormal = z1;
        return mu + sigma * z0;
    }
}

function parseDepartureToTimestamp(departureValue, now = new Date()) {
    // Accept "HH:MM" or full ISO timestamp. Returns ms epoch.
    if (typeof departureValue !== 'string') return null;
    const trimmed = departureValue.trim();
    // ISO?
    if (trimmed.includes('T')) {
        const d = new Date(trimmed);
        if (!Number.isNaN(d.getTime())) return d.getTime();
    }
    // HH:MM
    const m = trimmed.match(/^(\d{1,2}):(\d{2})$/);
    if (!m) return null;
    const hh = clamp(m[1], 0, 23);
    const mm = clamp(m[2], 0, 59);

    const candidate = new Date(now);
    candidate.setHours(hh, mm, 0, 0);
    if (candidate.getTime() <= now.getTime()) {
        candidate.setDate(candidate.getDate() + 1);
    }
    return candidate.getTime();
}

function solarProfile01(date) {
    // Very simple day curve 0..1
    const hour = date.getHours() + date.getMinutes() / 60;
    const sunrise = 6.0;
    const sunset = 20.0;
    if (hour <= sunrise || hour >= sunset) return 0;
    const x = (hour - sunrise) / (sunset - sunrise); // 0..1
    // Smooth sine: 0..pi
    return Math.sin(Math.PI * x);
}

function dcTaperFactor(socPct) {
    // Simple taper: 100% power until 80%, then linearly down to 20% at 100%.
    const soc = clamp(socPct, 0, 100);
    if (soc <= 80) return 1;
    const x = (soc - 80) / 20; // 0..1
    return clamp(1 - 0.8 * x, 0.2, 1);
}

class NexowattSimAdapter extends utils.Adapter {
    constructor(options = {}) {
        super({
            ...options,
            name: 'nexowatt-sim',
        });

        this._timer = null;
        this._rng = new LcgRandom(1337);
        this._lastTickMs = null;

        this._cfg = null; // normalized adapter config
        this._scenario = {
            selected: 'baseline',
            active: 'baseline',
            running: false,
            started_ms: 0,
            phase: 'idle',
            status: 'idle',
            last_applied: '',
            data: {},
        };

        this._model = null;
        this._cache = new Map(); // id -> last value for reducing updates

        this.on('ready', this.onReady.bind(this));
        this.on('stateChange', this.onStateChange.bind(this));
        this.on('unload', this.onUnload.bind(this));
    }

    async onReady() {
        try {
            // Normalize config
            const cfg = {
                updateIntervalMs: clamp(this.config.updateIntervalMs ?? 1000, 200, 60000),
                randomSeed: clamp(this.config.randomSeed ?? 1337, 0, 999999),
                gridLimitKw: clamp(this.config.gridLimitKw ?? 40, 1, 2000),
                baseLoadKw: clamp(this.config.baseLoadKw ?? 8, 0, 500),

                pvInstalledKwp: clamp(this.config.pvInstalledKwp ?? 200, 0, 2000),
                pvWeatherFactor: clamp(this.config.pvWeatherFactor ?? 0.85, 0, 1),

                storageCapacityKwh: clamp(this.config.storageCapacityKwh ?? 200, 1, 5000),
                storageMaxChargeKw: clamp(this.config.storageMaxChargeKw ?? 200, 0, 5000),
                storageMaxDischargeKw: clamp(this.config.storageMaxDischargeKw ?? 200, 0, 5000),
                storageInitialSocPct: clamp(this.config.storageInitialSocPct ?? 55, 0, 100),

                chargersCount: clamp(this.config.chargersCount ?? 50, 1, 200),
                autoConnectEnabled: !!this.config.autoConnectEnabled,
                autoConnectCount: clamp(this.config.autoConnectCount ?? 0, 0, 200),
                defaultDepartureTime: (this.config.defaultDepartureTime ?? '06:15').toString(),
                defaultTargetSocPct: clamp(this.config.defaultTargetSocPct ?? 100, 0, 100),
            };

            this._cfg = cfg;

            this._rng = new LcgRandom(cfg.randomSeed);

            this.log.info(`Starting simulator with ${cfg.chargersCount} EVCS, interval=${cfg.updateIntervalMs}ms`);

            await this._createObjects(cfg);
            await this._initModel(cfg);

            // Scenario catalog & defaults
            await this._initScenarioStates();

            // Subscribe to commands and manual overrides
            await this.subscribeStatesAsync('*.ctrl.*');
            await this.subscribeStatesAsync('*.vehicle.*');
            await this.subscribeStatesAsync('grid.*');
            await this.subscribeStatesAsync('pv.*');
            await this.subscribeStatesAsync('storage.*');
            await this.subscribeStatesAsync('tariff.*');
            await this.subscribeStatesAsync('scenario.*');

            // Mark connection
            await this.setStateAsync('info.connection', { val: true, ack: true });

            this._lastTickMs = Date.now();
            this._timer = setInterval(() => { this._tick().catch(err => this.log.error(`tick failed: ${err?.stack || err}`)); }, cfg.updateIntervalMs);
        } catch (err) {
            this.log.error(`onReady failed: ${err?.stack || err}`);
        }
    }

    onUnload(callback) {
        try {
            if (this._timer) clearInterval(this._timer);
            this._timer = null;
            callback();
        } catch (e) {
            callback();
        }
    }

    async onStateChange(id, state) {
        try {
            if (!id || !state) return;
            if (state.ack) return;

            const rel = id.replace(this.namespace + '.', '');

            // Scenario controls are handled asynchronously (buttons, selection)
            if (rel.startsWith('scenario.')) {
                await this._handleScenarioStateWrite(rel, state.val);
                return;
            }

            // Apply and ACK
            const applied = this._applyWrite(rel, state.val);
            if (applied) {
                await this.setStateAsync(rel, { val: state.val, ack: true });
            } else {
                // If not applicable, still ACK to avoid stuck commands
                await this.setStateAsync(rel, { val: state.val, ack: true });
            }
        } catch (err) {
            this.log.warn(`stateChange handler error: ${err?.message || err}`);
        }
    }

    _applyWrite(relId, value) {
        if (!this._model) return false;

        // Grid
        if (relId === 'grid.limit_kw') {
            this._model.grid.limit_kw = clamp(value, 1, 5000);
            return true;
        }
        if (relId === 'grid.base_load_kw') {
            this._model.grid.base_load_kw = clamp(value, 0, 5000);
            return true;
        }
        if (relId === 'grid.available') {
            this._model.grid.available = !!value;
            return true;
        }

        // Tariff
        if (relId === 'tariff.mode') {
            const v = String(value || '').toLowerCase();
            this._model.tariff.mode = (v === 'manual') ? 'manual' : 'auto';
            return true;
        }
        if (relId === 'tariff.price_ct_per_kwh') {
            this._model.tariff.price_ct_per_kwh = clamp(value, -500, 500);
            return true;
        }

        // PV
        if (relId === 'pv.installed_kwp') {
            this._model.pv.installed_kwp = clamp(value, 0, 5000);
            return true;
        }
        if (relId === 'pv.weather_factor') {
            this._model.pv.weather_factor = clamp(value, 0, 1);
            return true;
        }
        if (relId === 'pv.override.enabled') {
            this._model.pv.override.enabled = !!value;
            return true;
        }
        if (relId === 'pv.override.power_kw') {
            this._model.pv.override.power_kw = clamp(value, 0, 100000);
            return true;
        }

        // Storage
        if (relId === 'storage.capacity_kwh') {
            this._model.storage.capacity_kwh = clamp(value, 1, 50000);
            return true;
        }
        if (relId === 'storage.max_charge_kw') {
            this._model.storage.max_charge_kw = clamp(value, 0, 50000);
            return true;
        }
        if (relId === 'storage.max_discharge_kw') {
            this._model.storage.max_discharge_kw = clamp(value, 0, 50000);
            return true;
        }
        if (relId === 'storage.soc_pct') {
            this._model.storage.soc_pct = clamp(value, 0, 100);
            return true;
        }
        if (relId === 'storage.ctrl.enabled') {
            this._model.storage.ctrl.enabled = !!value;
            return true;
        }
        if (relId === 'storage.ctrl.power_set_kw') {
            this._model.storage.ctrl.power_set_kw = Number(value) || 0;
            return true;
        }

        // Heatpump / CHP / Generator
        for (const dev of ['heatpump', 'chp', 'generator']) {
            if (relId === `${dev}.ctrl.enabled`) {
                this._model[dev].ctrl.enabled = !!value;
                return true;
            }
            if (relId === `${dev}.ctrl.power_set_kw`) {
                this._model[dev].ctrl.power_set_kw = Number(value) || 0;
                return true;
            }
        }

        // EVCS per charger
        const m = relId.match(/^evcs\.(c\d{2})\.(.+)$/);
        if (m) {
            const cid = m[1];
            const tail = m[2];
            const charger = this._model.evcs.chargers.find(c => c.id === cid);
            if (!charger) return false;

            // Controls
            if (tail === 'ctrl.enabled') {
                charger.ctrl.enabled = !!value;
                return true;
            }
            if (tail === 'ctrl.limit_kw') {
                charger.ctrl.limit_kw = clamp(value, 0, 10000);
                return true;
            }
            if (tail === 'ctrl.plugged') {
                charger.ctrl.plugged = !!value;
                if (!charger.ctrl.plugged) {
                    // Reset session on unplug
                    charger.meas.power_kw = 0;
                    charger.meas.energy_kwh = 0;
                    charger.meas.status = 'Available';
                }
                return true;
            }
            if (tail === 'ctrl.priority') {
                charger.ctrl.priority = clamp(value, 1, 10);
                return true;
            }

            // Vehicle
            if (tail === 'vehicle.soc_pct') {
                charger.vehicle.soc_pct = clamp(value, 0, 100);
                return true;
            }
            if (tail === 'vehicle.capacity_kwh') {
                charger.vehicle.capacity_kwh = clamp(value, 1, 1000);
                return true;
            }
            if (tail === 'vehicle.max_charge_kw') {
                charger.vehicle.max_charge_kw = clamp(value, 0, 2000);
                return true;
            }
            if (tail === 'vehicle.target_soc_pct') {
                charger.vehicle.target_soc_pct = clamp(value, 0, 100);
                return true;
            }
            if (tail === 'vehicle.departure_time') {
                charger.vehicle.departure_time = String(value || '').trim();
                charger.vehicle.departure_ts = parseDepartureToTimestamp(charger.vehicle.departure_time, new Date());
                return true;
            }
            if (tail === 'vehicle.id') {
                charger.vehicle.id = String(value || '').trim();
                return true;
            }

            // Command-like
            if (tail === 'ctrl.reset_session') {
                if (value === true || value === 1 || value === 'true') {
                    charger.meas.energy_kwh = 0;
                    charger.meas.status = charger.ctrl.plugged ? 'Preparing' : 'Available';
                }
                return true;
            }
        }

        return false;
    }

    // ---------------------------------------------------------------------
    // Scenario / Test modes
    // ---------------------------------------------------------------------
    _scenarioDefinitions() {
        // Full scenario catalog (One-Click in Admin via scenario.buttons.*)
        return [
            // --- Suites / automation ---
            {
                id: 'suite_smoke_all',
                title: 'Suite: Smoke Test (Quick)',
                kind: 'timeline',
                duration_s: 420,
                description: 'Runs a compact end-to-end smoke suite: baseline → LM6 deadline → tariff pulse → grid limit drop → PV surplus → DC rush → fault injection → done.',
            },
            {
                id: 'suite_full_all',
                title: 'Suite: Full Test (Long)',
                kind: 'timeline',
                duration_s: 1800,
                description: 'Runs a longer end-to-end suite with arrival/departure waves, PV clouds, tariff extremes and random disturbances (fuzz).',
            },

            // --- Baseline ---
            {
                id: 'baseline',
                title: 'Baseline / Reset',
                kind: 'oneshot',
                description: 'Resets the simulator to adapter defaults (grid/PV/storage) and clears EV sessions and fault flags.',
            },

            // --- Load management / deadlines ---
            {
                id: 'lm_6cars_deadline_0615',
                title: 'Load Mgmt: 6 Cars, Deadline 06:15',
                kind: 'oneshot',
                description: '40 kW grid, 6 sessions with different SoC, target 100%, departure 06:15. PV off (night).',
            },
            {
                id: 'lm_20mix_deadline_0615',
                title: 'Load Mgmt: 20 Mixed Chargers, Deadline 06:15',
                kind: 'oneshot',
                description: '40 kW grid, 20 sessions across AC 11/22 kW and DC, target 100%, departure 06:15. PV off.',
            },
            {
                id: 'lm_50ports_deadline_0615',
                title: 'Load Mgmt: Full Site (all ports), Deadline 06:15',
                kind: 'oneshot',
                description: '40 kW grid, plugs ALL available charge points with random SoC/capacity. Target 100%, departure 06:15. PV off.',
            },
            {
                id: 'lm_priorities_3tiers',
                title: 'Load Mgmt: Priorities 3 Tiers',
                kind: 'oneshot',
                description: 'Plugs 15 sessions with three priority tiers (10/7/4). Useful to verify priority handling in scarcity.',
            },
            {
                id: 'lm_arrival_wave_timeline',
                title: 'Timeline: Arrival Wave (vehicles arrive gradually)',
                kind: 'timeline',
                duration_s: 600,
                description: 'Starts empty and plugs one new vehicle every 10s (up to 20). Helps test dynamic allocation as new sessions appear.',
            },
            {
                id: 'lm_departure_wave_timeline',
                title: 'Timeline: Departure Wave (vehicles leave gradually)',
                kind: 'timeline',
                duration_s: 600,
                description: 'Starts with 20 sessions and unplugs vehicles over time. Helps test redistribution when sessions complete/leave.',
            },

            // --- PV / storage / surplus ---
            {
                id: 'pv_surplus_30cars',
                title: 'PV Surplus: 30 Cars + Forced PV Power',
                kind: 'oneshot',
                description: 'Forces PV power (override) to create surplus. 30 sessions, target 100%, departure 17:00.',
            },
            {
                id: 'pv_cloud_ramp_timeline',
                title: 'Timeline: PV Clouds / Ramp',
                kind: 'timeline',
                duration_s: 600,
                description: 'Forces PV override and modulates PV power up/down (clouds). Ideal to test PV-following behaviour.',
            },
            {
                id: 'storage_soc0',
                title: 'Storage Edge: SoC = 0%',
                kind: 'oneshot',
                description: 'Sets storage SoC to 0% (empty). Useful to test discharge limits and EMS fallback behaviour.',
            },
            {
                id: 'storage_soc100',
                title: 'Storage Edge: SoC = 100%',
                kind: 'oneshot',
                description: 'Sets storage SoC to 100% (full). Useful to test charge limits and EMS fallback behaviour.',
            },
            {
                id: 'storage_power_limit_low',
                title: 'Storage Limit: Low Power (±20 kW)',
                kind: 'oneshot',
                description: 'Constrains storage max charge/discharge to ±20 kW. Helps validate EMS under weak storage power.',
            },

            // --- Tariff / price optimisation ---
            {
                id: 'tariff_flat_low_10ct',
                title: 'Tariff: Manual Flat 10 ct/kWh',
                kind: 'oneshot',
                description: 'Switches to manual tariff and sets a flat price of 10 ct/kWh.',
            },
            {
                id: 'tariff_pulse_timeline',
                title: 'Timeline: Tariff Pulse (low/high/low)',
                kind: 'timeline',
                duration_s: 180,
                description: 'Manual price: 10 ct/kWh (0-60s), 120 ct/kWh (60-120s), 10 ct/kWh (120-180s).',
            },
            {
                id: 'tariff_extremes_timeline',
                title: 'Timeline: Tariff Extremes (negative / very high)',
                kind: 'timeline',
                duration_s: 240,
                description: 'Manual price: -20 (0-80s), +200 (80-160s), +30 (160-240s). Good for stress-testing tariff logic.',
            },

            // --- Grid events / constraints ---
            {
                id: 'grid_limit_drop_timeline',
                title: 'Timeline: Grid Limit Drop (80 -> 25 kW)',
                kind: 'timeline',
                duration_s: 240,
                description: 'Starts at 80 kW, after 60 s drops to 25 kW for 120 s, then restores to 80 kW.',
            },
            {
                id: 'grid_14a_limit_timeline',
                title: 'Timeline: Grid Limit (14a style)',
                kind: 'timeline',
                duration_s: 600,
                description: 'Simulates a curtailment event: 0-60s normal, 60-300s hard reduced grid limit, then recovery.',
            },
            {
                id: 'base_load_spike_timeline',
                title: 'Timeline: Base Load Spikes',
                kind: 'timeline',
                duration_s: 300,
                description: 'Simulates sudden site base load spikes (e.g., other consumers). Helps validate peak-shaving and robustness.',
            },
            {
                id: 'grid_blackout_60s_timeline',
                title: 'Timeline: Grid Blackout 60s',
                kind: 'timeline',
                duration_s: 150,
                description: 'Grid available=false from 30s to 90s, then recovers. Useful for failover tests.',
            },

            // --- DC-specific ---
            {
                id: 'dc_rush_10',
                title: 'DC Stress: 10 DC Sessions (Fast-Charge Rush)',
                kind: 'oneshot',
                description: 'Plugs up to 10 DC charge points, low SoC, target 80%. High grid limit for stress testing.',
            },
            {
                id: 'dc_taper_single_400',
                title: 'DC Taper Test: Single 400 kW',
                kind: 'oneshot',
                description: 'Plugs one 400 kW DC charger at ~75% SoC so you can observe tapering beyond 80%.',
            },
            {
                id: 'dc_all_ports_stress',
                title: 'DC Stress: All DC Ports',
                kind: 'oneshot',
                description: 'Plugs ALL DC charge points at low SoC (target 80%) with a high grid limit. Heavy stress test.',
            },

            // --- Fault injection ---
            {
                id: 'faults_evcs_faulted_5',
                title: 'Faults: 5 EVCS Faulted',
                kind: 'oneshot',
                description: 'Marks 5 random charge points as faulted (status=Faulted, power=0).',
            },
            {
                id: 'faults_evcs_unavailable_5',
                title: 'Faults: 5 EVCS Unavailable',
                kind: 'oneshot',
                description: 'Marks 5 random charge points as unavailable/offline (status=Unavailable, power=0).',
            },
            {
                id: 'faults_evcs_meter_freeze_3',
                title: 'Faults: 3 EVCS Meter Freeze',
                kind: 'oneshot',
                description: 'Freezes meas.* + vehicle.soc of 3 random charge points (stuck meter simulation).',
            },
            {
                id: 'faults_clear_all',
                title: 'Faults: Clear All',
                kind: 'oneshot',
                description: 'Clears all EVCS fault/offline/meter-freeze flags.',
            },

            // --- Randomised disturbances ---
            {
                id: 'fuzz_10min_medium',
                title: 'Fuzz: 10 min Medium Disturbance',
                kind: 'timeline',
                duration_s: 600,
                description: 'Random plug/unplug, PV/tariff/grid changes and occasional faults. Deterministic via randomSeed.',
            },
            {
                id: 'fuzz_30min_heavy',
                title: 'Fuzz: 30 min Heavy Disturbance',
                kind: 'timeline',
                duration_s: 1800,
                description: 'More frequent random events and faults. Intended to find edge-case logic bugs and stability issues.',
            },

            // --- External loads / generators (one-shot starting points) ---
            {
                id: 'ext_heatpump_30kw',
                title: 'External Load: Heatpump 30 kW',
                kind: 'oneshot',
                description: 'Enables heatpump and sets power_set=30 kW (as an external site consumer).',
            },
            {
                id: 'ext_chp_80kw',
                title: 'External Generation: CHP/BHKW 80 kW',
                kind: 'oneshot',
                description: 'Enables CHP/BHKW and sets power_set=80 kW (as an external generator).',
            },
            {
                id: 'ext_generator_200kw',
                title: 'External Generation: Generator 200 kW',
                kind: 'oneshot',
                description: 'Enables generator and sets power_set=200 kW (as an external generator).',
            },
        ];
    }

    _normalizeScenarioId(value) {
        const raw = String(value || '').trim();
        if (!raw) return 'baseline';
        const id = raw.replace(/\s+/g, '_');
        const known = new Set(this._scenarioDefinitions().map(s => s.id));
        return known.has(id) ? id : 'baseline';
    }

    async _initScenarioStates() {
        try {
            const sel = await this.getStateAsync('scenario.selected');
            this._scenario.selected = this._normalizeScenarioId(sel?.val ?? 'baseline');
            this._scenario.active = this._scenario.selected;
            this._scenario.running = false;
            this._scenario.started_ms = 0;
            this._scenario.phase = 'idle';
            this._scenario.status = 'idle';

            const catalog = this._scenarioDefinitions();
            await this.setStateAsync('scenario.catalog_json', { val: JSON.stringify(catalog, null, 2), ack: true });
            await this.setStateAsync('scenario.selected', { val: this._scenario.selected, ack: true });

            await this._publishScenarioStates();
        } catch (err) {
            this.log.warn(`initScenarioStates failed: ${err?.message || err}`);
        }
    }

    async _handleScenarioStateWrite(relId, value) {
        try {
            if (relId === 'scenario.selected') {
                const normalized = this._normalizeScenarioId(value);
                this._scenario.selected = normalized;
                await this.setStateAsync(relId, { val: normalized, ack: true });
                this._scenario.status = `selected=${normalized}`;
                await this._publishScenarioStates();
                return;
            }

            // Quick scenario button: one click triggers the respective scenario
            if (relId.startsWith('scenario.buttons.')) {
                const isPressed = value === true || value === 1 || value === 'true';
                await this.setStateAsync(relId, { val: value, ack: true });
                if (!isPressed) return;

                const scenarioId = this._normalizeScenarioId(relId.substring('scenario.buttons.'.length));
                const def = this._scenarioDefinitions().find(s => s.id === scenarioId);
                const start = def?.kind === 'timeline';
                await this._applyScenario(scenarioId, { start });

                // Auto-reset button to false so it can be clicked again
                await this.setStateAsync(relId, { val: false, ack: true });
                return;
            }

            // Button handling (momentary)
            const isPressed = value === true || value === 1 || value === 'true';
            await this.setStateAsync(relId, { val: value, ack: true });
            if (!isPressed) return;

            if (relId === 'scenario.ctrl.apply') {
                await this._applyScenario(this._scenario.selected, { start: false });
            } else if (relId === 'scenario.ctrl.start') {
                await this._applyScenario(this._scenario.selected, { start: true });
            } else if (relId === 'scenario.ctrl.stop') {
                await this._stopScenario('user');
            } else if (relId === 'scenario.ctrl.reset') {
                await this._applyScenario('baseline', { start: false });
            } else {
                // unknown scenario control -> just ack
            }

            // Auto-reset button to false so it can be clicked again
            await this.setStateAsync(relId, { val: false, ack: true });
        } catch (err) {
            this.log.warn(`scenario write handler error (${relId}): ${err?.message || err}`);
        }
    }

    async _publishScenarioStates() {
        const elapsed = (this._scenario.running && this._scenario.started_ms) ? ((Date.now() - this._scenario.started_ms) / 1000) : 0;
        await this._setChanged('scenario.selected', this._scenario.selected);
        await this._setChanged('scenario.active', this._scenario.active);
        await this._setChanged('scenario.running', this._scenario.running);
        await this._setChanged('scenario.phase', this._scenario.phase);
        await this._setChanged('scenario.elapsed_s', Number(elapsed.toFixed(1)));
        await this._setChanged('scenario.status', this._scenario.status);
    }

    async _applyScenario(scenarioId, { start } = { start: false }) {
        if (!this._model || !this._cfg) {
            this._scenario.status = 'model not ready';
            await this._publishScenarioStates();
            return;
        }

        const id = this._normalizeScenarioId(scenarioId);
        const nowMs = Date.now();

        // Keep selection in sync with the last applied scenario (helps in Admin)
        this._scenario.selected = id;

        // Stop any running scenario first
        this._scenario.running = false;
        this._scenario.started_ms = 0;
        this._scenario.phase = 'applying';
        this._scenario.status = `applying ${id}`;

        // Reset per-scenario runtime data
       switch (id) {
            case 'baseline':
                this._resetToDefaultsModel();
                break;

            // Suites
            case 'suite_smoke_all':
                this._scenarioSuiteSmokeSetup();
                break;
            case 'suite_full_all':
                this._scenarioSuiteFullSetup();
                break;

            // Load management
            case 'lm_6cars_deadline_0615':
                this._scenarioLm6CarsDeadline();
                break;
            case 'lm_20mix_deadline_0615':
                this._scenarioLm20MixedDeadline();
                break;
            case 'lm_50ports_deadline_0615':
                this._scenarioLm50PortsDeadline();
                break;
            case 'lm_priorities_3tiers':
                this._scenarioLmPriorities3tiers();
                break;
            case 'lm_arrival_wave_timeline':
                this._scenarioLmArrivalWaveSetup();
                break;
            case 'lm_departure_wave_timeline':
                this._scenarioLmDepartureWaveSetup();
                break;

            // PV / storage
            case 'pv_surplus_30cars':
                this._scenarioPvSurplus30Cars();
                break;
            case 'pv_cloud_ramp_timeline':
                this._scenarioPvCloudRampSetup();
                break;
            case 'storage_soc0':
                this._scenarioStorageSoc(0);
                break;
            case 'storage_soc100':
                this._scenarioStorageSoc(100);
                break;
            case 'storage_power_limit_low':
                this._scenarioStoragePowerLimitLow();
                break;

            // Tariff
            case 'tariff_flat_low_10ct':
                this._scenarioTariffFlatLow10ct();
                break;
            case 'tariff_pulse_timeline':
                this._scenarioTariffPulseTimelineSetup();
                break;
            case 'tariff_extremes_timeline':
                this._scenarioTariffExtremesTimelineSetup();
                break;

            // Grid events
            case 'grid_limit_drop_timeline':
                this._scenarioGridLimitDropTimelineSetup();
                break;
            case 'grid_14a_limit_timeline':
                this._scenarioGrid14aLimitTimelineSetup();
                break;
            case 'base_load_spike_timeline':
                this._scenarioBaseLoadSpikeTimelineSetup();
                break;
            case 'grid_blackout_60s_timeline':
                this._scenarioGridBlackoutTimelineSetup();
                break;

            // DC
            case 'dc_rush_10':
                this._scenarioDcRush10();
                break;
            case 'dc_taper_single_400':
                this._scenarioDcTaperSingle400();
                break;
            case 'dc_all_ports_stress':
                this._scenarioDcAllPortsStress();
                break;

            // Fault injection
            case 'faults_evcs_faulted_5':
                this._scenarioFaultsEvcsFaulted(5);
                break;
            case 'faults_evcs_unavailable_5':
                this._scenarioFaultsEvcsUnavailable(5);
                break;
            case 'faults_evcs_meter_freeze_3':
                this._scenarioFaultsEvcsMeterFreeze(3);
                break;
            case 'faults_clear_all':
                this._scenarioFaultsClearAll();
                break;

            // Random/fuzz
            case 'fuzz_10min_medium':
                this._scenarioFuzzSetup({ intensity: 'medium', duration_s: 600 });
                break;
            case 'fuzz_30min_heavy':
                this._scenarioFuzzSetup({ intensity: 'heavy', duration_s: 1800 });
                break;

            // External loads/generation
            case 'ext_heatpump_30kw':
                this._scenarioExternalHeatpump(30);
                break;
            case 'ext_chp_80kw':
                this._scenarioExternalChp(80);
                break;
            case 'ext_generator_200kw':
                this._scenarioExternalGenerator(200);
                break;

            default:
                this._resetToDefaultsModel();
                break;
        }

        // Activate
        this._scenario.active = id;
        this._scenario.last_applied = new Date(nowMs).toISOString();

        if (start) {
            this._scenario.running = true;
            this._scenario.started_ms = nowMs;
            this._scenario.phase = 'running';
            this._scenario.status = `running ${id}`;
        } else {
            this._scenario.running = false;
            this._scenario.started_ms = 0;
            this._scenario.phase = 'applied';
            this._scenario.status = `applied ${id}`;
        }

        await this._publishScenarioStates();
    }

    async _stopScenario(reason = 'user') {
        this._scenario.running = false;
        this._scenario.started_ms = 0;
        this._scenario.phase = 'stopped';
        this._scenario.status = `stopped (${reason})`;
        await this._publishScenarioStates();
    }

    _resetToDefaultsModel() {
        const cfg = this._cfg;
        if (!cfg || !this._model) return;

        // Grid
        this._model.grid.available = true;
        this._model.grid.limit_kw = cfg.gridLimitKw;
        this._model.grid.base_load_kw = cfg.baseLoadKw;

        // Tariff
        this._model.tariff.mode = 'auto';
        // Keep current price as-is; tick will update

        // PV
        this._model.pv.installed_kwp = cfg.pvInstalledKwp;
        this._model.pv.weather_factor = cfg.pvWeatherFactor;
        this._model.pv.override.enabled = false;
        this._model.pv.override.power_kw = 0;

        // Storage
        this._model.storage.capacity_kwh = cfg.storageCapacityKwh;
        this._model.storage.max_charge_kw = cfg.storageMaxChargeKw;
        this._model.storage.max_discharge_kw = cfg.storageMaxDischargeKw;
        this._model.storage.soc_pct = cfg.storageInitialSocPct;
        this._model.storage.ctrl.enabled = true;
        this._model.storage.ctrl.power_set_kw = 0;

        // Flex devices off
        for (const dev of ['heatpump', 'chp', 'generator']) {
            this._model[dev].ctrl.enabled = false;
            this._model[dev].ctrl.power_set_kw = 0;
        }

        // EVCS reset
        const now = new Date();
        for (let i = 0; i < this._model.evcs.chargers.length; i++) {
            const ch = this._model.evcs.chargers[i];
            const autoPlug = cfg.autoConnectEnabled && (i + 1) <= cfg.autoConnectCount;

            ch.ctrl.plugged = autoPlug;
            ch.ctrl.enabled = autoPlug;
            ch.ctrl.limit_kw = ch.max_kw;
            ch.ctrl.priority = 5;

            ch.vehicle.soc_pct = clamp(10 + 60 * this._rng.next(), 0, 100);
            ch.vehicle.capacity_kwh = clamp(40 + 50 * this._rng.next(), 1, 1000);
            ch.vehicle.max_charge_kw = ch.max_kw;
            ch.vehicle.target_soc_pct = cfg.defaultTargetSocPct;
            ch.vehicle.departure_time = cfg.defaultDepartureTime;
            ch.vehicle.departure_ts = parseDepartureToTimestamp(ch.vehicle.departure_time, now);

            ch.meas.power_kw = 0;
            ch.meas.energy_kwh = 0;
            ch.meas.status = autoPlug ? 'Preparing' : 'Available';

            // Clear simulation fault flags
            if (ch.sim) {
                ch.sim.faulted = false;
                ch.sim.unavailable = false;
                ch.sim.meter_freeze = false;
            }
        }
    }

    _plugCharger(id, { soc_pct, capacity_kwh, target_soc_pct, departure_time, priority } = {}) {
        const ch = this._model.evcs.chargers.find(c => c.id === id);
        if (!ch) return;
        const now = new Date();
        ch.ctrl.plugged = true;
        ch.ctrl.enabled = true;
        ch.ctrl.limit_kw = ch.max_kw;
        if (priority !== undefined) ch.ctrl.priority = clamp(priority, 1, 10);

        if (soc_pct !== undefined) ch.vehicle.soc_pct = clamp(soc_pct, 0, 100);
        if (capacity_kwh !== undefined) ch.vehicle.capacity_kwh = clamp(capacity_kwh, 1, 1000);
        ch.vehicle.max_charge_kw = ch.max_kw;
        if (target_soc_pct !== undefined) ch.vehicle.target_soc_pct = clamp(target_soc_pct, 0, 100);
        if (departure_time !== undefined) ch.vehicle.departure_time = String(departure_time);
        ch.vehicle.departure_ts = parseDepartureToTimestamp(ch.vehicle.departure_time, now);

        ch.meas.power_kw = 0;
        ch.meas.energy_kwh = 0;
        ch.meas.status = 'Preparing';

        // When explicitly plugging a charger for a scenario, clear fault flags
        if (ch.sim) {
            ch.sim.faulted = false;
            ch.sim.unavailable = false;
            ch.sim.meter_freeze = false;
        }
    }

    _unplugAllChargers() {
        for (const ch of this._model.evcs.chargers) {
            ch.ctrl.plugged = false;
            ch.ctrl.enabled = false;
            ch.meas.power_kw = 0;
            ch.meas.energy_kwh = 0;
            ch.meas.status = 'Available';
        }
    }

    _scenarioLm6CarsDeadline() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();

        // Grid constraint (example: 40 kW house connection)
        this._model.grid.limit_kw = 40;
        this._model.grid.base_load_kw = 8;
        this._model.grid.available = true;

        // Night: PV off
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        // Storage default size
        this._model.storage.capacity_kwh = 200;
        this._model.storage.max_charge_kw = 200;
        this._model.storage.max_discharge_kw = 200;
        this._model.storage.soc_pct = 60;
        this._model.storage.ctrl.enabled = true;
        this._model.storage.ctrl.power_set_kw = 0;

        const ids = ['c01', 'c02', 'c03', 'c04', 'c05', 'c06'];
        const socs = [20, 35, 10, 50, 25, 15];
        for (let i = 0; i < ids.length; i++) {
            if (i >= this._model.evcs.chargers.length) break;
            this._plugCharger(ids[i], {
                soc_pct: socs[i] ?? 20,
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
                priority: i === 0 ? 10 : 5,
            });
        }
    }

    _scenarioLm20MixedDeadline() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();

        this._model.grid.limit_kw = 40;
        this._model.grid.base_load_kw = 10;
        this._model.grid.available = true;

        // Night: PV off
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        // Mix across AC11 (c01..), AC22 (c26..), DC (c41..)
        const ids = [];
        for (let i = 1; i <= 10; i++) ids.push(`c${pad2(i)}`); // AC11
        for (let i = 26; i <= 30; i++) ids.push(`c${pad2(i)}`); // AC22
        for (let i = 41; i <= 45; i++) ids.push(`c${pad2(i)}`); // DC mix

        const list = ids.filter(cid => {
            const num = Number(cid.slice(1));
            return num >= 1 && num <= this._model.evcs.chargers.length;
        });

        for (let i = 0; i < list.length; i++) {
            const soc = clamp(10 + 60 * this._rng.next(), 0, 100);
            const cap = clamp(50 + 40 * this._rng.next(), 1, 1000);
            this._plugCharger(list[i], {
                soc_pct: soc,
                capacity_kwh: cap,
                target_soc_pct: 100,
                departure_time: '06:15',
                priority: i < 3 ? 9 : 5,
            });
        }
    }

    _scenarioPvSurplus30Cars() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();

        // More headroom
        this._model.grid.limit_kw = 200;
        this._model.grid.base_load_kw = 30;

        // Force PV power for quick tests independent of actual time
        this._model.pv.installed_kwp = Math.max(600, this._model.pv.installed_kwp);
        this._model.pv.override.enabled = true;
        this._model.pv.override.power_kw = 450;

        // 30 sessions (take first 30 charge points)
        const count = Math.min(30, this._model.evcs.chargers.length);
        for (let i = 1; i <= count; i++) {
            const cid = `c${pad2(i)}`;
            this._plugCharger(cid, {
                soc_pct: clamp(10 + 50 * this._rng.next(), 0, 100),
                capacity_kwh: clamp(50 + 40 * this._rng.next(), 1, 1000),
                target_soc_pct: 100,
                departure_time: '17:00',
                priority: 5,
            });
        }

        // Cheap price (optional)
        this._model.tariff.mode = 'manual';
        this._model.tariff.price_ct_per_kwh = 10;
    }

    _scenarioGridLimitDropTimelineSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._model.grid.limit_kw = 80;
        this._model.grid.base_load_kw = 10;
        this._model.grid.available = true;

        // Plug 12 sessions
        const count = Math.min(12, this._model.evcs.chargers.length);
        for (let i = 1; i <= count; i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 60 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
            });
        }
    }

    _scenarioTariffPulseTimelineSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._model.grid.limit_kw = 60;
        this._model.grid.base_load_kw = 8;

        // Plug 6 sessions
        for (let i = 1; i <= Math.min(6, this._model.evcs.chargers.length); i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 40 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
            });
        }

        // Manual tariff so we can pulse price in the timeline
        this._model.tariff.mode = 'manual';
        this._model.tariff.price_ct_per_kwh = 10;
    }

    _scenarioGridBlackoutTimelineSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._model.grid.limit_kw = 80;
        this._model.grid.base_load_kw = 10;
        this._model.grid.available = true;

        // Plug 8 sessions
        for (let i = 1; i <= Math.min(8, this._model.evcs.chargers.length); i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 40 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
            });
        }
    }

    _scenarioDcRush10() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();

        this._model.grid.limit_kw = 1000;
        this._model.grid.base_load_kw = 50;
        this._model.pv.override.enabled = false;
        this._model.pv.installed_kwp = 0;

        // Plug up to 10 DC chargers
        const dcChargers = this._model.evcs.chargers.filter(c => c.type === 'dc');
        const count = Math.min(10, dcChargers.length);
        for (let i = 0; i < count; i++) {
            const ch = dcChargers[i];
            this._plugCharger(ch.id, {
                soc_pct: clamp(5 + 25 * this._rng.next(), 0, 100),
                capacity_kwh: clamp(60 + 40 * this._rng.next(), 1, 1000),
                target_soc_pct: 80,
                departure_time: '08:00',
                priority: 5,
            });
        }
    }

    // ---------------------------------------------------------------------
    // Additional scenarios (v0.3.x)
    // ---------------------------------------------------------------------

    _randomSample(arr, n) {
        const copy = Array.isArray(arr) ? arr.slice() : [];
        // Fisher-Yates shuffle (deterministic via this._rng)
        for (let i = copy.length - 1; i > 0; i--) {
            const j = Math.floor(this._rng.next() * (i + 1));
            const tmp = copy[i];
            copy[i] = copy[j];
            copy[j] = tmp;
        }
        return copy.slice(0, Math.max(0, Math.min(n, copy.length)));
    }

    _clearAllEvcsSimFlags() {
        if (!this._model) return;
        for (const ch of this._model.evcs.chargers) {
            if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
            ch.sim.faulted = false;
            ch.sim.unavailable = false;
            ch.sim.meter_freeze = false;
        }
    }

    _scenarioSuiteSmokeSetup() {
        // Baseline + suite runtime data (stage machine in _runScenarioTimeline)
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();
        this._scenario.data = { stage: 'baseline' };
    }

    _scenarioSuiteFullSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();
        this._scenario.data = { stage: 'baseline' };
    }

    _scenarioLm50PortsDeadline() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        // Tight grid constraint
        this._model.grid.limit_kw = 40;
        this._model.grid.base_load_kw = 10;
        this._model.grid.available = true;

        // Night: PV off
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        // Plug ALL available charge points
        const now = new Date();
        for (let i = 0; i < this._model.evcs.chargers.length; i++) {
            const ch = this._model.evcs.chargers[i];
            const soc = clamp(5 + 70 * this._rng.next(), 0, 100);
            const cap = clamp(40 + 70 * this._rng.next(), 1, 1000);
            this._plugCharger(ch.id, {
                soc_pct: soc,
                capacity_kwh: cap,
                target_soc_pct: 100,
                departure_time: '06:15',
                priority: 5,
            });
            // Keep deterministic departure timestamp aligned
            ch.vehicle.departure_ts = parseDepartureToTimestamp(ch.vehicle.departure_time, now);
        }
    }

    _scenarioLmPriorities3tiers() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 40;
        this._model.grid.base_load_kw = 8;
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        const total = Math.min(15, this._model.evcs.chargers.length);
        for (let i = 1; i <= total; i++) {
            const cid = `c${pad2(i)}`;
            const pri = (i <= 3) ? 10 : (i <= 8) ? 7 : 4;
            this._plugCharger(cid, {
                soc_pct: clamp(10 + 50 * this._rng.next(), 0, 100),
                capacity_kwh: clamp(50 + 40 * this._rng.next(), 1, 1000),
                target_soc_pct: 100,
                departure_time: '06:15',
                priority: pri,
            });
        }
    }

    _scenarioLmArrivalWaveSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 60;
        this._model.grid.base_load_kw = 8;
        this._model.grid.available = true;

        // Night baseline
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        this._scenario.data = {
            total: Math.min(20, this._model.evcs.chargers.length),
            next_index: 1,
            interval_s: 10,
            next_at_s: 0,
        };
    }

    _scenarioLmDepartureWaveSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 60;
        this._model.grid.base_load_kw = 8;
        this._model.grid.available = true;

        // Plug 20 sessions upfront
        const total = Math.min(20, this._model.evcs.chargers.length);
        for (let i = 1; i <= total; i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 60 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
                priority: 5,
            });
        }

        this._scenario.data = {
            total,
            next_unplug: 1,
            interval_s: 12,
            next_at_s: 20,
        };
    }

    _scenarioPvCloudRampSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 200;
        this._model.grid.base_load_kw = 20;

        // Ensure sufficient PV size and use override for deterministic ramp
        this._model.pv.installed_kwp = Math.max(600, this._model.pv.installed_kwp);
        this._model.pv.override.enabled = true;
        this._model.pv.override.power_kw = 0;

        // Plug 20 sessions so PV variability has visible effect
        const count = Math.min(20, this._model.evcs.chargers.length);
        for (let i = 1; i <= count; i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 50 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '17:00',
                priority: 5,
            });
        }

        this._scenario.data = {
            max_pv_kw: 550,
        };
    }

    _scenarioStorageSoc(socPct) {
        if (!this._model) return;
        this._model.storage.soc_pct = clamp(socPct, 0, 100);
        this._model.storage.ctrl.enabled = true;
    }

    _scenarioStoragePowerLimitLow() {
        if (!this._model) return;
        this._model.storage.max_charge_kw = 20;
        this._model.storage.max_discharge_kw = 20;
        this._model.storage.ctrl.enabled = true;
    }

    _scenarioTariffFlatLow10ct() {
        if (!this._model) return;
        this._model.tariff.mode = 'manual';
        this._model.tariff.price_ct_per_kwh = 10;
    }

    _scenarioTariffExtremesTimelineSetup() {
        if (!this._model) return;
        this._model.tariff.mode = 'manual';
        this._model.tariff.price_ct_per_kwh = -20;
    }

    _scenarioGrid14aLimitTimelineSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 40;
        this._model.grid.base_load_kw = 8;
        this._model.grid.available = true;

        // Plug 12 sessions
        const count = Math.min(12, this._model.evcs.chargers.length);
        for (let i = 1; i <= count; i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 60 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
            });
        }
    }

    _scenarioBaseLoadSpikeTimelineSetup() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 80;
        this._model.grid.base_load_kw = 8;
        this._model.grid.available = true;

        // Plug 10 sessions
        const count = Math.min(10, this._model.evcs.chargers.length);
        for (let i = 1; i <= count; i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(10 + 50 * this._rng.next(), 0, 100),
                capacity_kwh: 60,
                target_soc_pct: 100,
                departure_time: '06:15',
            });
        }
    }

    _scenarioDcTaperSingle400() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 1000;
        this._model.grid.base_load_kw = 30;
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        const dc400 = this._model.evcs.chargers.find(c => c.type === 'dc' && c.max_kw >= 390) || this._model.evcs.chargers.find(c => c.type === 'dc');
        if (!dc400) return;

        this._plugCharger(dc400.id, {
            soc_pct: 75,
            capacity_kwh: 90,
            target_soc_pct: 100,
            departure_time: '08:00',
            priority: 5,
        });
    }

    _scenarioDcAllPortsStress() {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 5000;
        this._model.grid.base_load_kw = 50;
        this._model.pv.installed_kwp = 0;
        this._model.pv.override.enabled = false;

        const dcChargers = this._model.evcs.chargers.filter(c => c.type === 'dc');
        for (const ch of dcChargers) {
            this._plugCharger(ch.id, {
                soc_pct: clamp(5 + 20 * this._rng.next(), 0, 100),
                capacity_kwh: clamp(60 + 60 * this._rng.next(), 1, 1000),
                target_soc_pct: 80,
                departure_time: '08:00',
                priority: 5,
            });
        }
    }

    _scenarioFaultsEvcsFaulted(count = 5) {
        if (!this._model) return;
        this._clearAllEvcsSimFlags();

        const sample = this._randomSample(this._model.evcs.chargers, count);
        for (const ch of sample) {
            if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
            ch.sim.faulted = true;
            ch.sim.unavailable = false;

            // Make it visible during charging attempts
            ch.ctrl.plugged = true;
            ch.ctrl.enabled = true;
            ch.ctrl.limit_kw = ch.max_kw;
            ch.meas.power_kw = 0;
            ch.meas.status = 'Faulted';
        }
    }

    _scenarioFaultsEvcsUnavailable(count = 5) {
        if (!this._model) return;
        this._clearAllEvcsSimFlags();

        const sample = this._randomSample(this._model.evcs.chargers, count);
        for (const ch of sample) {
            if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
            ch.sim.unavailable = true;
            ch.sim.faulted = false;

            ch.ctrl.plugged = true;
            ch.ctrl.enabled = true;
            ch.ctrl.limit_kw = ch.max_kw;
            ch.meas.power_kw = 0;
            ch.meas.status = 'Unavailable';
        }
    }

    _scenarioFaultsEvcsMeterFreeze(count = 3) {
        if (!this._model) return;
        this._clearAllEvcsSimFlags();

        const sample = this._randomSample(this._model.evcs.chargers, count);
        for (const ch of sample) {
            if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
            ch.sim.meter_freeze = true;
            ch.sim.faulted = false;
            ch.sim.unavailable = false;

            // Create a "stuck" charging snapshot
            ch.ctrl.plugged = true;
            ch.ctrl.enabled = true;
            ch.ctrl.limit_kw = ch.max_kw;

            const p = clamp(Math.min(ch.max_kw, 0.6 * ch.max_kw + 5), 1, ch.max_kw);
            ch.meas.power_kw = p;
            ch.meas.status = 'Charging';
            ch.meas.energy_kwh = clamp(ch.meas.energy_kwh + 0.1, 0, 1000000);

            ch.vehicle.soc_pct = clamp(ch.vehicle.soc_pct, 5, 95);
        }
    }

    _scenarioFaultsClearAll() {
        this._clearAllEvcsSimFlags();
    }

    _scenarioFuzzSetup({ intensity = 'medium', duration_s = 600 } = {}) {
        this._resetToDefaultsModel();
        this._unplugAllChargers();
        this._clearAllEvcsSimFlags();

        this._model.grid.limit_kw = 120;
        this._model.grid.base_load_kw = 12;
        this._model.grid.available = true;

        this._model.pv.installed_kwp = Math.max(600, this._model.pv.installed_kwp);
        this._model.pv.override.enabled = false;

        // Plug 25 sessions as baseline load
        const count = Math.min(25, this._model.evcs.chargers.length);
        for (let i = 1; i <= count; i++) {
            this._plugCharger(`c${pad2(i)}`, {
                soc_pct: clamp(5 + 70 * this._rng.next(), 0, 100),
                capacity_kwh: clamp(45 + 55 * this._rng.next(), 1, 1000),
                target_soc_pct: 100,
                departure_time: '06:15',
                priority: 5,
            });
        }

        this._scenario.data = {
            intensity,
            duration_s,
            next_event_s: 0,
            // medium: 1 event every 5s, heavy: 2 events every 1s
            interval_s: (intensity === 'heavy') ? 1 : 5,
        };
    }

    _scenarioExternalHeatpump(powerKw) {
        if (!this._model) return;
        this._model.heatpump.ctrl.enabled = true;
        this._model.heatpump.ctrl.power_set_kw = clamp(powerKw, 0, 500);
    }

    _scenarioExternalChp(powerKw) {
        if (!this._model) return;
        this._model.chp.ctrl.enabled = true;
        this._model.chp.ctrl.power_set_kw = clamp(powerKw, 0, 5000);
    }

    _scenarioExternalGenerator(powerKw) {
        if (!this._model) return;
        this._model.generator.ctrl.enabled = true;
        this._model.generator.ctrl.power_set_kw = clamp(powerKw, 0, 5000);
    }

    _fuzzRandomEvent(intensity = 'medium') {
        if (!this._model) return;
        const heavy = intensity === 'heavy';

        // Weighted event selection
        const r = this._rng.next();

        // Helper to pick a random charger
        const chargers = this._model.evcs.chargers;
        const ch = chargers[Math.floor(this._rng.next() * chargers.length)];

        if (r < 0.25) {
            // Plug/unplug storm
            const doPlug = this._rng.next() < 0.6;
            ch.ctrl.plugged = doPlug;
            ch.ctrl.enabled = doPlug;
            if (doPlug) {
                ch.ctrl.limit_kw = ch.max_kw;
                ch.vehicle.soc_pct = clamp(ch.vehicle.soc_pct + this._rng.normal(0, 1), 0, 100);
                ch.meas.status = 'Preparing';
            } else {
                ch.meas.power_kw = 0;
                ch.meas.status = 'Available';
            }
        } else if (r < 0.40) {
            // Grid limit swing
            const lo = heavy ? 10 : 20;
            const hi = heavy ? 400 : 200;
            this._model.grid.limit_kw = clamp(lo + (hi - lo) * this._rng.next(), 1, 5000);
        } else if (r < 0.55) {
            // Base load fluctuation
            const lo = heavy ? 0 : 2;
            const hi = heavy ? 150 : 80;
            this._model.grid.base_load_kw = clamp(lo + (hi - lo) * this._rng.next(), 0, 5000);
        } else if (r < 0.70) {
            // PV override toggle + power
            const enable = this._rng.next() < 0.7;
            this._model.pv.override.enabled = enable;
            if (enable) {
                const pv = clamp(50 + 600 * this._rng.next(), 0, 100000);
                this._model.pv.override.power_kw = pv;
            }
        } else if (r < 0.85) {
            // Tariff shock
            this._model.tariff.mode = 'manual';
            const price = heavy ? (-50 + 350 * this._rng.next()) : (-20 + 220 * this._rng.next());
            this._model.tariff.price_ct_per_kwh = clamp(price, -500, 500);
        } else {
            // Fault injection / recovery
            const action = this._rng.next();
            if (action < 0.15) {
                // clear all
                this._clearAllEvcsSimFlags();
            } else if (action < 0.55) {
                // fault one
                if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
                ch.sim.faulted = true;
                ch.sim.unavailable = false;
                ch.sim.meter_freeze = false;
            } else if (action < 0.80) {
                // offline one
                if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
                ch.sim.unavailable = true;
                ch.sim.faulted = false;
                ch.sim.meter_freeze = false;
            } else {
                // meter freeze one
                if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };
                ch.sim.meter_freeze = true;
                ch.sim.faulted = false;
                ch.sim.unavailable = false;
                if ((Number(ch.meas.power_kw) || 0) < 0.5) {
                    ch.meas.power_kw = clamp(Math.min(ch.max_kw, 10 + 0.3 * ch.max_kw), 0, ch.max_kw);
                    ch.meas.status = 'Charging';
                }
            }
        }
    }
    _runScenarioTimeline(nowMs, dtSec, now) {
        if (!this._scenario.running || !this._model) return;
        if (!this._scenario.started_ms) return;

        const elapsed = (nowMs - this._scenario.started_ms) / 1000;
        const id = this._scenario.active;
        const data = this._scenario.data || (this._scenario.data = {});

        // Default status
        let status = `running ${id}`;
        let phase = 'running';

        const enterStage = (stage, fn) => {
            if (data.stage !== stage) {
                fn();
                data.stage = stage;
            }
        };

        switch (id) {
            case 'grid_limit_drop_timeline': {
                // 0-60s: 80 kW, 60-180s: 25 kW, 180-240s: 80 kW
                if (elapsed < 60) {
                    this._model.grid.limit_kw = 80;
                    status = 'grid limit = 80 kW (normal)';
                } else if (elapsed < 180) {
                    this._model.grid.limit_kw = 25;
                    status = 'grid limit = 25 kW (drop active)';
                } else if (elapsed < 240) {
                    this._model.grid.limit_kw = 80;
                    status = 'grid limit restored = 80 kW';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'grid_14a_limit_timeline': {
                // 0-60s: normal, 60-300s: reduced, 300-600s: recovered
                if (elapsed < 60) {
                    this._model.grid.limit_kw = 40;
                    status = 'curtailment inactive (limit 40 kW)';
                } else if (elapsed < 300) {
                    this._model.grid.limit_kw = 10;
                    status = 'curtailment active (limit 10 kW)';
                } else if (elapsed < 600) {
                    this._model.grid.limit_kw = 40;
                    status = 'curtailment ended (limit 40 kW)';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'base_load_spike_timeline': {
                // Spiky base load pattern
                if (elapsed < 60) {
                    this._model.grid.base_load_kw = 8;
                    status = 'base load = 8 kW';
                } else if (elapsed < 120) {
                    this._model.grid.base_load_kw = 25;
                    status = 'base load spike = 25 kW';
                } else if (elapsed < 180) {
                    this._model.grid.base_load_kw = 60;
                    status = 'base load spike = 60 kW';
                } else if (elapsed < 240) {
                    this._model.grid.base_load_kw = 15;
                    status = 'base load = 15 kW';
                } else if (elapsed < 300) {
                    this._model.grid.base_load_kw = 8;
                    status = 'base load back to normal';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'tariff_pulse_timeline': {
                // Manual tariff: 10 / 120 / 10 ct/kWh
                this._model.tariff.mode = 'manual';
                if (elapsed < 60) {
                    this._model.tariff.price_ct_per_kwh = 10;
                    status = 'tariff = 10 ct/kWh (low)';
                } else if (elapsed < 120) {
                    this._model.tariff.price_ct_per_kwh = 120;
                    status = 'tariff = 120 ct/kWh (high)';
                } else if (elapsed < 180) {
                    this._model.tariff.price_ct_per_kwh = 10;
                    status = 'tariff = 10 ct/kWh (low again)';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'tariff_extremes_timeline': {
                // Manual tariff: negative / very high / normal
                this._model.tariff.mode = 'manual';
                if (elapsed < 80) {
                    this._model.tariff.price_ct_per_kwh = -20;
                    status = 'tariff = -20 ct/kWh (negative)';
                } else if (elapsed < 160) {
                    this._model.tariff.price_ct_per_kwh = 200;
                    status = 'tariff = 200 ct/kWh (very high)';
                } else if (elapsed < 240) {
                    this._model.tariff.price_ct_per_kwh = 30;
                    status = 'tariff = 30 ct/kWh (normal)';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'pv_cloud_ramp_timeline': {
                // Modulate PV override (clouds) for deterministic, fast tests
                this._model.pv.override.enabled = true;

                const maxPv = clamp(Number(data.max_pv_kw ?? 550), 0, 100000);
                const wave = 0.5 + 0.5 * Math.sin((2 * Math.PI * elapsed) / 120);
                const ripple = 0.85 + 0.15 * Math.sin((2 * Math.PI * elapsed) / 37);

                // occasional "deep cloud" dip every ~90s
                const dip = (Math.sin((2 * Math.PI * elapsed) / 90) > 0.92) ? 0.25 : 1.0;

                const pv = clamp(maxPv * wave * ripple * dip, 0, 100000);
                this._model.pv.override.power_kw = pv;

                status = `PV clouds: ${pv.toFixed(0)} kW (override)`;

                if (elapsed >= 600) {
                    phase = 'done';
                }
                break;
            }

            case 'lm_arrival_wave_timeline': {
                const total = clamp(Number(data.total ?? 20), 1, this._model.evcs.chargers.length);
                const interval = clamp(Number(data.interval_s ?? 10), 1, 120);

                if (data.next_index === undefined) data.next_index = 1;
                if (data.next_at_s === undefined) data.next_at_s = 0;

                if (elapsed >= data.next_at_s && data.next_index <= total) {
                    const cid = `c${pad2(data.next_index)}`;
                    this._plugCharger(cid, {
                        soc_pct: clamp(10 + 60 * this._rng.next(), 0, 100),
                        capacity_kwh: clamp(45 + 55 * this._rng.next(), 1, 1000),
                        target_soc_pct: 100,
                        departure_time: '06:15',
                        priority: 5,
                    });

                    data.next_index += 1;
                    data.next_at_s = elapsed + interval;
                }

                status = `arrival wave: plugged ${Math.min(total, data.next_index - 1)}/${total}`;

                if (elapsed >= 600) {
                    phase = 'done';
                }
                break;
            }

            case 'lm_departure_wave_timeline': {
                const total = clamp(Number(data.total ?? 20), 1, this._model.evcs.chargers.length);
                const interval = clamp(Number(data.interval_s ?? 12), 1, 120);

                if (data.next_unplug === undefined) data.next_unplug = 1;
                if (data.next_at_s === undefined) data.next_at_s = 20;

                if (elapsed >= data.next_at_s && data.next_unplug <= total) {
                    const cid = `c${pad2(data.next_unplug)}`;
                    const ch = this._model.evcs.chargers.find(c => c.id === cid);
                    if (ch) {
                        ch.ctrl.plugged = false;
                        ch.ctrl.enabled = false;
                        ch.meas.power_kw = 0;
                        ch.meas.status = 'Available';
                    }
                    data.next_unplug += 1;
                    data.next_at_s = elapsed + interval;
                }

                status = `departure wave: unplugged ${Math.min(total, data.next_unplug - 1)}/${total}`;

                if (elapsed >= 600) {
                    phase = 'done';
                }
                break;
            }

            case 'grid_blackout_60s_timeline': {
                // 0-30: up, 30-90 down, 90-150 up
                if (elapsed < 30) {
                    this._model.grid.available = true;
                    status = 'grid up (pre)';
                } else if (elapsed < 90) {
                    this._model.grid.available = false;
                    status = 'grid blackout (active)';
                } else if (elapsed < 150) {
                    this._model.grid.available = true;
                    status = 'grid recovered';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'fuzz_10min_medium':
            case 'fuzz_30min_heavy': {
                const intensity = (id === 'fuzz_30min_heavy') ? 'heavy' : (data.intensity ?? 'medium');
                const duration = clamp(Number(data.duration_s ?? (id === 'fuzz_30min_heavy' ? 1800 : 600)), 10, 100000);
                const interval = clamp(Number(data.interval_s ?? (intensity === 'heavy' ? 1 : 5)), 1, 120);

                if (data.next_event_s === undefined) data.next_event_s = 0;

                // Run events
                if (elapsed >= data.next_event_s) {
                    const events = (intensity === 'heavy') ? 2 : 1;
                    for (let i = 0; i < events; i++) {
                        this._fuzzRandomEvent(intensity);
                    }
                    data.next_event_s = elapsed + interval;
                }

                // Derive a compact fault count for status
                let faulted = 0;
                let offline = 0;
                let frozen = 0;
                for (const ch of this._model.evcs.chargers) {
                    if (ch.sim?.faulted) faulted++;
                    if (ch.sim?.unavailable) offline++;
                    if (ch.sim?.meter_freeze) frozen++;
                }

                status = `fuzz(${intensity}): gridLimit=${this._model.grid.limit_kw.toFixed(0)} kW, baseLoad=${this._model.grid.base_load_kw.toFixed(0)} kW, faults=${faulted}, offline=${offline}, frozen=${frozen}`;

                if (elapsed >= duration) {
                    phase = 'done';
                }
                break;
            }

            case 'suite_smoke_all': {
                // A compact, deterministic stage-machine.
                if (elapsed < 10) {
                    enterStage('baseline', () => {
                        this._resetToDefaultsModel();
                        this._unplugAllChargers();
                        this._clearAllEvcsSimFlags();
                    });
                    status = 'suite smoke: baseline';
                } else if (elapsed < 70) {
                    enterStage('lm6', () => this._scenarioLm6CarsDeadline());
                    status = 'suite smoke: LM6 deadline';
                } else if (elapsed < 130) {
                    enterStage('tariff', () => this._scenarioTariffPulseTimelineSetup());
                    const t = elapsed - 70;
                    this._model.tariff.mode = 'manual';
                    this._model.tariff.price_ct_per_kwh = (t < 20) ? 10 : (t < 40) ? 120 : 10;
                    status = `suite smoke: tariff pulse (${this._model.tariff.price_ct_per_kwh} ct/kWh)`;
                } else if (elapsed < 190) {
                    enterStage('grid_drop', () => this._scenarioGridLimitDropTimelineSetup());
                    const t = elapsed - 130;
                    this._model.grid.limit_kw = (t < 20) ? 80 : (t < 40) ? 25 : 80;
                    status = `suite smoke: grid limit ${this._model.grid.limit_kw} kW`;
                } else if (elapsed < 280) {
                    enterStage('pv_surplus', () => this._scenarioPvSurplus30Cars());
                    status = 'suite smoke: PV surplus (override)';
                } else if (elapsed < 350) {
                    enterStage('dc_rush', () => this._scenarioDcRush10());
                    status = 'suite smoke: DC rush';
                } else if (elapsed < 410) {
                    enterStage('faults', () => this._scenarioFaultsEvcsFaulted(5));
                    status = 'suite smoke: fault injection (5 faulted)';
                } else if (elapsed < 420) {
                    enterStage('cleanup', () => this._scenarioFaultsClearAll());
                    status = 'suite smoke: cleanup';
                } else {
                    phase = 'done';
                }
                break;
            }

            case 'suite_full_all': {
                // Longer stage-suite: arrival → PV clouds → tariff extremes → fuzz → done
                if (elapsed < 30) {
                    enterStage('baseline', () => {
                        this._resetToDefaultsModel();
                        this._unplugAllChargers();
                        this._clearAllEvcsSimFlags();
                        this._model.grid.limit_kw = 80;
                        this._model.grid.base_load_kw = 10;
                        this._model.pv.override.enabled = false;
                    });
                    status = 'suite full: baseline';
                } else if (elapsed < 630) {
                    // Arrival wave for 10 minutes
                    enterStage('arrival', () => {
                        this._scenarioLmArrivalWaveSetup();
                        // Store arrival settings under suite data as well
                        data.arrival_total = data.total ?? 30;
                    });

                    // Reuse arrival-wave logic (without changing scenario.active)
                    const total = Math.min(30, this._model.evcs.chargers.length);
                    if (data.arrival_next === undefined) data.arrival_next = 1;
                    if (data.arrival_next_at === undefined) data.arrival_next_at = 30;

                    if (elapsed >= data.arrival_next_at && data.arrival_next <= total) {
                        const cid = `c${pad2(data.arrival_next)}`;
                        this._plugCharger(cid, {
                            soc_pct: clamp(10 + 60 * this._rng.next(), 0, 100),
                            capacity_kwh: clamp(45 + 55 * this._rng.next(), 1, 1000),
                            target_soc_pct: 100,
                            departure_time: '06:15',
                            priority: 5,
                        });
                        data.arrival_next += 1;
                        data.arrival_next_at = elapsed + 10;
                    }

                    status = `suite full: arrival wave (${Math.min(total, data.arrival_next - 1)}/${total})`;
                } else if (elapsed < 1230) {
                    enterStage('pv_clouds', () => {
                        this._scenarioPvCloudRampSetup();
                    });

                    // Keep PV clouds running
                    this._model.pv.override.enabled = true;
                    const maxPv = 550;
                    const wave = 0.5 + 0.5 * Math.sin((2 * Math.PI * (elapsed - 630)) / 180);
                    const dip = (Math.sin((2 * Math.PI * (elapsed - 630)) / 75) > 0.92) ? 0.3 : 1.0;
                    const pv = clamp(maxPv * wave * dip, 0, 100000);
                    this._model.pv.override.power_kw = pv;

                    status = `suite full: PV clouds (${pv.toFixed(0)} kW)`;
                } else if (elapsed < 1470) {
                    enterStage('tariff_extremes', () => {
                        this._model.tariff.mode = 'manual';
                        this._model.tariff.price_ct_per_kwh = -20;
                    });

                    const t = elapsed - 1230;
                    this._model.tariff.mode = 'manual';
                    this._model.tariff.price_ct_per_kwh = (t < 80) ? -20 : (t < 160) ? 200 : 30;
                    status = `suite full: tariff ${this._model.tariff.price_ct_per_kwh} ct/kWh`;
                } else if (elapsed < 1770) {
                    enterStage('fuzz', () => {
                        // Keep existing sessions but start fuzz events
                        data.fuzz_intensity = 'medium';
                        data.next_event_s = elapsed;
                        data.interval_s = 5;
                    });

                    if (elapsed >= data.next_event_s) {
                        this._fuzzRandomEvent('medium');
                        data.next_event_s = elapsed + 5;
                    }

                    status = 'suite full: fuzz (medium)';
                } else if (elapsed < 1800) {
                    enterStage('cleanup', () => this._scenarioFaultsClearAll());
                    status = 'suite full: cleanup';
                } else {
                    phase = 'done';
                }
                break;
            }

            default:
                // no timeline
                break;
        }

        this._scenario.status = status;
        this._scenario.phase = phase;

        if (phase === 'done') {
            // Auto-stop
            this._scenario.running = false;
            this._scenario.started_ms = 0;
            this._scenario.phase = 'done';
            this._scenario.status = `done ${id}`;
        }
    }

    async _createObjects(cfg) {
        // Root channels
        await this._ensureChannel('grid', 'Grid');
        await this._ensureChannel('pv', 'PV');
        await this._ensureChannel('pv.override', 'PV Override');
        await this._ensureChannel('storage', 'Storage');
        await this._ensureChannel('storage.ctrl', 'Storage Control');
        await this._ensureChannel('heatpump', 'Heatpump');
        await this._ensureChannel('heatpump.ctrl', 'Heatpump Control');
        await this._ensureChannel('chp', 'CHP / BHKW');
        await this._ensureChannel('chp.ctrl', 'CHP Control');
        await this._ensureChannel('generator', 'Generator');
        await this._ensureChannel('generator.ctrl', 'Generator Control');
        await this._ensureChannel('tariff', 'Tariff');
        await this._ensureChannel('evcs', 'EVCS');
        await this._ensureChannel('evcs.total', 'EVCS Totals');

        // Scenario/Test control
        await this._ensureChannel('scenario', 'Test Scenarios');
        await this._ensureChannel('scenario.ctrl', 'Scenario Control');
        await this._ensureChannel('scenario.buttons', 'Scenario Quick Buttons');

        // Grid states
        await this._ensureState('grid.available', { type: 'boolean', role: 'indicator.reachable', read: true, write: true, def: true });
        await this._ensureState('grid.limit_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: cfg.gridLimitKw, min: 1, max: 5000 });
        await this._ensureState('grid.base_load_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: true, def: cfg.baseLoadKw, min: 0, max: 5000 });
        await this._ensureState('grid.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('grid.over_limit', { type: 'boolean', role: 'indicator.alarm', read: true, write: false, def: false });

        // Tariff
        await this._ensureState('tariff.mode', { type: 'string', role: 'text', read: true, write: true, def: 'auto' });
        await this._ensureState('tariff.price_ct_per_kwh', { type: 'number', role: 'value.price', unit: 'ct/kWh', read: true, write: true, def: 30, min: -500, max: 500 });
        await this._ensureState('tariff.price_next_24h_json', { type: 'string', role: 'json', read: true, write: false, def: '[]' });

        // Scenario states
        await this._ensureState('scenario.selected', {
            type: 'string',
            role: 'text',
            read: true,
            write: true,
            def: 'baseline',
        });
        await this._ensureState('scenario.active', { type: 'string', role: 'text', read: true, write: false, def: 'baseline' });
        await this._ensureState('scenario.running', { type: 'boolean', role: 'indicator.working', read: true, write: false, def: false });
        await this._ensureState('scenario.phase', { type: 'string', role: 'text', read: true, write: false, def: 'idle' });
        await this._ensureState('scenario.elapsed_s', { type: 'number', role: 'value.time', unit: 's', read: true, write: false, def: 0 });
        await this._ensureState('scenario.status', { type: 'string', role: 'text', read: true, write: false, def: 'idle' });
        await this._ensureState('scenario.catalog_json', { type: 'string', role: 'json', read: true, write: false, def: '[]' });

        await this._ensureState('scenario.ctrl.apply', { type: 'boolean', role: 'button', read: true, write: true, def: false });
        await this._ensureState('scenario.ctrl.start', { type: 'boolean', role: 'button', read: true, write: true, def: false });
        await this._ensureState('scenario.ctrl.stop', { type: 'boolean', role: 'button', read: true, write: true, def: false });
        await this._ensureState('scenario.ctrl.reset', { type: 'boolean', role: 'button', read: true, write: true, def: false });

        // Quick buttons: one click per scenario (best for fast testing in Admin)
        // We auto-generate one button per scenario definition so new scenarios appear automatically.
        for (const s of this._scenarioDefinitions()) {
            await this._ensureState(`scenario.buttons.${s.id}`, { type: 'boolean', role: 'button', read: true, write: true, def: false });
        }

        // PV
        await this._ensureState('pv.installed_kwp', { type: 'number', role: 'value', unit: 'kWp', read: true, write: true, def: cfg.pvInstalledKwp, min: 0, max: 5000 });
        await this._ensureState('pv.weather_factor', { type: 'number', role: 'level', unit: '', read: true, write: true, def: cfg.pvWeatherFactor, min: 0, max: 1 });
        await this._ensureState('pv.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('pv.override.enabled', { type: 'boolean', role: 'switch.enable', read: true, write: true, def: false });
        await this._ensureState('pv.override.power_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: 0, min: 0, max: 100000 });

        // Storage
        await this._ensureState('storage.capacity_kwh', { type: 'number', role: 'value', unit: 'kWh', read: true, write: true, def: cfg.storageCapacityKwh, min: 1, max: 50000 });
        await this._ensureState('storage.max_charge_kw', { type: 'number', role: 'value', unit: 'kW', read: true, write: true, def: cfg.storageMaxChargeKw, min: 0, max: 50000 });
        await this._ensureState('storage.max_discharge_kw', { type: 'number', role: 'value', unit: 'kW', read: true, write: true, def: cfg.storageMaxDischargeKw, min: 0, max: 50000 });
        await this._ensureState('storage.soc_pct', { type: 'number', role: 'value.battery', unit: '%', read: true, write: true, def: cfg.storageInitialSocPct, min: 0, max: 100 });
        await this._ensureState('storage.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('storage.ctrl.enabled', { type: 'boolean', role: 'switch.enable', read: true, write: true, def: true });
        await this._ensureState('storage.ctrl.power_set_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: 0 });

        // Heatpump / CHP / Generator
        await this._ensureState('heatpump.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('heatpump.ctrl.enabled', { type: 'boolean', role: 'switch.enable', read: true, write: true, def: false });
        await this._ensureState('heatpump.ctrl.power_set_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: 0 });

        await this._ensureState('chp.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('chp.ctrl.enabled', { type: 'boolean', role: 'switch.enable', read: true, write: true, def: false });
        await this._ensureState('chp.ctrl.power_set_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: 0 });

        await this._ensureState('generator.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('generator.ctrl.enabled', { type: 'boolean', role: 'switch.enable', read: true, write: true, def: false });
        await this._ensureState('generator.ctrl.power_set_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: 0 });

        // EVCS totals
        await this._ensureState('evcs.count', { type: 'number', role: 'value', read: true, write: false, def: cfg.chargersCount });
        await this._ensureState('evcs.total_power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('evcs.total_energy_kwh', { type: 'number', role: 'value.energy', unit: 'kWh', read: true, write: false, def: 0 });

        // EVCS chargers
        const specs = this._defaultChargerSpecs(cfg.chargersCount);
        for (let i = 1; i <= cfg.chargersCount; i++) {
            const id = `c${pad2(i)}`;
            const spec = specs[i - 1];

            await this._ensureChannel(`evcs.${id}`, `Charge Point ${id.toUpperCase()}`);
            await this._ensureChannel(`evcs.${id}.meta`, 'Meta');
            await this._ensureChannel(`evcs.${id}.ctrl`, 'Control');
            await this._ensureChannel(`evcs.${id}.vehicle`, 'Vehicle');
            await this._ensureChannel(`evcs.${id}.meas`, 'Measurements');

            await this._ensureState(`evcs.${id}.meta.type`, { type: 'string', role: 'text', read: true, write: false, def: spec.type });
            await this._ensureState(`evcs.${id}.meta.max_kw`, { type: 'number', role: 'value', unit: 'kW', read: true, write: false, def: spec.max_kw });

            await this._ensureState(`evcs.${id}.ctrl.enabled`, { type: 'boolean', role: 'switch.enable', read: true, write: true, def: false });
            await this._ensureState(`evcs.${id}.ctrl.limit_kw`, { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: spec.max_kw, min: 0, max: 10000 });
            await this._ensureState(`evcs.${id}.ctrl.plugged`, { type: 'boolean', role: 'indicator.connected', read: true, write: true, def: false });
            await this._ensureState(`evcs.${id}.ctrl.priority`, { type: 'number', role: 'value', read: true, write: true, def: 5, min: 1, max: 10 });
            await this._ensureState(`evcs.${id}.ctrl.reset_session`, { type: 'boolean', role: 'button', read: true, write: true, def: false });

            // Vehicle values are writable to allow manual SoC/test patterns
            await this._ensureState(`evcs.${id}.vehicle.id`, { type: 'string', role: 'text', read: true, write: true, def: `VEH-${pad2(i)}` });
            await this._ensureState(`evcs.${id}.vehicle.soc_pct`, { type: 'number', role: 'value.battery', unit: '%', read: true, write: true, def: 20, min: 0, max: 100 });
            await this._ensureState(`evcs.${id}.vehicle.capacity_kwh`, { type: 'number', role: 'value', unit: 'kWh', read: true, write: true, def: 60, min: 1, max: 1000 });
            await this._ensureState(`evcs.${id}.vehicle.max_charge_kw`, { type: 'number', role: 'value', unit: 'kW', read: true, write: true, def: spec.max_kw, min: 0, max: 2000 });
            await this._ensureState(`evcs.${id}.vehicle.target_soc_pct`, { type: 'number', role: 'level', unit: '%', read: true, write: true, def: cfg.defaultTargetSocPct, min: 0, max: 100 });
            await this._ensureState(`evcs.${id}.vehicle.departure_time`, { type: 'string', role: 'text', read: true, write: true, def: cfg.defaultDepartureTime });

            // Measurements (read-only)
            await this._ensureState(`evcs.${id}.meas.status`, { type: 'string', role: 'text', read: true, write: false, def: 'Available' });
            await this._ensureState(`evcs.${id}.meas.power_kw`, { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
            await this._ensureState(`evcs.${id}.meas.energy_kwh`, { type: 'number', role: 'value.energy', unit: 'kWh', read: true, write: false, def: 0 });
        }
    }

    async _initModel(cfg) {
        // Load existing states (to respect persisted values) or fall back to defaults.
        const get = async (id, def) => {
            const st = await this.getStateAsync(id);
            if (!st || st.val === null || st.val === undefined) return def;
            return st.val;
        };

        const gridLimit = clamp(await get('grid.limit_kw', cfg.gridLimitKw), 1, 5000);
        const baseLoad = clamp(await get('grid.base_load_kw', cfg.baseLoadKw), 0, 5000);
        const gridAvailable = !!(await get('grid.available', true));

        const pvInstalled = clamp(await get('pv.installed_kwp', cfg.pvInstalledKwp), 0, 5000);
        const pvWeather = clamp(await get('pv.weather_factor', cfg.pvWeatherFactor), 0, 1);
        const pvOverrideEnabled = !!(await get('pv.override.enabled', false));
        const pvOverridePower = clamp(await get('pv.override.power_kw', 0), 0, 100000);

        const storageCapacity = clamp(await get('storage.capacity_kwh', cfg.storageCapacityKwh), 1, 50000);
        const storageMaxCh = clamp(await get('storage.max_charge_kw', cfg.storageMaxChargeKw), 0, 50000);
        const storageMaxDis = clamp(await get('storage.max_discharge_kw', cfg.storageMaxDischargeKw), 0, 50000);
        const storageSoc = clamp(await get('storage.soc_pct', cfg.storageInitialSocPct), 0, 100);
        const storageEnabled = !!(await get('storage.ctrl.enabled', true));
        const storageSet = Number(await get('storage.ctrl.power_set_kw', 0)) || 0;

        const tariffMode = String(await get('tariff.mode', 'auto'));
        const tariffPrice = clamp(await get('tariff.price_ct_per_kwh', 30), -500, 500);

        const hpEnabled = !!(await get('heatpump.ctrl.enabled', false));
        const hpSet = Number(await get('heatpump.ctrl.power_set_kw', 0)) || 0;

        const chpEnabled = !!(await get('chp.ctrl.enabled', false));
        const chpSet = Number(await get('chp.ctrl.power_set_kw', 0)) || 0;

        const genEnabled = !!(await get('generator.ctrl.enabled', false));
        const genSet = Number(await get('generator.ctrl.power_set_kw', 0)) || 0;

        // EVCS
        const specs = this._defaultChargerSpecs(cfg.chargersCount);
        const chargers = [];
        const now = new Date();

        for (let i = 1; i <= cfg.chargersCount; i++) {
            const id = `c${pad2(i)}`;
            const spec = specs[i - 1];

            const pluggedDefault = cfg.autoConnectEnabled && i <= cfg.autoConnectCount;

            const plugged = !!(await get(`evcs.${id}.ctrl.plugged`, pluggedDefault));
            const enabled = !!(await get(`evcs.${id}.ctrl.enabled`, false));
            const limitKw = clamp(await get(`evcs.${id}.ctrl.limit_kw`, spec.max_kw), 0, 10000);
            const priority = clamp(await get(`evcs.${id}.ctrl.priority`, 5), 1, 10);

            const vehId = String(await get(`evcs.${id}.vehicle.id`, `VEH-${pad2(i)}`));
            const soc = clamp(await get(`evcs.${id}.vehicle.soc_pct`, clamp(10 + 60 * this._rng.next(), 0, 100)), 0, 100);
            const cap = clamp(await get(`evcs.${id}.vehicle.capacity_kwh`, clamp(40 + 50 * this._rng.next(), 1, 1000)), 1, 1000);
            const maxCharge = clamp(await get(`evcs.${id}.vehicle.max_charge_kw`, spec.max_kw), 0, 2000);
            const targetSoc = clamp(await get(`evcs.${id}.vehicle.target_soc_pct`, cfg.defaultTargetSocPct), 0, 100);
            const departureTime = String(await get(`evcs.${id}.vehicle.departure_time`, cfg.defaultDepartureTime));
            const departureTs = parseDepartureToTimestamp(departureTime, now);

            const energyKwh = clamp(await get(`evcs.${id}.meas.energy_kwh`, 0), 0, 100000);
            const status = String(await get(`evcs.${id}.meas.status`, plugged ? 'Preparing' : 'Available'));

            chargers.push({
                id,
                type: spec.type,
                max_kw: spec.max_kw,
                sim: { faulted: false, unavailable: false, meter_freeze: false },
                ctrl: { enabled, limit_kw: limitKw, plugged, priority },
                vehicle: {
                    id: vehId,
                    soc_pct: soc,
                    capacity_kwh: cap,
                    max_charge_kw: maxCharge,
                    target_soc_pct: targetSoc,
                    departure_time: departureTime,
                    departure_ts: departureTs,
                },
                meas: {
                    status,
                    power_kw: 0,
                    energy_kwh: energyKwh,
                },
            });
        }

        this._model = {
            grid: { available: gridAvailable, limit_kw: gridLimit, base_load_kw: baseLoad, power_kw: 0, over_limit: false },
            tariff: { mode: (tariffMode.toLowerCase() === 'manual') ? 'manual' : 'auto', price_ct_per_kwh: tariffPrice, price_next_24h_json: '[]' },
            pv: {
                installed_kwp: pvInstalled,
                weather_factor: pvWeather,
                power_kw: 0,
                override: { enabled: pvOverrideEnabled, power_kw: pvOverridePower },
            },
            storage: {
                capacity_kwh: storageCapacity,
                max_charge_kw: storageMaxCh,
                max_discharge_kw: storageMaxDis,
                soc_pct: storageSoc,
                power_kw: 0,
                ctrl: { enabled: storageEnabled, power_set_kw: storageSet },
            },
            heatpump: { power_kw: 0, ctrl: { enabled: hpEnabled, power_set_kw: hpSet } },
            chp: { power_kw: 0, ctrl: { enabled: chpEnabled, power_set_kw: chpSet } },
            generator: { power_kw: 0, ctrl: { enabled: genEnabled, power_set_kw: genSet } },
            evcs: { chargers },
        };
    }

    _defaultChargerSpecs(count) {
        // Default mix:
        // - ~50% AC 11 kW
        // - ~30% AC 22 kW
        // - remaining DC in repeating pattern 50/150/300/400
        const specs = [];
        const ac11 = Math.max(1, Math.round(count * 0.5));
        const ac22 = Math.max(0, Math.round(count * 0.3));
        const remaining = Math.max(0, count - ac11 - ac22);

        for (let i = 0; i < ac11; i++) specs.push({ type: 'ac', max_kw: 11 });
        for (let i = 0; i < ac22; i++) specs.push({ type: 'ac', max_kw: 22 });

        const dcPattern = [50, 150, 300, 400];
        for (let i = 0; i < remaining; i++) {
            const maxKw = dcPattern[i % dcPattern.length];
            specs.push({ type: 'dc', max_kw: maxKw });
        }

        // If rounding exceeded count, trim
        return specs.slice(0, count);
    }

    async _tick() {
        if (!this._model) return;

        const nowMs = Date.now();
        const last = this._lastTickMs ?? nowMs;
        this._lastTickMs = nowMs;

        const dtSec = Math.max(0.2, Math.min(60, (nowMs - last) / 1000));
        const dtH = dtSec / 3600;

        const now = new Date(nowMs);

        // Scenario engine (time-based modifications)
        this._runScenarioTimeline(nowMs, dtSec, now);

        // Tariff
        // We generate both the current price and a simple forward curve.
        // Important: base/amp must be available for curve generation even in manual mode.
        let base = 32;
        let amp = 10;

        if (this._model.tariff.mode === 'auto') {
            const hour = now.getHours() + now.getMinutes() / 60;
            // Example: 22..45 ct/kWh with daily sine + noise
            const daily = base + amp * Math.sin((2 * Math.PI * (hour - 7)) / 24);
            const noise = this._rng.normal(0, 0.4);
            this._model.tariff.price_ct_per_kwh = clamp(daily + noise, -500, 500);
        } else {
            // Manual mode: keep current price as baseline and use a flat curve
            base = clamp(this._model.tariff.price_ct_per_kwh ?? base, -500, 500);
            amp = 0;
            this._model.tariff.price_ct_per_kwh = base;
        }

        // Build a simple 24h forward curve (hourly) for EMS tests
        const curve = [];
        for (let h = 0; h < 24; h++) {
            const d = new Date(nowMs + h * 3600 * 1000);
            const hh = d.getHours() + d.getMinutes() / 60;
            const v = base + amp * Math.sin((2 * Math.PI * (hh - 7)) / 24);
            curve.push(Number(clamp(v, -500, 500).toFixed(2)));
        }
        this._model.tariff.price_next_24h_json = JSON.stringify(curve);

        // PV
        if (this._model.pv.override?.enabled) {
            this._model.pv.power_kw = clamp(this._model.pv.override.power_kw, 0, 100000);
        } else {
            const pvProfile = solarProfile01(now);
            const pvNoise = clamp(this._rng.normal(0, 0.02), -0.05, 0.05);
            this._model.pv.power_kw = clamp(
                this._model.pv.installed_kwp * this._model.pv.weather_factor * pvProfile * (1 + pvNoise),
                0,
                Math.max(0, this._model.pv.installed_kwp)
            );
        }

        // Flexible producers
        this._model.chp.power_kw = (this._model.chp.ctrl.enabled) ? clamp(this._model.chp.ctrl.power_set_kw, 0, 5000) : 0;
        this._model.generator.power_kw = (this._model.generator.ctrl.enabled) ? clamp(this._model.generator.ctrl.power_set_kw, 0, 5000) : 0;

        // Heatpump as consumer
        this._model.heatpump.power_kw = (this._model.heatpump.ctrl.enabled) ? clamp(this._model.heatpump.ctrl.power_set_kw, 0, 500) : 0;

        // Storage follows power_set_kw
        let storagePower = 0;
        if (this._model.storage.ctrl.enabled) {
            const req = Number(this._model.storage.ctrl.power_set_kw) || 0; // + discharge / - charge
            const maxDis = this._model.storage.max_discharge_kw;
            const maxCh = this._model.storage.max_charge_kw;
            storagePower = clamp(req, -maxCh, maxDis);

            // SoC boundaries
            if (this._model.storage.soc_pct <= 0 && storagePower > 0) storagePower = 0;
            if (this._model.storage.soc_pct >= 100 && storagePower < 0) storagePower = 0;

            // Update SoC (discharge reduces)
            const cap = Math.max(1e-6, this._model.storage.capacity_kwh);
            const deltaSoc = (-storagePower * dtH / cap) * 100;
            this._model.storage.soc_pct = clamp(this._model.storage.soc_pct + deltaSoc, 0, 100);
        }
        this._model.storage.power_kw = storagePower;

        // EVCS
        let evTotalPower = 0;
        let evTotalEnergy = 0;

        for (const ch of this._model.evcs.chargers) {
            // Backward compatible: ensure sim flags exist
            if (!ch.sim) ch.sim = { faulted: false, unavailable: false, meter_freeze: false };

            // Fault/offline simulation overrides normal behaviour
            if (ch.sim.unavailable) {
                ch.meas.power_kw = 0;
                ch.meas.status = 'Unavailable';
                evTotalEnergy += ch.meas.energy_kwh;
                continue;
            }
            if (ch.sim.faulted) {
                ch.meas.power_kw = 0;
                ch.meas.status = 'Faulted';
                evTotalEnergy += ch.meas.energy_kwh;
                continue;
            }
            if (ch.sim.meter_freeze) {
                // Keep previous meas.* and vehicle.* values unchanged
                evTotalPower += Number(ch.meas.power_kw) || 0;
                evTotalEnergy += ch.meas.energy_kwh;
                continue;
            }

            const plugged = !!ch.ctrl.plugged;
            if (!plugged) {
                ch.meas.power_kw = 0;
                ch.meas.status = 'Available';
                evTotalEnergy += ch.meas.energy_kwh;
                continue;
            }

            const targetSoc = clamp(ch.vehicle.target_soc_pct, 0, 100);
            if (ch.vehicle.soc_pct >= targetSoc - 0.01) {
                ch.meas.power_kw = 0;
                ch.meas.status = 'Finished';
                ch.vehicle.soc_pct = clamp(ch.vehicle.soc_pct, 0, 100);
                evTotalEnergy += ch.meas.energy_kwh;
                continue;
            }

            if (!ch.ctrl.enabled) {
                ch.meas.power_kw = 0;
                ch.meas.status = 'Preparing';
                evTotalEnergy += ch.meas.energy_kwh;
                continue;
            }

            const limitKw = clamp(ch.ctrl.limit_kw, 0, 10000);
            let p = Math.min(limitKw, ch.max_kw, ch.vehicle.max_charge_kw);

            if (ch.type === 'dc') {
                p *= dcTaperFactor(ch.vehicle.soc_pct);
            }

            // If close to full, don't overshoot energy in the timestep
            const capKwh = Math.max(0.1, ch.vehicle.capacity_kwh);
            const energyNeedKwh = Math.max(0, (targetSoc - ch.vehicle.soc_pct) / 100 * capKwh);
            const maxStepKwh = Math.max(0, p * dtH);
            if (maxStepKwh > energyNeedKwh && dtH > 0) {
                p = energyNeedKwh / dtH;
            }

            p = clamp(p, 0, ch.max_kw);

            ch.meas.power_kw = p;
            ch.meas.status = (p > 0.01) ? 'Charging' : 'Suspended';

            // Update energy and SoC
            const eff = 0.96;
            const deliveredKwh = p * dtH * eff;
            ch.meas.energy_kwh = clamp(ch.meas.energy_kwh + deliveredKwh, 0, 1000000);

            const deltaSocVeh = (deliveredKwh / capKwh) * 100;
            ch.vehicle.soc_pct = clamp(ch.vehicle.soc_pct + deltaSocVeh, 0, 100);

            evTotalPower += p;
            evTotalEnergy += ch.meas.energy_kwh;
        }

// Base load with slight noise
        const baseNoise = clamp(this._rng.normal(0, 0.3), -1.0, 1.0);
        const baseLoad = clamp(this._model.grid.base_load_kw + baseNoise, 0, 5000);

        // Grid power: + import, - export
        // net = consumption - generation - storageDischarge
        // storagePower is + discharge (generation), - charge (consumption)
        let gridPower = baseLoad + this._model.heatpump.power_kw + evTotalPower
            - this._model.pv.power_kw - this._model.chp.power_kw - this._model.generator.power_kw
            - this._model.storage.power_kw;

        if (!this._model.grid.available) {
            // Grid down: show 0 power and force over_limit false
            gridPower = 0;
        }

        this._model.grid.power_kw = gridPower;
        this._model.grid.over_limit = this._model.grid.available && (gridPower > this._model.grid.limit_kw + 1e-6);

        // Publish states (reduced churn)
        await this._publish(now);
        await this._publishAggregates(evTotalPower, evTotalEnergy);
    }

    async _publish(now) {
        // Scenario
        const elapsed = (this._scenario.running && this._scenario.started_ms) ? ((Date.now() - this._scenario.started_ms) / 1000) : 0;
        await this._setChanged('scenario.selected', this._scenario.selected);
        await this._setChanged('scenario.active', this._scenario.active);
        await this._setChanged('scenario.running', this._scenario.running);
        await this._setChanged('scenario.phase', this._scenario.phase);
        await this._setChanged('scenario.elapsed_s', Number(elapsed.toFixed(1)));
        await this._setChanged('scenario.status', this._scenario.status);

        // Grid
        await this._setChanged('grid.available', this._model.grid.available);
        await this._setChanged('grid.limit_kw', this._model.grid.limit_kw);
        await this._setChanged('grid.base_load_kw', this._model.grid.base_load_kw);
        await this._setChanged('grid.power_kw', Number(this._model.grid.power_kw.toFixed(3)));
        await this._setChanged('grid.over_limit', this._model.grid.over_limit);

        // Tariff
        await this._setChanged('tariff.mode', this._model.tariff.mode);
        await this._setChanged('tariff.price_ct_per_kwh', Number(this._model.tariff.price_ct_per_kwh.toFixed(2)));
        await this._setChanged('tariff.price_next_24h_json', this._model.tariff.price_next_24h_json);

        // PV
        await this._setChanged('pv.installed_kwp', this._model.pv.installed_kwp);
        await this._setChanged('pv.weather_factor', Number(this._model.pv.weather_factor.toFixed(3)));
        await this._setChanged('pv.power_kw', Number(this._model.pv.power_kw.toFixed(3)));
        await this._setChanged('pv.override.enabled', this._model.pv.override.enabled);
        await this._setChanged('pv.override.power_kw', Number(this._model.pv.override.power_kw.toFixed(3)));

        // Storage
        await this._setChanged('storage.capacity_kwh', this._model.storage.capacity_kwh);
        await this._setChanged('storage.max_charge_kw', this._model.storage.max_charge_kw);
        await this._setChanged('storage.max_discharge_kw', this._model.storage.max_discharge_kw);
        await this._setChanged('storage.soc_pct', Number(this._model.storage.soc_pct.toFixed(3)));
        await this._setChanged('storage.power_kw', Number(this._model.storage.power_kw.toFixed(3)));
        await this._setChanged('storage.ctrl.enabled', this._model.storage.ctrl.enabled);
        await this._setChanged('storage.ctrl.power_set_kw', Number((Number(this._model.storage.ctrl.power_set_kw) || 0).toFixed(3)));

        // Heatpump/CHP/Generator
        for (const dev of ['heatpump', 'chp', 'generator']) {
            await this._setChanged(`${dev}.power_kw`, Number(this._model[dev].power_kw.toFixed(3)));
            await this._setChanged(`${dev}.ctrl.enabled`, this._model[dev].ctrl.enabled);
            await this._setChanged(`${dev}.ctrl.power_set_kw`, Number((Number(this._model[dev].ctrl.power_set_kw) || 0).toFixed(3)));
        }

        // EVCS per charger
        for (const ch of this._model.evcs.chargers) {
            const base = `evcs.${ch.id}`;
            await this._setChanged(`${base}.ctrl.enabled`, ch.ctrl.enabled);
            await this._setChanged(`${base}.ctrl.limit_kw`, Number(ch.ctrl.limit_kw.toFixed(3)));
            await this._setChanged(`${base}.ctrl.plugged`, ch.ctrl.plugged);
            await this._setChanged(`${base}.ctrl.priority`, ch.ctrl.priority);

            await this._setChanged(`${base}.vehicle.id`, ch.vehicle.id);
            await this._setChanged(`${base}.vehicle.soc_pct`, Number(ch.vehicle.soc_pct.toFixed(3)));
            await this._setChanged(`${base}.vehicle.capacity_kwh`, Number(ch.vehicle.capacity_kwh.toFixed(3)));
            await this._setChanged(`${base}.vehicle.max_charge_kw`, Number(ch.vehicle.max_charge_kw.toFixed(3)));
            await this._setChanged(`${base}.vehicle.target_soc_pct`, Number(ch.vehicle.target_soc_pct.toFixed(3)));
            await this._setChanged(`${base}.vehicle.departure_time`, ch.vehicle.departure_time);

            await this._setChanged(`${base}.meas.status`, ch.meas.status);
            await this._setChanged(`${base}.meas.power_kw`, Number(ch.meas.power_kw.toFixed(3)));
            await this._setChanged(`${base}.meas.energy_kwh`, Number(ch.meas.energy_kwh.toFixed(6)));
        }
    }

    async _publishAggregates(evTotalPower, evTotalEnergy) {
        await this._setChanged('evcs.count', this._model.evcs.chargers.length);
        await this._setChanged('evcs.total_power_kw', Number(evTotalPower.toFixed(3)));
        await this._setChanged('evcs.total_energy_kwh', Number(evTotalEnergy.toFixed(6)));
    }

    async _setChanged(relId, val) {
        const key = relId;
        const prev = this._cache.get(key);

        // Normalize NaN / undefined
        const normalized = (typeof val === 'number' && Number.isNaN(val)) ? null : val;

        // For numbers: compare with small epsilon; for others: strict
        let changed = false;
        if (typeof normalized === 'number' && typeof prev === 'number') {
            changed = Math.abs(normalized - prev) > 1e-6;
        } else {
            changed = normalized !== prev;
        }

        if (changed) {
            this._cache.set(key, normalized);
            await this.setStateAsync(relId, { val: normalized, ack: true });
        }
    }

    async _ensureChannel(id, name) {
        await this.setObjectNotExistsAsync(id, {
            type: 'channel',
            common: { name },
            native: {},
        });
    }

    async _ensureState(id, common) {
        const obj = {
            type: 'state',
            common: {
                name: id,
                ...common,
            },
            native: {},
        };
        await this.setObjectNotExistsAsync(id, obj);

        // Only apply default value if the state has no persisted value yet.
        if (common.def !== undefined) {
            const existing = await this.getStateAsync(id);
            if (!existing || existing.val === null || existing.val === undefined) {
                await this.setStateAsync(id, { val: common.def, ack: true });
            }
        }
    }
}

if (module.parent) {
    // Export for tests
    module.exports = (options) => new NexowattSimAdapter(options);
} else {
    // Start directly
    new NexowattSimAdapter();
}