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

            this._rng = new LcgRandom(cfg.randomSeed);

            this.log.info(`Starting simulator with ${cfg.chargersCount} EVCS, interval=${cfg.updateIntervalMs}ms`);

            await this._createObjects(cfg);
            await this._initModel(cfg);

            // Subscribe to commands and manual overrides
            await this.subscribeStatesAsync('*.ctrl.*');
            await this.subscribeStatesAsync('*.vehicle.*');
            await this.subscribeStatesAsync('grid.*');
            await this.subscribeStatesAsync('pv.*');
            await this.subscribeStatesAsync('storage.*');
            await this.subscribeStatesAsync('tariff.*');

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
            this._model.tariff.price_ct_per_kwh = clamp(value, 0, 500);
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

    async _createObjects(cfg) {
        // Root channels
        await this._ensureChannel('grid', 'Grid');
        await this._ensureChannel('pv', 'PV');
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

        // Grid states
        await this._ensureState('grid.available', { type: 'boolean', role: 'indicator.reachable', read: true, write: true, def: true });
        await this._ensureState('grid.limit_kw', { type: 'number', role: 'level', unit: 'kW', read: true, write: true, def: cfg.gridLimitKw, min: 1, max: 5000 });
        await this._ensureState('grid.base_load_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: true, def: cfg.baseLoadKw, min: 0, max: 5000 });
        await this._ensureState('grid.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });
        await this._ensureState('grid.over_limit', { type: 'boolean', role: 'indicator.alarm', read: true, write: false, def: false });

        // Tariff
        await this._ensureState('tariff.mode', { type: 'string', role: 'text', read: true, write: true, def: 'auto' });
        await this._ensureState('tariff.price_ct_per_kwh', { type: 'number', role: 'value.price', unit: 'ct/kWh', read: true, write: true, def: 30 });
        await this._ensureState('tariff.price_next_24h_json', { type: 'string', role: 'json', read: true, write: false, def: '[]' });

        // PV
        await this._ensureState('pv.installed_kwp', { type: 'number', role: 'value', unit: 'kWp', read: true, write: true, def: cfg.pvInstalledKwp, min: 0, max: 5000 });
        await this._ensureState('pv.weather_factor', { type: 'number', role: 'level', unit: '', read: true, write: true, def: cfg.pvWeatherFactor, min: 0, max: 1 });
        await this._ensureState('pv.power_kw', { type: 'number', role: 'value.power', unit: 'kW', read: true, write: false, def: 0 });

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

        const storageCapacity = clamp(await get('storage.capacity_kwh', cfg.storageCapacityKwh), 1, 50000);
        const storageMaxCh = clamp(await get('storage.max_charge_kw', cfg.storageMaxChargeKw), 0, 50000);
        const storageMaxDis = clamp(await get('storage.max_discharge_kw', cfg.storageMaxDischargeKw), 0, 50000);
        const storageSoc = clamp(await get('storage.soc_pct', cfg.storageInitialSocPct), 0, 100);
        const storageEnabled = !!(await get('storage.ctrl.enabled', true));
        const storageSet = Number(await get('storage.ctrl.power_set_kw', 0)) || 0;

        const tariffMode = String(await get('tariff.mode', 'auto'));
        const tariffPrice = clamp(await get('tariff.price_ct_per_kwh', 30), 0, 500);

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
            pv: { installed_kwp: pvInstalled, weather_factor: pvWeather, power_kw: 0 },
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
            this._model.tariff.price_ct_per_kwh = clamp(daily + noise, 0, 500);
        } else {
            // Manual mode: keep current price as baseline and use a flat curve
            base = clamp(this._model.tariff.price_ct_per_kwh ?? base, 0, 500);
            amp = 0;
            this._model.tariff.price_ct_per_kwh = base;
        }

        // Build a simple 24h forward curve (hourly) for EMS tests
        const curve = [];
        for (let h = 0; h < 24; h++) {
            const d = new Date(nowMs + h * 3600 * 1000);
            const hh = d.getHours() + d.getMinutes() / 60;
            const v = base + amp * Math.sin((2 * Math.PI * (hh - 7)) / 24);
            curve.push(Number(clamp(v, 0, 500).toFixed(2)));
        }
        this._model.tariff.price_next_24h_json = JSON.stringify(curve);

        // PV
        const pvProfile = solarProfile01(now);
        const pvNoise = clamp(this._rng.normal(0, 0.02), -0.05, 0.05);
        this._model.pv.power_kw = clamp(
            this._model.pv.installed_kwp * this._model.pv.weather_factor * pvProfile * (1 + pvNoise),
            0,
            Math.max(0, this._model.pv.installed_kwp)
        );

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