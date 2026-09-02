// RCX RTK Base — software architecture knowledge graph
//
// Companion to RCX_RTK_Datalogger_software_architecture.cypher (the rover) and to the
// shared wiring graph; the two boards are the same hardware. This graph models the
// running base-station software: translation units, the two cores and what runs on
// each, the resources consumed, the buffers and queues, the data flows, the state
// machines, the gates that decide whether a correction byte may leave the device, the
// receiver-command sequences and the probe caches they leave behind, the budgets, the
// instrumentation and its gaps, the standing rules, the failure modes observed in the
// field, the recovery levers, and the triage runbooks.
//
// Provenance convention. Every quantity carries a `source` property:
//     'code'      — read directly out of the firmware
//     'measured'  — read out of a field capture (SD status/gps/event CSVs, serial log)
//     'derived'   — computed from code or measured values
//     'estimate'  — not yet measured; treat as a hypothesis, not a fact
//     'unknown'   — the value is not currently observable
// A quantity that cannot be observed is an InstrumentationGap node, not an absent
// property. Field sessions are cited by their SD session number (NNNN).
//
// Two labels are specific to this graph and absent from the rover graph, because the
// base has failed through them repeatedly: Gate (a boolean that must hold before RTCM
// is published) and ProbeCache (a variable holding the receiver's last reply — a memory
// of the past, not a statement about the present). CommandSequence models the paced
// PQTM command sets, and Store models the two independent non-volatile stores that a
// reflash clears neither of.
//
// Units are named in the property (bytes, ms, s, hz, m) so nothing is ambiguous.


// ─────────────────────────────────────────────────────────────────────────────
// 1. System and execution substrate
// ─────────────────────────────────────────────────────────────────────────────

CREATE (sys:SoftwareSystem {id: 'rcx1_fw', name: 'RCX RTK Base firmware', platform: 'ESP32-S3 / Arduino-ESP32 on ESP-IDF', sketch: 'RCX_RTK_Base.ino', sketch_lines: 7430, binding_constraint: 'receiver state truth and internal SRAM', module_firmware: 'LG290P03AAN / R02A01S (from RTCM 1033)', source: 'code', notes: 'Mobile NTRIP base for autocross: LG290P in base mode emits RTCM3 on one UART; loop() frames and CRC-checks it and pushes every valid frame to each enabled NTRIP caster and to local rovers over the softAP. Position comes from a rover-mode PPP survey averaged under our own criteria, then locked into the module as a fixed base.'})

CREATE (core0:Core {id: 'core0', name: 'ESP32-S3 Core 0', role: 'SD logger only', hosts: 'bridgeSdLog task, plus library WiFi/lwIP tasks', source: 'code', notes: 'Nothing on this core may write NVS or touch casters[] — a flash commit disables the instruction cache on BOTH cores and stalls the UART drain on Core 1.'})
CREATE (core1:Core {id: 'core1', name: 'ESP32-S3 Core 1', role: 'everything real-time', hosts: 'Arduino loop(): UART drain, RTCM framing, caster TCP, local caster, synchronous web server, display, receiver command sequences', source: 'code', notes: 'Single-threaded by design. Every blocking call here is a blocking call on the 460800-baud GNSS drain unless it pumps processGnssSerial() itself.'})


// ─────────────────────────────────────────────────────────────────────────────
// 2. Resources and stores
// ─────────────────────────────────────────────────────────────────────────────

CREATE (r_isram:Resource {id: 'internal_sram', name: 'Internal SRAM heap', caps: 'MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT', scarcity: 'binding', warn_floor_bytes: 40000, relocatable: false, source: 'code', notes: 'Shared by the SD driver DMA buffers, lwIP sockets, the WiFi stack, every task stack, and every static buffer in the sketch. An 11-hour session ended with sdmmc_cmd reporting ESP_ERR_NO_MEM with 8 MB of PSRAM idle.'})
CREATE (r_dma:Resource {id: 'dma_pool', name: 'DMA-capable internal memory', caps: 'MALLOC_CAP_DMA', scarcity: 'binding', shares_pool_with: 'internal_sram', source: 'code', notes: 'A stricter view of the same pool. SDMMC transfer buffers and lwIP pbufs come from here.'})
CREATE (r_psram:Resource {id: 'psram', name: 'External PSRAM', size_bytes: 8388608, used_bytes: 0, scarcity: 'abundant', relocatable: true, source: 'code', notes: 'UNUSED. Nothing in this build allocates from it. The rover places every queue and the directory snapshot here; the base places nothing. See lever_psram_queue.'})
CREATE (r_flash:Resource {id: 'flash_rodata', name: 'Flash / rodata', scarcity: 'abundant', source: 'code', notes: 'Dashboard and PPP pages are served from PROGMEM with no RAM copy.'})
CREATE (r_uart1:Resource {id: 'uart1', name: 'UART1 to LG290P', baud: 460800, framing: '8N1', pins: 'RX GPIO4 / TX GPIO5', line_rate_bytes_per_s: 46080, rx_ring_bytes: 4096, ring_coverage_ms: 89, scarcity: 'abundant bandwidth, tight latency', source: 'code', notes: 'RTCM3 and NMEA/PQTM share this one line in both directions. Utilisation is low; the constraint is that the 4096-byte ring is 89 ms of data, so any loop pass longer than that without draining loses bytes.'})
CREATE (r_uart0:Resource {id: 'uart0', name: 'UART0 / USB CDC console', role: 'serial diagnostics', scarcity: 'abundant', hazard: 'With no host reading the port a write BLOCKS ~100 ms. A print on any periodic path is a periodic UART overrun. All periodic echo is behind SERIAL_GNSS_ECHO_ENABLE / SERIAL_GNSS_CMD_ECHO_ENABLE, both false.', source: 'measured', notes: 'Sessions 0442-0455 lost UART bytes continuously (219 RTCM CRC + 193 framing failures per hour); removing one per-response print took both to zero in 0457.'})
CREATE (r_sdbus:Resource {id: 'sd_bus', name: 'SDMMC 1-bit bus and FatFs volume', pins: 'CLK 14 / CMD 15 / D0 16 / D3 21 pulled up, not driven', mount_attempts: '20 MHz then 400 kHz', format_on_fail: false, scarcity: 'latency contended', stall_class_ms: 100, source: 'code', notes: 'FAT cluster-allocation stalls up to ~100 ms are absorbed by the Core-0 task and the row queue, never by loop().'})
CREATE (r_wifi:Resource {id: 'wifi_radio', name: 'WiFi radio (single, AP+STA)', mode: 'WIFI_AP_STA', ap_open: true, sta_networks_max: 10, scarcity: 'shared', source: 'code', notes: 'One radio serves the recovery AP and the STA uplink. AP channel-hop disruption while the STA scans is inherent to the ESP32, not a bug. WiFi.persistent(false) and setAutoReconnect(false) are load-bearing.'})
CREATE (r_lg290p:Resource {id: 'lg290p_receiver', name: 'Quectel LG290P receiver', fix_rate_hz: 1, fix_interval_measured_ms: 1000, modes: 'rover (1) / base (2); svin survey-in (1) / fixed (2)', separately_powered: true, source: 'measured', notes: 'The other computer in this system. Its mode changes without announcing itself; the only trustworthy evidence that a configuration applied is the output stream. PPP on this module requires the 1 Hz fix rate, which the base satisfies and the 20 Hz rover cannot.'})

CREATE (st_lg290p:Store {id: 'lg290p_nvm', name: 'LG290P retained configuration', medium: 'battery-backed, on the module', written_by: 'PQTMSAVEPAR (11 send sites, ack never checked)', holds: 'rcvrMode, svinMode + fixed ECEF, elevation/CNR masks, PPP enable, message rates, RTCM set', cleared_by: 'disconnecting the LG290P backup battery AND all power (field-proven); no ESP32-side action touches it', survives: 'ESP32 erase, reflash, PQTMHOT, sometimes PQTMSRR', source: 'measured', notes: 'The reason the base can publish a coordinate from a previous venue with the ESP32 completely blank, and why every mode change is read back rather than assumed.'})
CREATE (st_nvs:Store {id: 'esp_nvs', name: 'ESP32 NVS (Preferences)', medium: 'SPI flash', hazard: 'Every commit disables the instruction cache on both cores — never from Core 0, and never on a periodic path.', source: 'code'})
CREATE (ns_wifi:Store {id: 'nvs_rcx_wifi', name: 'rcx_wifi', kind: 'NVS namespace', keys: 'n, s0..s9, p0..p9', shared_with_rover: true, source: 'code', notes: 'Shared with the RCX RaceCapture rover so credentials survive a base/rover reflash. Do not rename.'})
CREATE (ns_pos:Store {id: 'nvs_rcx1pos', name: 'rcx1pos', kind: 'NVS namespace', keys: 'valid, lat, lon, alt, hacc, src, svsec, note', written_by: 'checkPppSurveyCompletion (src ppp / ppp-auto), serviceForceBase (src forced), /api/setpos (src manual)', cleared_by: 'resolvePosCheckMoved, /api/clearpos, FORCE_CLEAR_POSITION_ON_BOOT', source: 'code', notes: 'The coordinate the base will cast from on the next boot, with its provenance and assumed accuracy.'})
CREATE (ns_cast:Store {id: 'nvs_rcx1cast', name: 'rcx1cast', kind: 'NVS namespace', holds: 'user-added casters and per-caster enable flags', source: 'code'})
CREATE (ns_xbee:Store {id: 'nvs_xbee', name: 'xbee', kind: 'NVS namespace', keys: 'survey_sec, survey_acc', source: 'code', notes: 'Legacy name; survey window and accuracy limit.'})
CREATE (ns_blog:Store {id: 'nvs_bridgelog', name: 'bridgelog', kind: 'NVS namespace', holds: 'SD log channel enable flags, session counter', source: 'code'})
CREATE (ns_id:Store {id: 'nvs_rcx1id', name: 'rcx1id', kind: 'NVS namespace', holds: 'device name (MAC-derived on first boot), localcast enable', source: 'code'})
CREATE (ns_drv:Store {id: 'nvs_wifi_driver', name: 'WiFi driver internal NVS', kind: 'driver-owned flash store', disabled_by: 'WiFi.persistent(false)', source: 'measured', notes: 'By default every WiFi.begin() commits STA config here. With the 10 s rotation across out-of-range networks that was a flash commit every 10 s indefinitely — the confirmed root cause of the worst prior data-corruption regression.'})

MATCH (a {id: 'dma_pool'}), (b {id: 'internal_sram'}) CREATE (a)-[:SUBSET_OF {notes: 'Exhausting one exhausts the other.'}]->(b)
MATCH (a {id: 'lg290p_nvm'}), (b {id: 'lg290p_receiver'}) CREATE (a)-[:PART_OF {}]->(b)
MATCH (n:Store), (s {id: 'esp_nvs'}) WHERE n.kind = 'NVS namespace' CREATE (n)-[:PART_OF {}]->(s)
MATCH (a {id: 'nvs_wifi_driver'}), (b {id: 'esp_nvs'}) CREATE (a)-[:PART_OF {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 3. Modules — translation units and what they own
// ─────────────────────────────────────────────────────────────────────────────

CREATE (m_main:Module {id: 'mod_main', file: 'RCX_RTK_Base.ino', lines: 7430, role: 'setup ordering, loop() service order, UART drain, NMEA/PQTM and RTCM parsers, caster state machines, local caster, receiver command sequences, gates, boot position check, web handlers, display, temperatures', owns: 'casters[6], rtcmParser, rtcmStats, surveyStatus, every probe cache, every gate', hazard: 'Every function here runs on loop(). The 36 logEvent() sites pair a serial print with an SD event row so the two cannot drift apart.', source: 'code'})
CREATE (m_config:Module {id: 'mod_config', file: 'config.h', role: 'compile-time policy: survey defaults, BASE_SURVEY_USE_PPP, FORCE_CLEAR_POSITION_ON_BOOT, satellite masks, quality-gate limits, ban-safe reconnect floors, pins', source: 'code', notes: 'Kept verbatim; module tunables that would otherwise land here are defined in the module headers instead.'})
CREATE (m_sdlog:Module {id: 'mod_sdlog', file: 'bridge_sd_log.cpp / .h', role: 'owns ALL SD access: the Core-0 task, the row queue, the raw stream buffer, file lifecycle, rotation, channel enables', owns: 'q_sdlog_rows, sb_raw_rtcm, task_sdlog', source: 'code'})
CREATE (m_web:Module {id: 'mod_web_page', file: 'bridge_web_page.h', role: 'dashboard HTML/JS served from PROGMEM; polls /api/status at 1.5 s, /logs.json at 5 min, /api/wifilist at 10 s', source: 'code'})
CREATE (m_ppp:Module {id: 'mod_ppp_survey', file: 'ppp_survey.cpp / .h', role: 'rover-mode PPP survey state machine: paced configure, gated averaging, scatter and EPE convergence tests, echo detector, ECEF lock sequence', owns: 'PppSurveyState, 60-fix scatter window, acceptance criteria', source: 'code', notes: 'Command sequence is the field-proven 7-25-26 form and is not to be corrected against the spec: mode switches apply live there, and PQTMEPE takes no MsgVer in rover mode.'})
CREATE (m_pppweb:Module {id: 'mod_ppp_web', file: 'ppp_web.cpp / .h', role: '/ppp page and /api/ppp, /api/pppstart, /api/pppabort; duration parsed as hh:mm:ss with a bare number read as seconds', source: 'code'})
CREATE (m_tft:Module {id: 'mod_tft_setup', file: 'TFT_ESP32S3_LCD_147B_Setup.h', role: 'project-local TFT_eSPI setup for the ST7789 172x320 panel', hazard: 'Backlight GPIO46 is deliberately NOT declared to TFT_eSPI; the sketch owns it via LEDC. Two owners produce the IO 46 boot error.', source: 'code'})


// ─────────────────────────────────────────────────────────────────────────────
// 4. Tasks — two of ours, the rest library-managed
// ─────────────────────────────────────────────────────────────────────────────

CREATE (t_loop:Task {id: 'task_loop', name: 'loop', core: 'core1', priority: 1, stack_bytes: 8192, stack_high_water_free_bytes: null, instrumented: false, owner: 'Arduino core', source: 'code', notes: 'Stack is the Arduino default; the sketch does not set it. handleApiWifiList puts a 3072-byte buffer on it and handleDownload roughly 3 kB more, so the deep path is a web handler.'})
CREATE (t_sdlog:Task {id: 'task_sdlog', name: 'bridgeSdLog', core: 'core0', priority: 1, stack_bytes: 10240, stack_high_water_free_bytes: null, instrumented: false, source: 'code', notes: 'Drains the row queue with a 20 ms receive timeout, then the raw stream buffer; parks on a 20 ms delay while a download is active.'})
CREATE (t_wifi:Task {id: 'task_wifi_lwip', name: 'WiFi / lwIP / tcpip', core: 'library-managed', owner: 'ESP-IDF', instrumented: false, source: 'code'})

MATCH (a {id: 'task_loop'}),  (b {id: 'core1'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'core0'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'mod_sdlog'}),  (b {id: 'task_sdlog'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'mod_web_page'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_HANDLERS_ON {notes: 'WebServer is SYNCHRONOUS: server.handleClient() runs every route handler on loop(). A handler that blocks blocks the GNSS drain. Deferred work (reconfigure, force base, PPP start) is flagged and serviced by loop() for this reason.'}]->(b)
MATCH (a {id: 'mod_ppp_web'}),  (b {id: 'task_loop'}) CREATE (a)-[:RUNS_HANDLERS_ON {}]->(b)
MATCH (a {id: 'mod_ppp_survey'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_ON_TASK {notes: 'ppp_survey_tick() and ppp_survey_feed_nmea() are called from loop(); the module has no task of its own.'}]->(b)

MATCH (t:Task), (r {id: 'internal_sram'}) WHERE t.stack_bytes IS NOT NULL
CREATE (t)-[:CONSUMES {kind: 'task_stack', bytes: t.stack_bytes, relocatable: false, source: 'code'}]->(r)


// ─────────────────────────────────────────────────────────────────────────────
// 5. Boot ladder — setup() order, and how long each rung may block
// ─────────────────────────────────────────────────────────────────────────────

CREATE (s00:BringUpStage {id: 'stage_identity',   seq: 0,  label: 'serial, device name, suggested mount, localcast flag', blocking_bound_ms: 500, cost_bytes: null, source: 'code'})
CREATE (s01:BringUpStage {id: 'stage_temps',      seq: 1,  label: 'QMI8658 probe + die-sensor prime', blocking_bound_ms: 300, cost_bytes: null, source: 'code'})
CREATE (s02:BringUpStage {id: 'stage_bootline',   seq: 2,  label: 'boot diagnostic line: reset_reason, heap, psram, temps', blocking_bound_ms: 100, cost_bytes: null, source: 'code', notes: 'The only heap figure printed at boot. See gap_boot_heap_ladder.'})
CREATE (s03:BringUpStage {id: 'stage_prefs',      seq: 3,  label: 'survey prefs (xbee) + saved position (rcx1pos)', cost_bytes: null, source: 'code'})
CREATE (s04:BringUpStage {id: 'stage_display',    seq: 4,  label: 'display init + boot splash', cost_bytes: null, source: 'code'})
CREATE (s05:BringUpStage {id: 'stage_sdlog',      seq: 5,  label: 'SD mount, row queue, raw stream buffer, Core-0 task', blocking_bound_ms: 1000, cost_bytes: 80480, cost_note: 'queue 53856 + stream buffer 16384 + task stack 10240, before SDMMC driver buffers', source: 'derived'})
CREATE (s06:BringUpStage {id: 'stage_uart',       seq: 6,  label: 'GnssSerial ring 4096 + begin 460800', cost_bytes: 4096, source: 'code'})
CREATE (s07:BringUpStage {id: 'stage_forceclear', seq: 7,  label: 'FORCE_CLEAR_POSITION_ON_BOOT (compile-time, normally off)', blocking_bound_ms: 5250, cost_bytes: 0, source: 'code', notes: 'Wipes rcx1pos AND commands the module back to survey-in with PQTMSRR — the only path that clears both stores together.'})
CREATE (s08:BringUpStage {id: 'stage_satgating',  seq: 8,  label: 'applySatelliteGating: write 5 deg / 10 dB-Hz, read both back', blocking_bound_ms: 1000, cost_bytes: 0, source: 'code', notes: 'Must precede any fix collection. Masks live in the module NVM; a module that was ever a rover keeps the tight 30 deg / 32 dB-Hz pair until overwritten.'})
CREATE (s09:BringUpStage {id: 'stage_gnsscfg',    seq: 9,  label: 'configureLg290pBaseOnce: rover-first escape, verify, telemetry rates', blocking_bound_ms: 18250, cost_bytes: 0, source: 'derived', notes: 'Rover-first path: 3 x 250 ms spacing + 4500 ms PQTMHOT wait + up to 3000 ms verify (8250), doubled if PQTMSRR escalation is needed, plus 1750 ms of telemetry-rate writes. The hot-start probe path (saved position confirmed) can wait up to 60 s for BOTH config replies.'})
CREATE (s10:BringUpStage {id: 'stage_wifi',       seq: 10, label: 'WiFi AP_STA, persistent(false), autoReconnect(false), softAP, first STA attempt', cost_bytes: null, source: 'code', notes: 'The rover measured AP bring-up at 52 kB and STA at 23 kB of internal SRAM; the base has never measured either.'})
CREATE (s11:BringUpStage {id: 'stage_routes',     seq: 11, label: '28 routes registered + server.begin()', cost_bytes: null, source: 'code'})
CREATE (s12:BringUpStage {id: 'stage_casters',    seq: 12, label: 'loadCasters from NVS, states to WaitingForWifi/Disabled', cost_bytes: 0, source: 'code'})

MATCH (a:BringUpStage), (b:BringUpStage) WHERE b.seq = a.seq + 1 CREATE (a)-[:NEXT {}]->(b)
MATCH (s:BringUpStage), (r {id: 'internal_sram'}) WHERE s.cost_bytes > 0
CREATE (s)-[:CONSUMES {kind: 'bring_up', bytes: s.cost_bytes, source: s.source}]->(r)
MATCH (a {id: 'stage_sdlog'}),     (b {id: 'mod_sdlog'})      CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_gnsscfg'}),   (b {id: 'mod_main'})       CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_satgating'}), (b {id: 'lg290p_nvm'})     CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'stage_gnsscfg'}),   (b {id: 'lg290p_nvm'})     CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'stage_forceclear'}), (b {id: 'nvs_rcx1pos'})   CREATE (a)-[:CLEARS {}]->(b)
MATCH (a {id: 'stage_forceclear'}), (b {id: 'lg290p_nvm'})    CREATE (a)-[:CLEARS {method: 'PQTMCFGRCVRMODE,W,1 + PQTMCFGSVIN,W,1 + PQTMSAVEPAR + PQTMSRR'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 6. Buffers and queues — all in internal SRAM, none in PSRAM
// ─────────────────────────────────────────────────────────────────────────────

CREATE (q_rows:Queue {id: 'q_sdlog_rows', name: 'SD log row queue (LogLine)', depth: 96, item_bytes: 561, bytes: 53856, placement: 'internal', producer_rate_rows_per_s: 30, coverage_s: 3.2, on_full: 'drop, counted in g_dropped (log_drop column)', source: 'derived', notes: 'LogLine is tag + text[560]. The header comment sizing the queue at ~353 bytes per row is stale: at depth 192 x 352 the queue held ~68 kB and the session ended in ESP_ERR_NO_MEM; halving the depth while widening the line to 560 recovered only ~14 kB. See lever_psram_queue.'})
CREATE (sb_raw:Buffer {id: 'sb_raw_rtcm', name: 'Raw RTCM stream buffer', bytes: 16384, placement: 'internal', kind: 'FreeRTOS stream buffer', drained_by: 'task_sdlog, 512-byte reads, flushed at 1 s', enabled_by_default: false, source: 'code'})
CREATE (b_uartrx:Buffer {id: 'buf_uart1_rx', name: 'UART1 RX ring', bytes: 4096, placement: 'internal', coverage_ms: 89, source: 'code', notes: 'Default 256 bytes is 5.5 ms. Highest-ROI line in the sketch. g_uartHighWater samples occupancy before each drain.'})
CREATE (b_casters:Buffer {id: 'buf_casters', name: 'casters[6] (NtripTarget)', slots: 6, per_slot_bytes: 3400, bytes: 20400, placement: 'internal static', source: 'derived', notes: 'Per slot: txBuf 1024 + responseHeader 1024 + lastResponseHeader 1024 + host/mount/password 144 + counters and a WiFiClient. Fixed array, never resized — WiFiClient members are not safely movable.'})
CREATE (b_txcap:Buffer {id: 'buf_caster_tx_capture', name: 'Caster TX capture rings', slots: 2, bytes: 8192, placement: 'internal static', served_by: '/caster0/raw, /caster1/raw', source: 'code', notes: 'Reduced from a 49 kB pair; the authoritative raw capture is rtcm_NNNN.bin on the card.'})
CREATE (b_validcap:Buffer {id: 'buf_rtcm_valid_capture', name: 'Valid RTCM capture ring', bytes: 8192, placement: 'internal static', served_by: '/rtcm.raw', holds_about_s: 60, source: 'code'})
CREATE (b_parser:Buffer {id: 'buf_rtcm_parser', name: 'RTCM frame assembly buffer', bytes: 1029, placement: 'internal static', source: 'code', notes: '3 + 1023 payload + 3 CRC. Frames over 1023 bytes are framing failures by construction.'})
CREATE (b_local:Buffer {id: 'buf_local_clients', name: 'LocalCasterClient[4]', slots: 4, header_bytes: 512, bytes: 2400, placement: 'internal static', source: 'derived', notes: 'Header was 160 bytes; a real NTRIP request with an Authorization header is ~168, so every authenticating rover was dropped deterministically. 512 now.'})
CREATE (b_status:Buffer {id: 'buf_api_status', name: '/api/status JSON buffer', bytes: 5632, placement: 'internal static', poll_interval_ms: 1500, source: 'code'})
CREATE (b_pppjson:Buffer {id: 'buf_api_ppp', name: '/api/ppp JSON buffer', bytes: 640, placement: 'task_loop stack', source: 'code', notes: 'ppp_survey_status_json() blanks the whole buffer rather than emitting a truncated object.'})
CREATE (b_wifilist:Buffer {id: 'buf_wifilist', name: '/api/wifilist JSON buffer', bytes: 3072, placement: 'task_loop stack', source: 'code'})
CREATE (b_dl:Buffer {id: 'buf_download', name: 'download chunk + tar header + zero pad', bytes: 2996, placement: 'task_loop stack', source: 'code', notes: '1460-byte chunk, 512-byte tar header, 1024-byte end-of-archive. Streams SD files from loop() with dlPump() between chunks.'})
CREATE (b_nmea:Buffer {id: 'buf_nmea_line', name: 'NMEA sentence buffer', bytes: 180, placement: 'internal static', source: 'code'})
CREATE (b_typestats:Buffer {id: 'buf_rtcm_type_stats', name: 'RtcmTypeStat[32]', bytes: 384, placement: 'internal static', source: 'code', notes: 'Per-type count and last-seen; the basis of isRtcmBaseActive(), the missing-types list and the output watchdog.'})
CREATE (b_logscache:Buffer {id: 'buf_logs_cache', name: '/logs.json String cache', bytes: null, placement: 'internal heap (String)', ttl_ms: 300000, source: 'code', notes: 'Rebuilt at most every 5 minutes so the directory scan is not paid on every dashboard load.'})

MATCH (b), (r {id: 'internal_sram'}) WHERE (b:Queue OR b:Buffer) AND b.placement IN ['internal', 'internal static', 'internal heap (String)'] AND b.bytes IS NOT NULL
CREATE (b)-[:ALLOCATED_IN {bytes: b.bytes, relocatable: b.id IN ['q_sdlog_rows', 'sb_raw_rtcm', 'buf_caster_tx_capture', 'buf_rtcm_valid_capture'], source: b.source}]->(r)
MATCH (b:Buffer), (t {id: 'task_loop'}) WHERE b.placement = 'task_loop stack' CREATE (b)-[:ALLOCATED_IN {bytes: b.bytes, transient: true, source: 'code'}]->(t)
MATCH (a {id: 'q_sdlog_rows'}),  (b {id: 'mod_sdlog'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sb_raw_rtcm'}),   (b {id: 'mod_sdlog'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'buf_casters'}),   (b {id: 'mod_main'})  CREATE (a)-[:OWNED_BY {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 7. Concurrency — there are no locks; the queue and the stream buffer are the
//    only cross-core channels, and everything else belongs to loop()
// ─────────────────────────────────────────────────────────────────────────────

CREATE (cc_queue:CrossCoreChannel {id: 'xcore_rows', name: 'Row queue, Core 1 -> Core 0', primitive: 'xQueueSend(0 timeout) / xQueueReceive(20 ms)', safe: true, source: 'code'})
CREATE (cc_raw:CrossCoreChannel {id: 'xcore_raw', name: 'Raw RTCM stream buffer, Core 1 -> Core 0', primitive: 'xStreamBufferSend(0) / xStreamBufferReceive(0)', safe: true, source: 'code'})
CREATE (cc_flags:CrossCoreChannel {id: 'xcore_flags', name: 'volatile channel enables + g_downloadActive + g_dropped', primitive: 'volatile bool/uint32', safe: 'benign torn reads: at worst one row logged or skipped', source: 'code', notes: 'g_downloadActive MUST be paired: set true before streaming, false on every exit path, or the SD task parks forever.'})

MATCH (a {id: 'xcore_rows'}), (b {id: 'q_sdlog_rows'}) CREATE (a)-[:IMPLEMENTED_BY {}]->(b)
MATCH (a {id: 'xcore_raw'}),  (b {id: 'sb_raw_rtcm'})  CREATE (a)-[:IMPLEMENTED_BY {}]->(b)
MATCH (c:CrossCoreChannel), (t {id: 'task_loop'})  CREATE (t)-[:PRODUCES_ON {}]->(c)
MATCH (c:CrossCoreChannel), (t {id: 'task_sdlog'}) CREATE (t)-[:CONSUMES_FROM {}]->(c)


// ─────────────────────────────────────────────────────────────────────────────
// 8. Data flows
// ─────────────────────────────────────────────────────────────────────────────

CREATE (f_nmea:DataFlow {id: 'flow_nmea_pqtm', name: 'NMEA + PQTM from the receiver', transport: 'uart1', direction: 'LG290P -> ESP32', sentences: 'GGA, RMC, GSV at 1 Hz; PQTMPVT, PQTMEPE, PQTMSVINSTATUS (base mode only) ~3 lines/s; PQTMPPPNAV requested in rover mode; config replies', rate_hz: 1, source: 'code', notes: 'Checksum-failed sentences are DROPPED, split into real corruption (printable 5-char header) versus framer desync (a 0x24 byte inside RTCM, ~91/hr, benign).'})
CREATE (f_rtcm_in:DataFlow {id: 'flow_rtcm_from_module', name: 'RTCM3 from the receiver', transport: 'uart1', direction: 'LG290P -> ESP32', burst: 'whole message set within ~3 ms at each epoch, then ~1000 ms silence', frames_per_s_measured: 5.09, frames_per_s_sigma: 0.29, bytes_per_s_measured: 758, types: '1005/1033/1230 at 10 s; 1019/1020/1042/1046 at 1 s; MSM4 1074/1084/1094/1124 via PQTMCFGRTCM', source: 'measured', notes: 'The burst shape is why a 1 s rate window read 0..31 fps on a true 5.09 fps stream (0454) and why a freshly authenticated caster is idle for up to a full second unless frames are staged (0043).'})
CREATE (f_cmd:DataFlow {id: 'flow_pqtm_cmd', name: 'PQTM commands to the receiver', transport: 'uart1', direction: 'ESP32 -> LG290P', spacing_ms: 250, ack_checked: false, savepar_sites: 11, srr_sites: 7, hot_sites: 3, source: 'code', notes: 'Fire-and-forget at every site. The output stream is the only evidence a write applied; see rule_read_back and gap_pqtm_ack.'})
CREATE (f_src:DataFlow {id: 'flow_ntrip_source', name: 'NTRIP v1 SOURCE upload to casters', transport: 'tcp', port: 2101, direction: 'ESP32 -> caster', coalescing_bytes: 1024, flush_at_bytes: 512, max_hold_ms: 400, write_gate: 'select() writable, 30 ms bound, 100 ms recheck when congested', staged_during_handshake: true, source: 'code'})
CREATE (f_local:DataFlow {id: 'flow_local_ntrip', name: 'Local NTRIP caster to rovers over the softAP', transport: 'tcp server', port: 2101, direction: 'ESP32 -> rover', max_clients: 4, mount: 'Local-Wifi', header_timeout_ms: 5000, source: 'code', notes: 'The correction path a rover actually uses at an event with no internet. Counted as WiFi activity so the STA rotation cannot bounce the AP while a rover is being served.'})
CREATE (f_rows:DataFlow {id: 'flow_sd_rows', name: 'CSV rows to the SD task', transport: 'q_sdlog_rows', direction: 'loop -> task_sdlog', gps_hz: 1, sat_rows_per_s: 28, status_hz: 0.2, event: 'sporadic', rtcm_detail: 'per frame, opt-in', source: 'code'})
CREATE (f_raw:DataFlow {id: 'flow_raw_rtcm', name: 'Raw RTCM bytes to rtcm_NNNN.bin', transport: 'sb_raw_rtcm', direction: 'loop -> task_sdlog', opt_in: true, flush_ms: 1000, source: 'code'})
CREATE (f_snap:DataFlow {id: 'flow_snapshot', name: 'BridgeLogSnapshot', transport: 'struct built in serviceLogging()', rate_hz: 5, fields: 'fix, EPE 2D/3D, ECEF, svin state, pos source/accuracy, hot start, heap free/largest/min, loop max, uart high water, integrity counters, temps, two caster slots, pos-check state', source: 'code', notes: 'The logger owns nothing in the sketch; loop() hands it this struct.'})
CREATE (f_http:DataFlow {id: 'flow_http_dashboard', name: 'Dashboard polling', transport: 'tcp_80', direction: 'browser <-> ESP32', status_poll_ms: 1500, ppp_poll_ms: 1500, logs_poll_ms: 300000, wifilist_poll_ms: 10000, source: 'code'})
CREATE (f_dl:DataFlow {id: 'flow_download', name: 'Log download and tar export', transport: 'tcp_80', direction: 'SD -> browser', chunk_bytes: 1460, pumps_gnss_between_chunks: true, pauses_sd_task: true, source: 'code', notes: 'The one place loop() reads the card directly. Single reader is enforced by g_downloadActive, not a lock.'})
CREATE (f_serial:DataFlow {id: 'flow_serial_console', name: 'Serial diagnostics', transport: 'uart0', direction: 'ESP32 -> host', gated: true, source: 'code'})
CREATE (f_lcd:DataFlow {id: 'flow_lcd', name: 'LCD status', transport: 'SPI 40 MHz', rate_hz: 1, source: 'code'})

MATCH (a {id: 'mod_main'}),       (b {id: 'flow_nmea_pqtm'})     CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_ppp_survey'}), (b {id: 'flow_nmea_pqtm'})     CREATE (a)-[:CONSUMES_FLOW {notes: 'GGA, PQTMEPE, GAGSV signal 5 only'}]->(b)
MATCH (a {id: 'mod_sdlog'}),      (b {id: 'flow_nmea_pqtm'})     CREATE (a)-[:CONSUMES_FLOW {notes: 'GSV only, for sat_NNNN.csv'}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_rtcm_from_module'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_pqtm_cmd'})      CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_ppp_survey'}), (b {id: 'flow_pqtm_cmd'})      CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_ntrip_source'})  CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_local_ntrip'})   CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_sd_rows'})       CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_sdlog'}),      (b {id: 'flow_sd_rows'})       CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_sdlog'}),      (b {id: 'flow_raw_rtcm'})      CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_snapshot'})      CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_sdlog'}),      (b {id: 'flow_snapshot'})      CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_web_page'}),   (b {id: 'flow_http_dashboard'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),       (b {id: 'flow_download'})      CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'flow_nmea_pqtm'}),        (b {id: 'uart1'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'flow_rtcm_from_module'}), (b {id: 'uart1'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'flow_pqtm_cmd'}),         (b {id: 'uart1'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'flow_ntrip_source'}),     (b {id: 'wifi_radio'}) CREATE (a)-[:TRAVERSES {side: 'STA'}]->(b)
MATCH (a {id: 'flow_local_ntrip'}),      (b {id: 'wifi_radio'}) CREATE (a)-[:TRAVERSES {side: 'AP'}]->(b)
MATCH (a {id: 'flow_http_dashboard'}),   (b {id: 'wifi_radio'}) CREATE (a)-[:TRAVERSES {side: 'AP or STA'}]->(b)
MATCH (a {id: 'flow_download'}),         (b {id: 'sd_bus'})   CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'flow_serial_console'}),   (b {id: 'uart0'})    CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'flow_rtcm_from_module'}), (b {id: 'flow_ntrip_source'}) CREATE (a)-[:FEEDS {via: 'handleValidRtcmFrame -> sendFrameToCaster'}]->(b)
MATCH (a {id: 'flow_rtcm_from_module'}), (b {id: 'flow_local_ntrip'})  CREATE (a)-[:FEEDS {via: 'handleValidRtcmFrame -> localCasterBroadcast'}]->(b)
MATCH (a {id: 'flow_rtcm_from_module'}), (b {id: 'flow_raw_rtcm'})     CREATE (a)-[:FEEDS {}]->(b)
MATCH (a {id: 'flow_pqtm_cmd'}),         (b {id: 'lg290p_nvm'})        CREATE (a)-[:WRITES {when: 'PQTMSAVEPAR'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 9. State machines
// ─────────────────────────────────────────────────────────────────────────────

CREATE (sm_caster:StateMachine {id: 'sm_caster', name: 'CasterState (per caster)', states: ['Disabled', 'Held', 'WaitingForWifi', 'Connecting', 'AwaitingResponse', 'Authenticated', 'Streaming', 'Error'], instances: 'casters[0..casterCount-1]', watchdogs: 'response 5 s; auth-stall 12 s; uplink-stall 8 s (only when behind and not congested); congestion max 20 s', backoff: '30 s base doubling per failure, cap 600 s; Held never counts as a failure', source: 'code', notes: 'Held is the operator-enabled-but-gate-withheld state, distinct from Disabled, so the log can say WHY nothing is casting. A stall over 2 s slides every caster timestamp forward, clamped to now, so all watchdogs do not fire at once on resume (0450 logged c0_age_s = 4294963 before the clamp).'})
CREATE (sm_rtcm:StateMachine {id: 'sm_rtcm_parser', name: 'RtcmParseState', states: ['WaitPreamble', 'ReadLen1', 'ReadLen2', 'ReadFrame'], validation: 'CRC-24Q over the whole frame; bad CRC is logged to the detail CSV with crc_ok=0', source: 'code'})
CREATE (sm_ppp:StateMachine {id: 'sm_ppp_survey', name: 'PppSurveyState', states: ['IDLE', 'CONFIGURING', 'SURVEYING', 'LOCKING', 'DONE', 'FAILED'], configure_steps: 8, step_interval_ms: 250, lock_steps: 5, post_reset_settle_ms: 5000, failure_exits: 'echoFault (identical fixes: still fixed base) / no valid fix at all', source: 'code', notes: 'Accepts a fix into the converged mean when EITHER the 60-fix scatter sigma <= 0.25 m OR EPE 2D <= 0.30 m and 3D <= 0.50 m. Locks the converged mean if >= 60 accepted fixes, else the unfiltered autonomous mean (src ppp-auto).'})
CREATE (sm_pos:StateMachine {id: 'sm_pos_check', name: 'PosCheckState (boot venue check)', states: ['Idle', 'Collecting', 'Confirmed', 'Moved', 'Timeout'], window_s_by_survey_len: '30 s (<1 h), 120 s (1-6 h), 300 s (>6 h)', min_fixes: 10, idle_timeout_ms: 120000, rover_settle_ms: 15000, move_thresh_m: 5.0, gross_move_m: 5000, echo_detector: 'identical fixes -> Timeout', source: 'code', notes: 'If the module is fixed when the check starts it is forced to rover first (rover + survey targets + SAVEPAR + PQTMHOT) so the fixes are independent — a fixed base echoes its own coordinate. That escape is NOT verified by read-back at this site; the echo detector is the backstop.'})
CREATE (sm_quality:StateMachine {id: 'sm_cast_quality', name: 'Correction quality verdict', states: ['ok', 'fault'], hold_ms: 20000, recover_ms: 5000, source: 'code', notes: 'Hysteretic: a fault must persist 20 s before the stream is cut and clear for 5 s before it resumes, because dropping a caster socket costs a reconnect and counts against rtk2go ban thresholds.'})
CREATE (sm_wifi:StateMachine {id: 'sm_wifi_rotation', name: 'STA rotation', states: ['connected', 'attempting index i', 'paused for AP occupancy'], attempt_timeout_ms: 10000, down_debounce_ms: 3000, ap_pause_window_ms: 60000, source: 'code', notes: 'Rotation pauses while a device on the AP has shown activity within 60 s — a dashboard write, a rover receiving RTCM, or joining the AP. Passive polling does not count.'})
CREATE (sm_mode:StateMachine {id: 'sm_lg290p_mode', name: 'Receiver mode (external, observed by probe)', states: ['rover + survey-in targets', 'rover + PPP', 'base + survey-in', 'base + fixed ECEF'], boot_policy: 'ALWAYS rover first; base only after a PPP lock or a confirmed saved position', transition_rule: 'a change OUT of rover needs PQTMCFGRCVRMODE,W + PQTMSAVEPAR + PQTMSRR; a bare write does not take; PQTMHOT is tried first and PQTMSRR is the escalation', source: 'measured', notes: 'This machine lives in the receiver. The firmware sees it only through replies to PQTMCFGRCVRMODE,R / PQTMCFGSVIN,R and through what the output stream contains.'})

MATCH (a {id: 'sm_caster'}),      (b {id: 'flow_ntrip_source'}) CREATE (a)-[:DRIVES {}]->(b)
MATCH (a {id: 'sm_rtcm_parser'}), (b {id: 'flow_rtcm_from_module'}) CREATE (a)-[:DRIVES {}]->(b)
MATCH (a {id: 'sm_ppp_survey'}),  (b {id: 'mod_ppp_survey'})    CREATE (a)-[:LIVES_IN {}]->(b)
MATCH (a {id: 'sm_pos_check'}),   (b {id: 'mod_main'})          CREATE (a)-[:LIVES_IN {}]->(b)
MATCH (a {id: 'sm_lg290p_mode'}), (b {id: 'lg290p_receiver'})   CREATE (a)-[:LIVES_IN {}]->(b)
MATCH (a {id: 'sm_pos_check'}),   (b {id: 'sm_ppp_survey'})     CREATE (a)-[:TRIGGERS {on: 'Moved / Timeout / echo -> g_pppSurveyPending'}]->(b)
MATCH (a {id: 'sm_pos_check'}),   (b {id: 'sm_lg290p_mode'})    CREATE (a)-[:COMMANDS {on: 'Confirmed -> g_reconfigPending -> base + fixed ECEF from rcx1pos'}]->(b)
MATCH (a {id: 'sm_ppp_survey'}),  (b {id: 'sm_lg290p_mode'})    CREATE (a)-[:COMMANDS {on: 'LOCKING -> base + fixed ECEF; DONE -> outputs restored + SAVEPAR'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 10. Gates — what must be true before a correction byte leaves the device
// ─────────────────────────────────────────────────────────────────────────────

CREATE (g_stream:Gate {id: 'gate_ready_to_stream', name: 'readyToStream', expression: 'surveyIsReady() AND venueConfirmed AND g_castQualityOk', evaluated_in: 'loop(), before serviceCaster()', on_false: 'stopCaster(Held) for every enabled caster', source: 'code'})
CREATE (g_ready:Gate {id: 'gate_base_ready', name: 'baseReady (latch)', expression: 'bypass (surveyInSkipped OR LG290P_USE_FIXED_BASE OR NVS-confirmed) OR (svinValidNow() == 2 AND meanAcc <= limit or unreported)', grace_ms: 60000, cleared_by: 'resetSurveyBookkeeping() only', source: 'code', notes: 'A set-only latch. Every path that abandons a coordinate must call resetSurveyBookkeeping() or the base carries the old verdict into the new survey (0022).'})
CREATE (g_venue:Gate {id: 'gate_venue_confirmed', name: 'venueConfirmed', expression: 'g_displacementCheckDone OR NOT g_savedPositionValid', source: 'code', notes: 'Holds ALL streaming while the boot check averages fixes against a saved position, because a hot-started fixed module is already emitting MSM from that position.'})
CREATE (g_quality:Gate {id: 'gate_cast_quality', name: 'g_castQualityOk', conditions_in_order: ['survey in progress (ppp_survey_active, authoritative, never stale)', 'receiver read as rover within 120 s (improves the reason only)', 'receiverHasSignal: GGA within 15 s AND (sats > 0 OR quality > 0)', 'isRtcmBaseActive: any MSM type seen within 25 s', 'CRC/framing failure rate <= 2 % over 30 s (min 50 candidates)', 'realistic reference accuracy <= 5 m (exempt under Force / fixed ECEF)'], hold_ms: 20000, recover_ms: 5000, source: 'code', notes: 'Answers a different question from baseReady: not did the survey converge, but is the stream leaving right now fit for a rover to steer by. Motivated by 0022, where casters sat authenticated on crtk advertising a mount producing nothing.'})
CREATE (g_svin:Gate {id: 'gate_svin_valid_now', name: 'svinValidNow()', expression: 'PQTMSVINSTATUS within 10 s, AND valid==2 demoted to 0 unless receiverHasSignal()', source: 'code', notes: 'PQTMSVINSTATUS is a base-mode-only output; the cache holds its last value forever once the receiver becomes a rover. And a hot-started fixed module re-asserts valid=2 at 1 Hz with no antenna, so clearing the cache is futile — the claim is corroborated instead.'})
CREATE (g_signal:Gate {id: 'gate_receiver_has_signal', name: 'receiverHasSignal()', expression: 'GGA seen within 15 s AND (satellites > 0 OR fixQuality > 0)', source: 'code'})
CREATE (g_msm:Gate {id: 'gate_msm_active', name: 'isRtcmBaseActive()', expression: 'any MSM observation type (1071-1137) last seen within 25 s', not_keyed_on: '1005/1006/1033, which the module emits during survey-in from an unconverged position', source: 'code'})
CREATE (g_fresh:Gate {id: 'gate_rcvr_mode_fresh', name: 'rcvrModeFresh', expression: 'g_probedRcvrModeMs set AND age < PROBE_MODE_TRUST_MS (120 s)', source: 'code'})
CREATE (g_wifi:Gate {id: 'gate_wifi_link', name: 'wifiLinkUp()', expression: 'WL_CONNECTED, or not continuously down for 3 s', recovery: 'instant', source: 'code', notes: 'One transient non-connected read used to drop both casters into 30 s backoff and advance the SSID rotation.'})
CREATE (g_writable:Gate {id: 'gate_caster_writable', name: 'casterWritable()', expression: 'select() write-ready within 30 ms', recheck_when_congested_ms: 100, on_false: 'hold the coalescing buffer; drop the OLDEST bytes if it fills', source: 'measured', notes: 'A {0,0} poll dropped 1.39 % of frames (0450) tracking frame count not byte volume — per-segment ACK timing, not congestion. availableForWrite() must not return as the gate: it reads 0 for both empty and full.'})
CREATE (g_hs:Gate {id: 'gate_handshake_accepted', name: 'handshakeAccepted', expression: 'ICY 200 OK or HTTP 200 seen', before_true: 'frames are STAGED into txBuf, never written', on_true: 'flush immediately', source: 'code', notes: 'Session 0043: crtk accepted and closed inside the same second because nothing was sent for up to an epoch. Staging closes that window.'})
CREATE (g_dl:Gate {id: 'gate_download_pause', name: 'g_downloadActive', expression: 'true while a download streams', effect: 'SD task parks; queue keeps accepting and dropping', source: 'code'})

MATCH (a {id: 'gate_ready_to_stream'}), (b {id: 'gate_base_ready'})      CREATE (a)-[:REQUIRES {}]->(b)
MATCH (a {id: 'gate_ready_to_stream'}), (b {id: 'gate_venue_confirmed'}) CREATE (a)-[:REQUIRES {}]->(b)
MATCH (a {id: 'gate_ready_to_stream'}), (b {id: 'gate_cast_quality'})    CREATE (a)-[:REQUIRES {}]->(b)
MATCH (a {id: 'gate_base_ready'}),      (b {id: 'gate_svin_valid_now'})  CREATE (a)-[:REQUIRES {unless: 'bypass'}]->(b)
MATCH (a {id: 'gate_svin_valid_now'}),  (b {id: 'gate_receiver_has_signal'}) CREATE (a)-[:REQUIRES {role: 'corroboration of valid==2'}]->(b)
MATCH (a {id: 'gate_cast_quality'}),    (b {id: 'gate_receiver_has_signal'}) CREATE (a)-[:REQUIRES {}]->(b)
MATCH (a {id: 'gate_cast_quality'}),    (b {id: 'gate_msm_active'})      CREATE (a)-[:REQUIRES {}]->(b)
MATCH (a {id: 'gate_cast_quality'}),    (b {id: 'gate_rcvr_mode_fresh'}) CREATE (a)-[:CONSULTS {notes: 'reason wording only; verdict never depends on the probe'}]->(b)
MATCH (a {id: 'gate_cast_quality'}),    (b {id: 'sm_ppp_survey'})        CREATE (a)-[:CONSULTS {notes: 'checked FIRST; a running survey is the reason and cannot go stale'}]->(b)
MATCH (a {id: 'gate_venue_confirmed'}), (b {id: 'sm_pos_check'})         CREATE (a)-[:RESOLVED_BY {}]->(b)
MATCH (a {id: 'sm_caster'}),            (b {id: 'gate_wifi_link'})       CREATE (a)-[:REQUIRES {}]->(b)
MATCH (a {id: 'flow_ntrip_source'}),    (b {id: 'gate_caster_writable'}) CREATE (a)-[:GATED_BY {}]->(b)
MATCH (a {id: 'flow_ntrip_source'}),    (b {id: 'gate_handshake_accepted'}) CREATE (a)-[:GATED_BY {}]->(b)
MATCH (a {id: 'flow_ntrip_source'}),    (b {id: 'gate_ready_to_stream'}) CREATE (a)-[:GATED_BY {}]->(b)
MATCH (a {id: 'flow_local_ntrip'}),     (b {id: 'gate_caster_writable'}) CREATE (a)-[:GATED_BY {notes: 'same select() poll, same 100 ms recheck'}]->(b)
MATCH (a {id: 'flow_sd_rows'}),         (b {id: 'gate_download_pause'})  CREATE (a)-[:GATED_BY {}]->(b)
MATCH (g:Gate), (m {id: 'mod_main'}) CREATE (g)-[:LIVES_IN {}]->(m)


// ─────────────────────────────────────────────────────────────────────────────
// 11. Probe caches — every variable that remembers what the receiver last said.
//     The recurring bug shape of this project: a cached reply read as a present truth.
// ─────────────────────────────────────────────────────────────────────────────

CREATE (pc_rm:ProbeCache {id: 'pc_rcvr_mode', variable: 'g_probedRcvrMode', values: '-1 unknown, 1 rover, 2 base', set_by: 'PQTMCFGRCVRMODE,OK parser (single site)', freshness_bound_ms: 120000, freshness_var: 'g_probedRcvrModeMs', intent_stamped_at: 'checkPppSurveyCompletion (=2 without read-back)', consumers: 'updateCastQuality (fresh only), checkPositionDisplacement, hot-start probe, verifyEscapedFixedBase', backstop: 'gate_msm_active — a rover emits no MSM', source: 'code'})
CREATE (pc_sm:ProbeCache {id: 'pc_svin_mode', variable: 'g_probedSvinMode', values: '0 unknown, 1 survey-in, 2 fixed', set_by: 'PQTMCFGSVIN,OK parser', freshness_bound_ms: null, intent_stamped_at: 'checkPppSurveyCompletion (=2 without read-back)', consumers: 'updateBaseOutputWatchdog (==2), checkPositionDisplacement forced-rover decision (==2), resolvePosCheck* (==2 with hot start)', backstop: 'none specific', source: 'code', notes: 'No freshness bound. Only cleared to 0 by the verify/probe sequences that immediately re-read it.'})
CREATE (pc_ss:ProbeCache {id: 'pc_survey_status', variable: 'surveyStatus', set_by: 'PQTMSVINSTATUS (base-mode-only output, 1 Hz)', freshness_bound_ms: 10000, corroborated_by: 'receiverHasSignal() for valid==2', cleared_by: 'resetSurveyBookkeeping()', consumers: 'updateBaseReadiness via svinValidNow(), dashboard, gps CSV svin_* columns', source: 'code'})
CREATE (pc_masks:ProbeCache {id: 'pc_sat_masks', variable: 'g_probedEleMask / g_probedCnrMask', set_by: 'PQTMCFGELETHD,OK / PQTMCFGCNRTHD,OK after the write', freshness_bound_ms: null, read_back_after_write: true, unset_value: -1, consumers: 'dashboard', source: 'code', notes: '-1 after a write is itself the answer: the mask write was not acknowledged.'})
CREATE (pc_bm:ProbeCache {id: 'pc_base_mode_confirmed', variable: 'g_baseModeConfirmed', set_by: 'read-back only: configureLg290pBaseOnce end, hot-start probe, serviceForceBase', cleared_by: 'any PQTMCFGRCVRMODE reply that is not 2 (the parser, single site)', consumers: 'baseUsingSavedPosition() and everything that reports a base on a saved position', source: 'code', notes: 'Introduced because intent and confirmation were the same variable and a rover masquerading as a base was invisible from every consumer.'})
CREATE (pc_ppp:ProbeCache {id: 'pc_ppp_supported', variable: 'g_pppSupported / g_pppCfgReply', set_by: 'any $PQTMCFGPPP reply (OK -> 1, else 0); -1 until then', source: 'code', notes: 'PQTMCFGPPP is written in two places and neither ever looked at the answer.'})
CREATE (pc_ant:ProbeCache {id: 'pc_antenna_desc', variable: 'g_antennaDesc / g_antennaDescLen', set_by: 'RTCM 1033 decode', measured_len: 0, source: 'measured', notes: 'Zero-length on this module, so rovers apply no ANTEX phase-centre correction (a few cm of vertical bias) and 1007 has nothing to carry. No verified command is known to set it.'})
CREATE (pc_saved:ProbeCache {id: 'pc_saved_position', variable: 'g_savedLat/Lon/Alt, g_savedHAcc, g_savedSource, g_savedSurveySec, g_savedNote', set_by: 'loadSavedPosition() from rcx1pos at boot', consumers: 'sm_pos_check, configureLg290pBaseOnce NVS branch, realisticAccuracyM, status CSV pos_src/pos_acc', source: 'code'})

MATCH (p:ProbeCache), (r {id: 'lg290p_receiver'}) WHERE p.id <> 'pc_saved_position' CREATE (p)-[:CACHES_STATE_OF {}]->(r)
MATCH (a {id: 'pc_saved_position'}), (b {id: 'nvs_rcx1pos'})     CREATE (a)-[:CACHES_STATE_OF {}]->(b)
MATCH (a {id: 'gate_rcvr_mode_fresh'}), (b {id: 'pc_rcvr_mode'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'gate_svin_valid_now'}),  (b {id: 'pc_survey_status'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'sm_pos_check'}),         (b {id: 'pc_svin_mode'})  CREATE (a)-[:READS {decision: 'force rover before collecting'}]->(b)
MATCH (a {id: 'sm_pos_check'}),         (b {id: 'pc_rcvr_mode'})  CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'sm_pos_check'}),         (b {id: 'pc_saved_position'}) CREATE (a)-[:READS {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 12. Command sequences — the paced PQTM sets, what they block, what they verify
// ─────────────────────────────────────────────────────────────────────────────

CREATE (cs_probe:CommandSequence {id: 'cs_probe', name: 'Hot-start probe', commands: ['PQTMCFGRCVRMODE,R', 'PQTMCFGSVIN,R'], waits_for: 'BOTH replies, not any UART traffic', blocking_bound_ms: 63500, verify: true, source: 'code', notes: 'Autonomous 1 Hz PQTMSVINSTATUS will trip a naive early exit before the CFGSVIN reply is parsed (field bug 2026-06-24).'})
CREATE (cs_esc_hot:CommandSequence {id: 'cs_rover_escape_hot', name: 'Rover escape (hot)', commands: ['PQTMCFGRCVRMODE,W,1', 'PQTMCFGSVIN,W,1,<sec>,<acc>,0,0,0', 'PQTMSAVEPAR', 'PQTMHOT'], blocking_ms: 5250, verify: 'verifyEscapedFixedBase (up to 3000 ms) at boot and PPP start; NOT at the pos-check site', sites: 'boot rover-first, servicePppSurveyStart phase 1, checkPositionDisplacement Idle', source: 'code'})
CREATE (cs_esc_srr:CommandSequence {id: 'cs_rover_escape_srr', name: 'Rover escape (full reset)', commands: ['PQTMCFGRCVRMODE,W,1', 'PQTMCFGSVIN,W,1,<sec>,<acc>,0,0,0', 'PQTMSAVEPAR', 'PQTMSRR'], blocking_ms: 5250, verify: true, escalation_of: 'cs_rover_escape_hot', costs: 'retained ephemeris (slower reacquisition)', source: 'code', notes: 'If this too leaves the module fixed, the operator-level recovery is the backup battery.'})
CREATE (cs_base:CommandSequence {id: 'cs_base_enter', name: 'Enter base mode (configureLg290pBaseOnce full path)', commands: ['PQTMCFGSVIN,W,2,0,0,<X>,<Y>,<Z> or W,1 targets', 'PQTMCFGRSID,W,290', 'PQTMCFGRCVRMODE,W,2', 'PQTMCFGRCVRMODE,R', 'enableLg290pBaseOutputs', 'PQTMSAVEPAR', 'PQTMSRR', 'PQTMCFGRCVRMODE,R', 'PQTMCFGSVIN,R'], blocking_ms: 8500, verify: 'read-back; one retry (adds ~5500 ms) (SAVEPAR + SRR) then logEvent fail', sets: 'g_baseModeConfirmed from the read-back', source: 'code', notes: 'PQTMSRR, not PQTMHOT: this was the one base-mode transition that used a hot start and never checked the result.'})
CREATE (cs_tele:CommandSequence {id: 'cs_telemetry_rates', name: 'enableLg290pTelemetryMessages', commands: ['PQTMCFGMSGRATE,W,PQTMSVINSTATUS,1,1', 'PQTMCFGMSGRATE,W,PQTMPVT,1,1', 'PQTMCFGMSGRATE,W,GGA,1', 'PQTMCFGMSGRATE,W,RMC,1', 'PQTMCFGMSGRATE,W,GSV,1', 'PQTMCFGMSGRATE,W,PQTMEPE,1,2'], blocking_ms: 1500, verify: false, non_destructive: true, source: 'code', notes: 'PQTM rates need a trailing MsgVer; NMEA rates take none; PQTMSVINSTATUS returns ERROR,3 unless base mode is already set. In rover mode PQTMEPE wants the bare form, which is sent as well.'})
CREATE (cs_out:CommandSequence {id: 'cs_base_outputs', name: 'enableLg290pBaseOutputs', commands: ['PQTMCFGRTCM,W,4,0,-90,07,06,1,0', 'PQTMCFGPROT,W,1,1,00000005,00000005', 'PQTMCFGMSGRATE,W,RTCM3-1005,10', 'PQTMCFGMSGRATE,W,RTCM3-1033,10', 'PQTMCFGMSGRATE,W,RTCM3-1230,10', 'PQTMCFGMSGRATE,W,RTCM3-1019,1', 'PQTMCFGMSGRATE,W,RTCM3-1020,1', 'PQTMCFGMSGRATE,W,RTCM3-1042,1', 'PQTMCFGMSGRATE,W,RTCM3-1046,1'], blocking_ms: 2250, verify: 'none per command; the output watchdog watches the stream instead', non_destructive: true, sites: 'full configure, hot-start skip, PPP completion, force base, output watchdog repair', source: 'code', notes: 'MSM4 is enabled by PQTMCFGRTCM, never per-message; wildcard MSGRATE forms return ERROR,1.'})
CREATE (cs_gate:CommandSequence {id: 'cs_sat_gating', name: 'applySatelliteGating', commands: ['PQTMCFGELETHD,W,5.0', 'PQTMCFGCNRTHD,W,10.0', 'PQTMCFGELETHD,R', 'PQTMCFGCNRTHD,R'], verify: 'read back into pc_sat_masks', sites: 'setup() before configure; serviceForceBase', source: 'code'})
CREATE (cs_pppcfg:CommandSequence {id: 'cs_ppp_configure', name: 'PPP survey configure (ppp_survey_tick CONFIGURING)', commands: ['PQTMCFGRCVRMODE,W,1', 'PQTMCFGPPP,W,2,1,120,0.10,0.15', 'PQTMCFGMSGRATE,W,PQTMEPE,1', 'PQTMCFGMSGRATE,W,GGA,1', 'PQTMCFGMSGRATE,W,GSV,1', 'PQTMCFGELETHD,W,5.0', 'PQTMCFGCNRTHD,W,10.0', 'PQTMCFGMSGRATE,W,PQTMPPPNAV,1,1'], step_interval_ms: 250, blocking: 'none — paced from loop()', verify: 'PQTMCFGPPP reply captured into pc_ppp_supported', source: 'code'})
CREATE (cs_lock:CommandSequence {id: 'cs_ppp_lock', name: 'PPP lock (ppp_survey_tick LOCKING)', commands: ['PQTMCFGPPP,W,0', 'PQTMCFGRCVRMODE,W,2', 'PQTMCFGSVIN,W,2,0,0,<X>,<Y>,<Z>', 'PQTMSAVEPAR', 'PQTMSRR'], step_interval_ms: 250, post_reset_settle_ms: 5000, followed_by: 'checkPppSurveyCompletion: savePositionToNvs, enableLg290pBaseOutputs, PQTMSAVEPAR, stamps pc_rcvr_mode = pc_svin_mode = 2', verify: 'none — the stamp is intent; gate_msm_active is the backstop', source: 'code', notes: 'Settle timing is measured against gSettleStartMs, not gStepMs, which the pacing gate reassigns every tick; measuring against gStepMs stalled LOCKING at step 5 forever.'})
CREATE (cs_force:CommandSequence {id: 'cs_force_base', name: 'serviceForceBase', commands: ['ppp_survey_abort', 'PQTMCFGRCVRMODE,W,2', 'PQTMCFGSVIN,W,2,0,0,<live ECEF>', 'PQTMSAVEPAR', 'PQTMSRR', 'PQTMCFGRCVRMODE,R', 'applySatelliteGating', 'enableLg290pBaseOutputs'], blocking_ms: 8750, verify: 'read-back; persists src forced with accuracy = live scatter sigma only if mode 2 confirmed', http: '409 when there is no fix', source: 'code'})
CREATE (cs_pppq:CommandSequence {id: 'cs_ppp_query', name: 'queryPppSupport', commands: ['PQTMCFGPPP,R'], not_sent: 'PQTMCFGSIGNAL,R — read syntax unverified; a wrong argument list could be treated as a write that persists a receiver tracking nothing', source: 'code'})

MATCH (c:CommandSequence), (f {id: 'flow_pqtm_cmd'}) CREATE (c)-[:SENT_OVER {}]->(f)
MATCH (c:CommandSequence), (t {id: 'task_loop'}) WHERE c.blocking_ms IS NOT NULL CREATE (c)-[:BLOCKS {ms: c.blocking_ms, drains_gnss: true, starves: 'casters, local rovers, web, display', source: 'derived'}]->(t)
MATCH (a {id: 'cs_rover_escape_srr'}), (b {id: 'cs_rover_escape_hot'}) CREATE (a)-[:ESCALATION_OF {}]->(b)
MATCH (a {id: 'cs_rover_escape_hot'}), (b {id: 'sm_lg290p_mode'}) CREATE (a)-[:TRANSITIONS {to: 'rover + survey-in targets'}]->(b)
MATCH (a {id: 'cs_base_enter'}),       (b {id: 'sm_lg290p_mode'}) CREATE (a)-[:TRANSITIONS {to: 'base + fixed or base + survey-in'}]->(b)
MATCH (a {id: 'cs_ppp_lock'}),         (b {id: 'sm_lg290p_mode'}) CREATE (a)-[:TRANSITIONS {to: 'base + fixed ECEF'}]->(b)
MATCH (a {id: 'cs_force_base'}),       (b {id: 'sm_lg290p_mode'}) CREATE (a)-[:TRANSITIONS {to: 'base + fixed ECEF'}]->(b)
MATCH (a {id: 'cs_ppp_configure'}),    (b {id: 'sm_lg290p_mode'}) CREATE (a)-[:TRANSITIONS {to: 'rover + PPP'}]->(b)
MATCH (a {id: 'cs_probe'}),            (b {id: 'pc_rcvr_mode'}) CREATE (a)-[:REFRESHES {}]->(b)
MATCH (a {id: 'cs_probe'}),            (b {id: 'pc_svin_mode'}) CREATE (a)-[:REFRESHES {}]->(b)
MATCH (a {id: 'cs_base_enter'}),       (b {id: 'pc_base_mode_confirmed'}) CREATE (a)-[:REFRESHES {}]->(b)
MATCH (a {id: 'cs_sat_gating'}),       (b {id: 'pc_sat_masks'}) CREATE (a)-[:REFRESHES {}]->(b)
MATCH (a {id: 'cs_ppp_lock'}),         (b {id: 'pc_rcvr_mode'}) CREATE (a)-[:STAMPS_INTENT_ON {notes: 'known; covered by observation-based gate conditions'}]->(b)
MATCH (a {id: 'cs_ppp_lock'}),         (b {id: 'pc_svin_mode'}) CREATE (a)-[:STAMPS_INTENT_ON {}]->(b)
MATCH (a {id: 'cs_ppp_configure'}),    (b {id: 'mod_ppp_survey'}) CREATE (a)-[:LIVES_IN {}]->(b)
MATCH (a {id: 'cs_ppp_lock'}),         (b {id: 'mod_ppp_survey'}) CREATE (a)-[:LIVES_IN {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 13. Budgets — the governed limits
// ─────────────────────────────────────────────────────────────────────────────

CREATE (bud_loop:Budget {id: 'budget_loop_pass', name: 'loop() pass without draining', resource: 'uart1', budget_ms: 89, measured_ms: null, measured_by: 'loop_max_ms column, peak per 5 s status row', status: 'GUARDED', source: 'derived', notes: 'Every command sequence pumps processGnssSerial() while it waits, so their multi-second blocks do not overrun the ring. Two calls do not pump: client.connect() (1200 ms, one per pass) and WiFi.hostByName() (unbounded).'})
CREATE (bud_uart:Budget {id: 'budget_uart1_rx', name: 'UART1 RX utilisation', resource: 'uart1', budget_pct: 60, measured_pct: null, estimate_pct: 4, status: 'OK', source: 'estimate', notes: 'RTCM ~760 B/s plus a few hundred B/s of NMEA against 46080 B/s.'})
CREATE (bud_heap:Budget {id: 'budget_internal_heap_floor', name: 'Free internal pool floor', resource: 'internal_sram', budget_bytes: 40000, measured_bytes: null, measured_by: 'heap_free_int / heap_largest_int / heap_min_int every status row; updateHeapWatchdog warns below 40 kB at most once per 60 s', status: 'THIN', source: 'measured', notes: 'One session ended in ESP_ERR_NO_MEM. Static consumers identified in this graph total roughly 139 kB before WiFi and lwIP; the measured ladder is a gap.'})
CREATE (bud_queue:Budget {id: 'budget_sd_queue', name: 'Row queue headroom', resource: 'sd_bus', budget_s: 3.2, measured: 'log_drop column', status: 'THIN', source: 'derived', notes: 'At 96 rows and ~30 rows/s a directory scan or an SD latency spike exceeds this; overflow shows as missing sat rows and gaps in gps while status looks fine.'})
CREATE (bud_dark:Budget {id: 'budget_caster_dark_time', name: 'Congestion tolerated before teardown', budget_ms: 20000, measured_max_run_ms: 15000, rover_rtk_hold_s: 30, status: 'OK', source: 'measured', notes: 'Longest legitimate congestion run seen was ~15 s (0453). Do not raise into the minutes.'})
CREATE (bud_recon:Budget {id: 'budget_reconnect_floor', name: 'Caster reconnect floor', budget_ms: 30000, cap_ms: 600000, ban_floor_ms: 10000, status: 'OK', source: 'code', notes: 'rtk2go abuse thresholds are low for push-in sources; rejected pushes earn IP bans of hours to weeks that then look like TCP timeouts to every device on the network.'})
CREATE (bud_hold:Budget {id: 'budget_tx_hold', name: 'Coalescing hold', budget_ms: 400, status: 'OK', source: 'code', notes: 'Well under the 8 s uplink-stall watchdog so a trickle cannot look like a stall.'})
CREATE (bud_forgive:Budget {id: 'budget_stall_forgive', name: 'Stall forgiveness threshold', budget_ms: 2000, status: 'OK', source: 'code'})
CREATE (bud_survey:Budget {id: 'budget_ppp_survey', name: 'PPP survey acceptance', window_s_default: 900, min_converged_fixes: 60, scatter_window_fixes: 60, scatter_limit_m: 0.25, epe_2d_limit_m: 0.30, epe_3d_limit_m: 0.50, epe_floor_measured_m: 1.300, scatter_60s_measured_m: '0.055-0.18 settled; 0.28-0.49 whole-session', status: 'EPE ROUTE UNREACHABLE; SCATTER ROUTE PRIMARY', source: 'measured', notes: 'PQTMEPE never reported below 1.300 m across 8850 fixes in three sessions and sat exactly on 1.300 for 10-46 % of them — a reporting floor. Sub-decimetre minute-scale scatter is PPP-grade, so convergence is now judged from position scatter computed here.'})
CREATE (bud_pos:Budget {id: 'budget_pos_check', name: 'Boot venue check', window_s: '30 / 120 / 300 by survey length', min_fixes: 10, idle_timeout_s: 120, rover_settle_s: 15, move_thresh_m: 5.0, gross_move_m: 5000, epe_fallback_m: 2.5, status: 'OPEN', source: 'code', notes: 'Measures BEFORE convergence: 0022 sampled 8 s after the rover switch with 7-8 sats, pc_epe 5.135 > thresh 5.000, and the converged position sat 7.81 m away.'})
CREATE (bud_quality:Budget {id: 'budget_cast_quality', name: 'Quality gate limits', crc_fail_rate_max: 0.02, crc_window_ms: 30000, crc_min_samples: 50, pos_acc_max_m: 5.0, hold_ms: 20000, recover_ms: 5000, msm_fresh_ms: 25000, signal_fresh_ms: 15000, status: 'OK', source: 'code', notes: 'Sessions 0457/0458 ran 34 h at zero CRC failures, so anything sustained above a fraction of a percent is a fault.'})
CREATE (bud_thermal:Budget {id: 'budget_thermal', name: 'Thermal bands', die_warn_hot_crit_c: [70, 85, 95], board_warn_hot_crit_c: [65, 75, 85], measured_with_duct_f: 152.8, measured_before_duct_f: '177-184', status: 'FIXED', source: 'measured', notes: 'Board sensor is the QMI8658 under the SD card in a dead-air pocket: the SD card environment, not ambient.'})
CREATE (bud_sdcap:Budget {id: 'budget_sd_capacity', name: 'Card capacity', rotate_bytes: 134217728, rotate_suffixes: 26, sat_csv_days_to_4gib: 29, card_full_days: 42, status: 'OPEN', source: 'derived', notes: 'Rotation now applies to every tag at 128 MB (_a.._z), which keeps single files under FAT32 limits, but nothing guards total capacity and writeLine() ignores the println() return, so a full card is silent.'})

MATCH (b:Budget), (r:Resource) WHERE b.resource = r.id CREATE (b)-[:GOVERNS {}]->(r)
MATCH (a {id: 'budget_ppp_survey'}),   (b {id: 'sm_ppp_survey'})  CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_pos_check'}),    (b {id: 'sm_pos_check'})   CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_cast_quality'}), (b {id: 'gate_cast_quality'}) CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_caster_dark_time'}), (b {id: 'sm_caster'})  CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_reconnect_floor'}),  (b {id: 'sm_caster'})  CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_tx_hold'}),      (b {id: 'flow_ntrip_source'}) CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_stall_forgive'}), (b {id: 'sm_caster'})     CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_sd_queue'}),     (b {id: 'q_sdlog_rows'})   CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_sd_capacity'}),  (b {id: 'sd_bus'})         CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'budget_loop_pass'}),    (b {id: 'buf_uart1_rx'})   CREATE (a)-[:SIZED_BY {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 14. Instrumentation — and its gaps
// ─────────────────────────────────────────────────────────────────────────────

CREATE (i_status:Instrument {id: 'inst_status_csv', file: 'status_NNNN.csv', cadence_s: 5, columns: 73, reports: 'wifi/ssid/ip, fix/sats, base_ready, rtcm fps/bps, valid frames, crc/framing/nmea-checksum/desync counters, loop_max_ms, uart_high_water, heap free/largest/min, local clients/served, two caster slots (host, mount, state, enabled, err, handshake, frames, bytes, dropped, age), svin state/mode, pos_src/pos_acc, hot start, pc_* boot-check fields, temps, log_drop', source: 'code', notes: 'Starts at SD-ready, not caster-connected, so boot and survey are always captured. Grown from 58 columns; LogLine was widened to 560 to stop the tail being cut off.'})
CREATE (i_gps:Instrument {id: 'inst_gps_csv', file: 'gps_NNNN.csv', cadence_hz: 1, reports: 'fix, sats, hdop, lat/lon/alt at 9 decimals, epe 3d/2d, ecef, svin valid/obs/target/meanacc, base_ready', source: 'code'})
CREATE (i_sat:Instrument {id: 'inst_sat_csv', file: 'sat_NNNN.csv', cadence: 'one row per satellite per second', reports: 'constellation, prn, elev, azim, snr', source: 'code', notes: 'Multi-band GSV tests must bucket by SECOND, never raw ms — a GSV cycle spans several sentences with different timestamps.'})
CREATE (i_event:Instrument {id: 'inst_event_csv', file: 'event_NNNN.csv', cadence: 'on event', sites: 36, reports: 'level ok/warn/fail/info + message; every logEvent() also prints to serial', source: 'code', notes: 'The record of things that used to exist only on a serial port nobody is watching in a paddock.'})
CREATE (i_rtcm:Instrument {id: 'inst_rtcm_csv', file: 'rtcm_NNNN.csv', cadence: 'per frame, opt-in', reports: 'type, len, crc_ok, per-caster disposition (sent / not-auth / disconnected / failed / oversize), decoded detail (station id, ECEF, sat/sig counts, 1033 descriptor length)', source: 'code'})
CREATE (i_raw:Instrument {id: 'inst_raw_bin', file: 'rtcm_NNNN.bin', cadence: 'opt-in, flushed at 1 s', reports: 'exact RTCM bytes pushed', source: 'code'})
CREATE (i_boot:Instrument {id: 'inst_boot_line', marker: 'BOOT: reset_reason=', cadence: 'boot', reports: 'POWERON / SW_RESET / PANIC / INT_WDT / TASK_WDT / BROWNOUT, free heap, largest block, PSRAM, temps', source: 'code', notes: 'Converts a frequent-restart complaint into a labelled event.'})
CREATE (i_heapwd:Instrument {id: 'inst_heap_watchdog', marker: 'Internal DMA heap low', cadence: 'when < 40 kB, at most 1 per 60 s', source: 'code'})
CREATE (i_api:Instrument {id: 'inst_api_status', endpoint: '/api/status', cadence_ms: 1500, bytes: 5632, reports: 'everything the LCD shows plus: probe modes, quality-gate reason, pos-check reason, missing expected RTCM types, antenna descriptor, PPP support reply, verbatim caster banner, accknown flag', source: 'code'})
CREATE (i_ppp:Instrument {id: 'inst_api_ppp', endpoint: '/api/ppp', cadence_ms: 1500, reports: 'state, elapsed/dur, samples/need, valid fixes, EPE 2D/3D last and best, limits, scatter sigma and limit, E6 sats and C/N0, mean position, locked ECEF, gate note naming which test is rejecting', source: 'code'})
CREATE (i_lcd:Instrument {id: 'inst_lcd', cadence_hz: 1, reports: 'stoplight status, caster short states, survey countdown, temps, bold 2 px glyphs for sunlight', source: 'code'})
CREATE (i_rings:Instrument {id: 'inst_capture_rings', endpoints: '/rtcm.raw, /caster0/raw, /caster1/raw', reports: 'last ~60 s of validated RTCM and per-caster TX bytes', source: 'code'})
CREATE (i_verify:Instrument {id: 'inst_escape_verify', marker: 'Escape verify (<context>)', cadence: 'per mode transition', reports: 'confirmed rover / STILL reports rcvrMode svinMode / no reply in 3000 ms treated as still fixed', source: 'code'})
CREATE (i_1033:Instrument {id: 'inst_1033_decode', marker: 'RTCM 1033 decode', reports: 'antenna descriptor length (empty = no ANTEX correction downstream), receiver firmware and serial (authoritative source of the module version)', source: 'code'})

MATCH (a {id: 'budget_internal_heap_floor'}), (b {id: 'inst_status_csv'})    CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'budget_internal_heap_floor'}), (b {id: 'inst_heap_watchdog'}) CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'budget_loop_pass'}),           (b {id: 'inst_status_csv'})    CREATE (a)-[:REPORTED_BY {column: 'loop_max_ms, uart_high_water'}]->(b)
MATCH (a {id: 'budget_sd_queue'}),            (b {id: 'inst_status_csv'})    CREATE (a)-[:REPORTED_BY {column: 'log_drop'}]->(b)
MATCH (a {id: 'budget_cast_quality'}),        (b {id: 'inst_status_csv'})    CREATE (a)-[:REPORTED_BY {column: 'crc/framing/nmea counters, c*_err carries the gate reason when Held'}]->(b)
MATCH (a {id: 'budget_ppp_survey'}),          (b {id: 'inst_api_ppp'})       CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'budget_pos_check'}),           (b {id: 'inst_status_csv'})    CREATE (a)-[:REPORTED_BY {column: 'pc_state, pc_fixes, pc_lat/lon, pc_epe, pc_dist, pc_thresh'}]->(b)
MATCH (a {id: 'budget_thermal'}),             (b {id: 'inst_status_csv'})    CREATE (a)-[:REPORTED_BY {column: 'esp_temp_f, imu_temp_f'}]->(b)
MATCH (a {id: 'gate_cast_quality'}),          (b {id: 'inst_api_status'})    CREATE (a)-[:REPORTED_BY {field: 'quality reason'}]->(b)
MATCH (a {id: 'pc_antenna_desc'}),            (b {id: 'inst_1033_decode'})   CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'cs_rover_escape_hot'}),        (b {id: 'inst_escape_verify'}) CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (i:Instrument), (m {id: 'mod_sdlog'}) WHERE i.file IS NOT NULL CREATE (i)-[:WRITTEN_BY {}]->(m)

CREATE (gap_ladder:InstrumentationGap {id: 'gap_boot_heap_ladder', subject: 'internal_sram', missing: 'per-stage free heap through setup()', impact: 'The base has one heap figure at boot and one per status row afterwards; where the internal pool goes between them is unknown, so the 40 kB floor cannot be budgeted against real costs. The rover measured its ladder; the base never has.', priority: 'high', source: 'unknown'})
CREATE (gap_stacks:InstrumentationGap {id: 'gap_task_stacks', subject: 'task_loop, task_sdlog', missing: 'stack high-water marks', impact: 'The deepest loop() path is a web handler with ~3-6 kB of locals on an 8 kB default stack; neither task can be trimmed or proven safe.', priority: 'medium', source: 'unknown'})
CREATE (gap_qdepth:InstrumentationGap {id: 'gap_queue_depth', subject: 'q_sdlog_rows', missing: 'depth high-water mark', impact: 'Only drops are counted; whether 96 is generous or barely enough is invisible until rows go missing.', priority: 'medium', source: 'unknown'})
CREATE (gap_attr:InstrumentationGap {id: 'gap_stall_attribution', subject: 'budget_loop_pass', missing: 'which call produced loop_max_ms', impact: 'A 1.2 s connect, an unbounded DNS lookup, a 4.5 s receiver reset and a web handler all look identical in the column.', priority: 'medium', source: 'unknown'})
CREATE (gap_ack:InstrumentationGap {id: 'gap_pqtm_ack', subject: 'flow_pqtm_cmd', missing: 'acknowledgement tracking for writes', impact: 'PQTMSAVEPAR is sent from 11 sites and never checked; every MSGRATE, RTCM, PROT and PPP write is likewise. A write the module declines is indistinguishable from one that applied. The output watchdog and the read-backs are the only substitutes.', priority: 'high', source: 'code'})
CREATE (gap_pppnav:InstrumentationGap {id: 'gap_pppnav', subject: 'flow_nmea_pqtm', missing: 'a PQTMPPPNAV sentence ever observed', impact: 'Requested at every PPP configure and captured verbatim, but not yet seen in a survey; whether the module reports a PPP solution at all is still unknown.', priority: 'medium', source: 'unknown'})
CREATE (gap_1084:InstrumentationGap {id: 'gap_1084_undercount', subject: 'flow_rtcm_from_module', missing: 'explanation for GLONASS 1084 at ~3945 frames against ~20000 for the other MSM types (0043)', priority: 'low', source: 'measured'})
CREATE (gap_dns:InstrumentationGap {id: 'gap_dns_blocking', subject: 'cs_probe', missing: 'WiFi.hostByName() duration', impact: 'The one unbounded, non-draining block on loop(). Re-resolved only after 3 consecutive failures, so it is rare — and unmeasured.', priority: 'medium', source: 'unknown'})
CREATE (gap_sdfull:InstrumentationGap {id: 'gap_sd_full', subject: 'sd_bus', missing: 'write-return and free-space checks', impact: 'A full card writes nothing and reports nothing; the status CSV that would show it is the thing that stopped.', priority: 'high', source: 'code'})
CREATE (gap_early:InstrumentationGap {id: 'gap_early_lock_0044', subject: 'sm_ppp_survey', missing: 'why 0044 locked at svin_obs_s 1625 of a 2700 s window', impact: 'Either the window was retargeted live or the elapsed accounting is wrong; the event log does not say which.', priority: 'medium', source: 'measured'})

MATCH (g:InstrumentationGap), (n) WHERE g.subject = n.id CREATE (g)-[:GAP_IN {}]->(n)
MATCH (a {id: 'gap_task_stacks'}), (b {id: 'task_loop'})  CREATE (a)-[:GAP_IN {}]->(b)
MATCH (a {id: 'gap_task_stacks'}), (b {id: 'task_sdlog'}) CREATE (a)-[:GAP_IN {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 15. Standing rules — the invariants, each one paid for
// ─────────────────────────────────────────────────────────────────────────────

CREATE (rule_enq:Rule    {id: 'rule_enqueue_only',      text: 'loop() and the RTCM parser never write the card. They enqueue; the Core-0 task writes.'})
CREATE (rule_core0:Rule  {id: 'rule_core0_no_flash',    text: 'Nothing on Core 0 writes NVS or mutates casters[]. A flash commit disables the instruction cache on both cores.'})
CREATE (rule_cdc:Rule    {id: 'rule_no_periodic_serial', text: 'An unattended base must not narrate itself: no Serial print on any periodic path. USB CDC blocks ~100 ms with no host.'})
CREATE (rule_probe:Rule  {id: 'rule_probe_freshness',   text: 'A cached probe value is a memory of the past. Any gate keyed on one needs a freshness bound AND an observation-based backstop.'})
CREATE (rule_corrob:Rule {id: 'rule_corroborate',       text: 'A live, re-asserted receiver claim must be corroborated, not reset. A hot-started fixed module emits valid=2 at 1 Hz with no antenna.'})
CREATE (rule_rb:Rule     {id: 'rule_read_back',         text: 'Writing a mode is not entering it. Read the mode back; never stamp a probe cache with intent.'})
CREATE (rule_mode:Rule   {id: 'rule_mode_change',       text: 'A change OUT of rover needs PQTMCFGRCVRMODE,W + PQTMSAVEPAR + a restart. PQTMHOT first, PQTMSRR as escalation, and verify each.'})
CREATE (rule_order:Rule  {id: 'rule_base_outputs_after_mode', text: 'Base-mode-only outputs (PQTMSVINSTATUS, RTCM rates) are configured only after PQTMCFGRCVRMODE,W,2; PQTM rates carry a MsgVer, NMEA rates do not, and MSM4 comes from PQTMCFGRTCM.'})
CREATE (rule_rover1:Rule {id: 'rule_rover_first',       text: 'The base never boots into base mode. Rover first; base mode exactly once, after a PPP lock or a confirmed saved position.'})
CREATE (rule_msm:Rule    {id: 'rule_msm_not_1005',      text: 'Readiness and activity key on MSM observations, never on 1005/1006/1033, which the module emits during survey-in from an unconverged position.'})
CREATE (rule_bad:Rule    {id: 'rule_dont_cast_bad',     text: 'Being ready and being worth listening to are different questions. A base that publishes nothing is an inconvenience; one that publishes a wrong reference drags every rover off position.'})
CREATE (rule_ban:Rule    {id: 'rule_ban_safe',          text: 'Reconnect floor 30 s with exponential backoff to 10 min; no caster is second-class and none is exempt.'})
CREATE (rule_pass:Rule   {id: 'rule_one_connect_per_pass', text: 'At most one blocking connect attempt per loop pass.'})
CREATE (rule_clamp:Rule  {id: 'rule_forgive_clamp',     text: 'Stall forgiveness never advances a timestamp beyond now; unsigned age arithmetic would otherwise wrap and fire every watchdog at once.'})
CREATE (rule_stage:Rule  {id: 'rule_stage_not_discard', text: 'Frames that arrive before the handshake completes are staged, never discarded, and never sent early.'})
CREATE (rule_drop:Rule   {id: 'rule_drop_not_block',    text: 'A correction that cannot be sent right now is stale. Drop the oldest, never block the drain.'})
CREATE (rule_window:Rule {id: 'rule_rate_window_5s',    text: 'Rate counters use a 5 s window. RTCM arrives as one burst per epoch; a 1 s window manufactures phantom dropouts.'})
CREATE (rule_torn:Rule   {id: 'rule_drop_torn_nmea',    text: 'A sentence that fails its checksum is garbage and is dropped, never parsed. One torn GGA must never reach the position path.'})
CREATE (rule_reset:Rule  {id: 'rule_reset_bookkeeping', text: 'Every path that abandons a coordinate and starts a survey calls resetSurveyBookkeeping(): baseReady is a latch and surveyStatus is a cache, and neither clears itself.'})
CREATE (rule_masks:Rule  {id: 'rule_base_masks_open',   text: 'A base gates the opposite way from a rover: 5 deg / 10 dB-Hz, written and read back before any fix is collected, because the module keeps whatever the last firmware wrote.'})
CREATE (rule_units:Rule  {id: 'rule_durations_hhmmss',  text: 'Durations are entered and displayed as hh:mm:ss everywhere; a units-ambiguous field costs a whole survey window.'})
CREATE (rule_scatter:Rule {id: 'rule_scatter_primary',  text: 'PPP convergence is judged from measured position scatter computed here (floorless); the receiver EPE is a second route, never the only one.'})
CREATE (rule_prec:Rule   {id: 'rule_9_decimals',        text: 'Coordinates render at 9 decimals everywhere via fmtDeg()/fmtLatLon(); 0,0 renders as a dash.'})
CREATE (rule_cmdseq:Rule {id: 'rule_ppp_sequence_frozen', text: 'The ppp_survey command sequence is the field-proven 7-25-26 form. Do not correct it against the spec again.'})
CREATE (rule_unverified:Rule {id: 'rule_no_unverified_cmd', text: 'No command whose syntax has not been seen answered by this module is sent — the PQTMCFGSIGNAL episode made an unverified read a suspect for total signal loss.'})
CREATE (rule_spec:Rule   {id: 'rule_spec_silence',      text: 'A datasheet omission is not a physical absence. The Q39 was measured at E6 after the spec listed only E1/E5.'})

MATCH (a {id: 'task_loop'}),            (b {id: 'rule_enqueue_only'})   CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'task_sdlog'}),           (b {id: 'rule_core0_no_flash'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'flow_serial_console'}),  (b {id: 'rule_no_periodic_serial'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (p:ProbeCache), (b {id: 'rule_probe_freshness'}) WHERE p.id <> 'pc_saved_position' CREATE (p)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'pc_survey_status'}),     (b {id: 'rule_corroborate'})    CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (c:CommandSequence), (b {id: 'rule_read_back'}) WHERE c.id IN ['cs_rover_escape_hot', 'cs_rover_escape_srr', 'cs_base_enter', 'cs_force_base'] CREATE (c)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'sm_lg290p_mode'}),       (b {id: 'rule_mode_change'})    CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'cs_base_outputs'}),      (b {id: 'rule_base_outputs_after_mode'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'cs_telemetry_rates'}),   (b {id: 'rule_base_outputs_after_mode'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'stage_gnsscfg'}),        (b {id: 'rule_rover_first'})    CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'gate_msm_active'}),      (b {id: 'rule_msm_not_1005'})   CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'gate_cast_quality'}),    (b {id: 'rule_dont_cast_bad'})  CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'sm_caster'}),            (b {id: 'rule_ban_safe'})       CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'sm_caster'}),            (b {id: 'rule_one_connect_per_pass'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'sm_caster'}),            (b {id: 'rule_forgive_clamp'})  CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'gate_handshake_accepted'}), (b {id: 'rule_stage_not_discard'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'gate_caster_writable'}), (b {id: 'rule_drop_not_block'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'flow_rtcm_from_module'}), (b {id: 'rule_rate_window_5s'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'flow_nmea_pqtm'}),       (b {id: 'rule_drop_torn_nmea'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'gate_base_ready'}),      (b {id: 'rule_reset_bookkeeping'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'cs_sat_gating'}),        (b {id: 'rule_base_masks_open'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'mod_ppp_web'}),          (b {id: 'rule_durations_hhmmss'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'sm_ppp_survey'}),        (b {id: 'rule_scatter_primary'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'mod_ppp_survey'}),       (b {id: 'rule_ppp_sequence_frozen'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'cs_ppp_query'}),         (b {id: 'rule_no_unverified_cmd'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'cs_ppp_lock'}),          (b {id: 'rule_read_back'})
CREATE (a)-[:VIOLATES {detail: 'checkPppSurveyCompletion stamps g_probedRcvrMode = g_probedSvinMode = 2 from the lock sequence without a read-back.', status: 'known; verdicts covered by observation-based conditions, wording is not', source: 'code'}]->(b)
MATCH (a {id: 'sm_pos_check'}),         (b {id: 'rule_read_back'})
CREATE (a)-[:VIOLATES {detail: 'The forced-rover escape in the Idle branch sends rover + SAVEPAR + PQTMHOT and proceeds without verifyEscapedFixedBase; the identical-fix echo detector is the only backstop and costs the whole window.', status: 'open', source: 'code'}]->(b)
MATCH (a {id: 'pc_svin_mode'}),         (b {id: 'rule_probe_freshness'})
CREATE (a)-[:VIOLATES {detail: 'No freshness bound; consumed by the output watchdog and the venue check as a present truth.', status: 'open', source: 'code'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 16. Failure modes — what has actually gone wrong, and why
// ─────────────────────────────────────────────────────────────────────────────

CREATE (fm_bw:FailureMode {id: 'fm_blocking_caster_write', name: 'Blocking client.write() stalls the drain', session: '0441-0458', symptom: 'Loop stalls up to 9.9 s; RTCM CRC/framing failures 219/hr; frame drops 1.39 %.', root_cause: 'NetworkClient::write() retries 10x on a 1 s select(); availableForWrite() read 0 for both empty and full so the drop gate never fired.', status: 'fixed', fix: 'fd-level select() write gate (30 ms), 1024-byte sub-MSS coalescing, debounced WiFi, stall-forgiving watchdogs. Drops 0, failures 0 for 34 h.', source: 'measured'})
CREATE (fm_cdc:FailureMode {id: 'fm_usb_cdc_blocking', name: 'USB CDC print shreds the UART', session: '0442-0457', symptom: '219 RTCM CRC + 193 framing failures per hour; torn NMEA parsed into a year of 2032.', root_cause: 'A per-response Serial print blocking ~100 ms with no host attached.', status: 'fixed', fix: 'All periodic echo behind compile-time flags, default off.', source: 'measured'})
CREATE (fm_flash:FailureMode {id: 'fm_flash_cache_stall', name: 'WiFi driver flash commits stall both cores', symptom: 'Data corruption regression correlating with WiFi rotation.', root_cause: 'WiFi.begin() committed STA config to driver NVS on every 10 s rotation attempt; each commit disables the instruction cache on both cores.', status: 'fixed', fix: 'WiFi.persistent(false); setAutoReconnect(false); no per-attempt WiFi.disconnect().', source: 'measured'})
CREATE (fm_glitch:FailureMode {id: 'fm_wifi_glitch_teardown', name: 'One WiFi blip becomes a minute-plus outage', symptom: 'Both casters into 30 s backoff and SSID rotation advanced by a single transient status read.', root_cause: 'serviceWifi() and serviceCaster() acted on a raw WiFi.status().', status: 'fixed', fix: '3 s down-debounce, instant recovery.', source: 'measured'})
CREATE (fm_storm:FailureMode {id: 'fm_watchdog_storm', name: 'All watchdogs fire on stall resume', session: '0450', symptom: 'c0_age_s = 4294963 on an Error row; a caster mid-handshake torn down while a streaming one survived.', root_cause: 'Absolute millis deltas inflated by the stall, then a forgiveness slide that pushed a timestamp past now and wrapped.', status: 'fixed', fix: 'Forgiveness over 2 s, clamped to now.', source: 'measured'})
CREATE (fm_uplink:FailureMode {id: 'fm_uplink_watchdog_false_kill', name: 'Uplink watchdog kills a live connection', session: '0453, 0454', symptom: 'Healthy Centipede socket torn down at age 11-12 s.', root_cause: '0453: congestion mistaken for a half-open socket. 0454: the receiver trickled to 0-1 fps for 45 s, lastValidMs stayed fresh, and the watchdog blamed the socket.', status: 'fixed', fix: 'Not congested AND actually behind (lastValidMs - lastWriteMs > 2 s) AND fresh RTCM; congestion tolerated up to 20 s.', source: 'measured'})
CREATE (fm_1s:FailureMode {id: 'fm_rate_window_phantom', name: 'Phantom RTCM dropouts', session: '0454', symptom: 'rtcm_fps read 0..31 on a true 5.09 +/- 0.29 fps stream; twelve rows under 4 fps while the receiver produced over 4.5.', root_cause: 'A 1 s rate window against a stream that arrives as one ~1 s burst per epoch.', status: 'fixed', fix: '5 s window for every rate counter.', source: 'measured'})
CREATE (fm_probe:FailureMode {id: 'fm_probe_early_exit', name: 'Hot-start probe exits on the wrong traffic', session: '2026-06-24', symptom: 'False unconfigured verdict, needless full reset, multi-minute cold survey every boot.', root_cause: 'The probe waited for any UART traffic; autonomous 1 Hz PQTMSVINSTATUS satisfied it before the CFGSVIN reply arrived.', status: 'fixed', fix: 'Wait for BOTH query replies, up to 60 s plus a 3 s retry.', source: 'measured'})
CREATE (fm_cast_early:FailureMode {id: 'fm_cast_before_convergence', name: 'Casting before survey-in finished', symptom: 'Casters went ready and streamed descriptor-only data from an unconverged position.', root_cause: 'Readiness keyed on 1005 presence; the module emits 1005/1033 throughout survey-in.', status: 'fixed', fix: 'Readiness on PQTMSVINSTATUS valid==2 plus accuracy; activity on MSM only.', source: 'measured'})
CREATE (fm_stale:FailureMode {id: 'fm_stale_svinstatus', name: 'False survey complete for a whole session', session: '0022', symptom: 'base_ready=1, svin_valid=2 in every row of an 8.5-minute log while a PPP survey was 17 % through; casters Authenticated - waiting RTCM advertising an empty mount.', root_cause: 'resolvePosCheckMoved() and both Timeout branches cleared the saved position but not baseReady (set-only latch) or surveyStatus (cache of a base-mode-only output that stops in rover mode).', status: 'fixed', fix: 'SVIN_STATUS_STALE_MS + svinValidNow() + one resetSurveyBookkeeping() called by every survey-starting path; updateCastQuality() as an independent gate.', source: 'measured'})
CREATE (fm_bench:FailureMode {id: 'fm_bench_false_complete', name: 'Survey converged with no antenna', symptom: 'Dashboard read Survey (converged), valid 2, 1.446 m, at position 0,0 with zero satellites.', root_cause: 'A hot-started fixed module re-asserts valid=2 at 1 Hz from its retained configuration; clearing the cache is futile.', status: 'fixed', fix: 'receiverHasSignal() corroboration inside svinValidNow().', source: 'measured'})
CREATE (fm_rcvrmode:FailureMode {id: 'fm_stale_rcvr_mode', name: 'Stale rover reading blocks casting forever', symptom: 'A confirmed-position boot left g_probedRcvrMode at 1; the quality gate held every caster down for the session.', root_cause: 'The base-mode write produces no reply; the cache was set to 2 in one place only.', status: 'fixed', fix: 'PQTMCFGRCVRMODE,R after every base write; g_probedRcvrModeMs with a 120 s trust window; ppp_survey_active() checked first so the reason cannot go stale.', source: 'measured'})
CREATE (fm_desc:FailureMode {id: 'fm_descriptors_only', name: 'Base publishes descriptors and no observations', session: '0029', symptom: '18.5 minutes of 1005 and 1033 at 500 ms (2254 each) and zero MSM of any constellation; looked healthy on every indicator.', root_cause: 'The hot-start skip path re-asserted telemetry rates but not the RTCM output set, and the retained configuration had MSM off.', status: 'fixed', fix: 'enableLg290pBaseOutputs() on the hot-start path; updateBaseOutputWatchdog() repairs from observed output at most once per 120 s.', source: 'measured'})
CREATE (fm_force:FailureMode {id: 'fm_force_noop', name: 'Force base did nothing and said ok', symptom: 'Button returned ok; module state unchanged.', root_cause: 'Handler set nothing the loop acted on.', status: 'fixed', fix: 'g_forceBasePending + serviceForceBase() with abort, mode write, SAVEPAR, SRR, read-back, gating, outputs, persisted as src forced; 409 without a fix.', source: 'measured'})
CREATE (fm_masks:FailureMode {id: 'fm_rover_masks_inherited', name: 'Base starved by rover masks', symptom: '~10 satellites, none below 30 deg, none under 32 dB-Hz.', root_cause: 'Elevation and C/N0 masks live in the module NVM; the base never wrote them and inherited the rover pair.', status: 'fixed', fix: 'applySatelliteGating() writes 5 deg / 10 dB-Hz and reads both back before any fix is collected.', source: 'measured'})
CREATE (fm_hot:FailureMode {id: 'fm_base_mode_hot_start', name: 'Base mode never entered after a confirmed position', symptom: 'Every downstream flag said base on saved position; module stayed a rover and published nothing; no retry, no error.', root_cause: 'configureLg290pBaseOnce ended in PQTMHOT and discarded its own read-back.', status: 'fixed', fix: 'PQTMSRR, read-back, one retry, g_baseModeConfirmed set only from the read-back.', source: 'measured'})
CREATE (fm_battery:FailureMode {id: 'fm_fixed_battery_backed', name: 'Module refuses to leave fixed mode', symptom: 'Rover escape sent, every command apparently accepted, module still fixed; survey averages the echoed coordinate and re-locks it.', root_cause: 'The retained configuration is battery-backed; PQTMHOT and sometimes PQTMSRR do not dislodge it.', status: 'mitigated', fix: 'verifyEscapedFixedBase() after every escape, PQTMSRR escalation, identical-fix echo detectors in both the venue check and the survey. Operator recovery: disconnect the backup battery and all power.', source: 'measured'})
CREATE (fm_deadend:FailureMode {id: 'fm_ppp_failed_dead_end', name: 'A survey with no fix is a dead base', symptom: 'PPP_FAILED after a full window with zero fixes; no position, no base mode, no retry, no message; quality gate reports no MSM observations.', root_cause: 'ppp_survey had two failure exits and only echoFault was handled.', status: 'fixed', fix: 'servicePppSurveyRecovery(): retry after 60 s backoff once satellites are tracked, each attempt announced.', source: 'code'})
CREATE (fm_settle:FailureMode {id: 'fm_locking_settle_stall', name: 'PPP LOCKING never reaches DONE', symptom: 'Survey stuck at lock step 5 forever.', root_cause: 'Settle measured against gStepMs, which the pacing gate reassigns every tick.', status: 'fixed', fix: 'gSettleStartMs.', source: 'measured'})
CREATE (fm_epe:FailureMode {id: 'fm_epe_clamped', name: 'PQTMEPE is clamped at 1.300 m', session: '0025, 0029, 0044', symptom: 'Every survey fell back to the unfiltered mean (src ppp-auto); the 0.30 m gate was never reachable.', root_cause: 'Across 8850 fixes not one EPE below 1.300 and 10-46 % exactly 1.300 — a reporting floor. Meanwhile measured 60 s scatter ran 0.055-0.18 m: PPP was converging all along.', status: 'fixed, awaiting field validation', fix: 'Scatter-sigma convergence as the primary route (60 fixes, 0.25 m), EPE kept as a second route.', source: 'measured'})
CREATE (fm_ant:FailureMode {id: 'fm_antenna_misdiagnosis', name: 'E6 declared absent from a datasheet', symptom: 'PPP judged impossible on the Q39 helical.', root_cause: 'Spec sheet listed E1/E5 only; the antenna had been bench-measured with comparable gain at E6.', status: 'corrected', fix: 'rule_spec_silence.', source: 'measured'})
CREATE (fm_crtk:FailureMode {id: 'fm_crtk_idle_drop', name: 'crtk accepts then closes within the second', session: '0043', symptom: '3 frames not-auth at ms 23337952-955, next burst 1008 ms later already disconnected; offered=0 accepted=0.', root_cause: 'RTCM leaves in one burst per epoch and pre-handshake frames were discarded, so a fresh connection was guaranteed silent for up to a second; crtk drops idle sources.', status: 'fixed, awaiting field validation', fix: 'Stage frames during the handshake; flush on acceptance; distinguish accepted-then-closed from a plain drop.', source: 'measured'})
CREATE (fm_header:FailureMode {id: 'fm_local_header_160', name: 'Every authenticating rover rejected by the local caster', symptom: 'Rover with an Authorization header dropped with no reply.', root_cause: 'A 160-byte request buffer; a real NTRIP request is ~168 bytes.', status: 'fixed', fix: '512-byte header buffer.', source: 'code'})
CREATE (fm_nomem:FailureMode {id: 'fm_heap_no_mem', name: 'ESP_ERR_NO_MEM after 11 hours', symptom: 'sdmmc_cmd reported no memory; scattered symptoms that do not name memory.', root_cause: 'Row queue at depth 192 x 352 bytes held ~68 kB of internal heap, more than half of what was free before WiFi started.', status: 'mitigated', fix: 'Depth 96 — but the line grew to 560 bytes, so the queue still holds ~54 kB. Heap columns and watchdog added so the trend is visible.', source: 'measured'})
CREATE (fm_1007:FailureMode {id: 'fm_1007_never_emitted', name: 'Permanent missing 1007', symptom: 'Completeness check always complaining.', root_cause: '1007 carries only the antenna descriptor; this module has a zero-length descriptor so 1007 has no payload and never appears (0 of 19050 and 0 of 70450 frames).', status: 'fixed', fix: 'Removed from EXPECTED_RTCM_TYPES; 1033 decode reports descriptor length; 1230 added.', source: 'measured'})
CREATE (fm_tar:FailureMode {id: 'fm_posCheck_premature', name: 'Venue check measures before convergence', session: '0022', symptom: 'pc_epe 5.135 > thresh 5.000 with 7-8 sats, sampled 8 s after the rover switch; converged position 7.81 m away.', root_cause: 'Collection starts on the first fix rather than on a converged one.', status: 'open', fix: 'Offered, not built: gate fix collection on epe_2d < thresh/2 plus a satellite floor.', source: 'measured'})
CREATE (fm_early:FailureMode {id: 'fm_early_lock_0044', name: 'Survey locked early', session: '0044', symptom: 'svin_obs_s 1625 of a 2700 s window.', root_cause: 'unknown', status: 'open', source: 'measured'})
CREATE (fm_wifi_hw:FailureMode {id: 'fm_wifi_board_hardware', name: 'One board fails every WPA2 handshake', symptom: 'Long WiFi investigation; spare board worked on every network.', root_cause: 'Hardware.', status: 'resolved by swapping boards', source: 'measured'})

MATCH (a {id: 'fm_blocking_caster_write'}),   (b {id: 'gate_caster_writable'})  CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_usb_cdc_blocking'}),         (b {id: 'uart0'})                 CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_flash_cache_stall'}),        (b {id: 'nvs_wifi_driver'})       CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_wifi_glitch_teardown'}),     (b {id: 'gate_wifi_link'})        CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_watchdog_storm'}),           (b {id: 'sm_caster'})             CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_uplink_watchdog_false_kill'}), (b {id: 'sm_caster'})           CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_rate_window_phantom'}),      (b {id: 'flow_rtcm_from_module'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_probe_early_exit'}),         (b {id: 'cs_probe'})              CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_cast_before_convergence'}),  (b {id: 'gate_msm_active'})       CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_stale_svinstatus'}),         (b {id: 'pc_survey_status'})      CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_stale_svinstatus'}),         (b {id: 'gate_base_ready'})       CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_bench_false_complete'}),     (b {id: 'pc_survey_status'})      CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_stale_rcvr_mode'}),          (b {id: 'pc_rcvr_mode'})          CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_descriptors_only'}),         (b {id: 'cs_base_outputs'})       CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_descriptors_only'}),         (b {id: 'lg290p_nvm'})            CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_force_noop'}),               (b {id: 'cs_force_base'})         CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_rover_masks_inherited'}),    (b {id: 'lg290p_nvm'})            CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_base_mode_hot_start'}),      (b {id: 'cs_base_enter'})         CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_fixed_battery_backed'}),     (b {id: 'lg290p_nvm'})            CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_ppp_failed_dead_end'}),      (b {id: 'sm_ppp_survey'})         CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_locking_settle_stall'}),     (b {id: 'cs_ppp_lock'})           CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_epe_clamped'}),              (b {id: 'lg290p_receiver'})       CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_epe_clamped'}),              (b {id: 'budget_ppp_survey'})     CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_crtk_idle_drop'}),           (b {id: 'gate_handshake_accepted'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_crtk_idle_drop'}),           (b {id: 'flow_rtcm_from_module'}) CREATE (a)-[:ARISES_FROM {notes: 'burst-per-epoch shape'}]->(b)
MATCH (a {id: 'fm_local_header_160'}),         (b {id: 'buf_local_clients'})     CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_heap_no_mem'}),              (b {id: 'q_sdlog_rows'})          CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_heap_no_mem'}),              (b {id: 'internal_sram'})         CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_1007_never_emitted'}),       (b {id: 'pc_antenna_desc'})       CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_posCheck_premature'}),       (b {id: 'sm_pos_check'})          CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_early_lock_0044'}),          (b {id: 'sm_ppp_survey'})         CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_wifi_board_hardware'}),      (b {id: 'wifi_radio'})            CREATE (a)-[:ARISES_FROM {}]->(b)

MATCH (a {id: 'fm_blocking_caster_write'}),   (b {id: 'rule_drop_not_block'})     CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_usb_cdc_blocking'}),         (b {id: 'rule_no_periodic_serial'}) CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_usb_cdc_blocking'}),         (b {id: 'rule_drop_torn_nmea'})     CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_flash_cache_stall'}),        (b {id: 'rule_core0_no_flash'})     CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_watchdog_storm'}),           (b {id: 'rule_forgive_clamp'})      CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_rate_window_phantom'}),      (b {id: 'rule_rate_window_5s'})     CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_cast_before_convergence'}),  (b {id: 'rule_msm_not_1005'})       CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_stale_svinstatus'}),         (b {id: 'rule_reset_bookkeeping'})  CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_stale_svinstatus'}),         (b {id: 'rule_dont_cast_bad'})      CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_bench_false_complete'}),     (b {id: 'rule_corroborate'})        CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_stale_rcvr_mode'}),          (b {id: 'rule_probe_freshness'})    CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_base_mode_hot_start'}),      (b {id: 'rule_read_back'})          CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_fixed_battery_backed'}),     (b {id: 'rule_mode_change'})        CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_fixed_battery_backed'}),     (b {id: 'rule_rover_first'})        CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_rover_masks_inherited'}),    (b {id: 'rule_base_masks_open'})    CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_crtk_idle_drop'}),           (b {id: 'rule_stage_not_discard'})  CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_epe_clamped'}),              (b {id: 'rule_scatter_primary'})    CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_antenna_misdiagnosis'}),     (b {id: 'rule_spec_silence'})       CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_blocking_caster_write'}),   (b {id: 'rule_one_connect_per_pass'}) CREATE (a)-[:MOTIVATED_RULE {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 17. Recovery levers — the menu the graph implies, ranked
// ─────────────────────────────────────────────────────────────────────────────

CREATE (lev_psram:Lever {id: 'lever_psram_queue', name: 'Move the row queue and stream buffer to PSRAM', rank: 1, recovery_bytes_estimate: 70240, risk: 'low', status: 'open', detail: 'xQueueCreateStatic / xStreamBufferCreateStatic with storage from heap_caps_malloc(MALLOC_CAP_SPIRAM). The rover already does exactly this. Recovers the queue (53856) and the stream buffer (16384) from the pool SDMMC and lwIP need.', risk_detail: 'Adds an allocation-failure path; fall back to internal on failure as the rover does.'})
CREATE (lev_rings:Lever {id: 'lever_capture_rings_psram', name: 'Move the three capture rings to PSRAM', rank: 2, recovery_bytes_estimate: 16384, risk: 'low', status: 'open', detail: 'Served only to a browser; nothing about them needs DMA or internal placement.'})
CREATE (lev_hdr:Lever {id: 'lever_caster_headers', name: 'Shrink or share caster response header buffers', rank: 3, recovery_bytes_estimate: 8000, risk: 'low', status: 'open', detail: 'Two 1024-byte header buffers per slot x 6 slots is 12 kB for text that is only ever a few hundred bytes; lastResponseHeader could be a single shared copy plus a per-slot summary.'})
CREATE (lev_svin:Lever {id: 'lever_svin_mode_freshness', name: 'Freshness-bound g_probedSvinMode', rank: 4, recovery_bytes_estimate: 0, risk: 'low', status: 'open', detail: 'Stamp a millis on every CFGSVIN reply and treat the value as unknown past PROBE_MODE_TRUST_MS, as g_probedRcvrMode already is. Closes the last unbounded probe consumer (output watchdog, venue check).'})
CREATE (lev_verify:Lever {id: 'lever_verify_poscheck_escape', name: 'Verify the venue-check rover escape', rank: 5, recovery_bytes_estimate: 0, risk: 'low', status: 'open', detail: 'Call verifyEscapedFixedBase() after the Idle-branch escape and escalate to PQTMSRR, instead of spending the collection window discovering an echo.'})
CREATE (lev_pos:Lever {id: 'lever_poscheck_converged', name: 'Gate venue-check collection on convergence', rank: 6, recovery_bytes_estimate: 0, risk: 'moderate', status: 'offered, not built', detail: 'Collect only while epe_2d < thresh/2 and satellites exceed a floor, so a coarse first fix cannot read as a move.'})
CREATE (lev_ack:Lever {id: 'lever_savepar_ack', name: 'Check the PQTMSAVEPAR acknowledgement', rank: 7, recovery_bytes_estimate: 0, risk: 'low', status: 'open', detail: 'Eleven send sites, one parser. Capturing OK/ERROR into a probe cache with a timestamp turns a silent decline into an event row.'})
CREATE (lev_sdfull:Lever {id: 'lever_sd_full_guard', name: 'Detect a full card', rank: 8, recovery_bytes_estimate: 0, risk: 'low', status: 'open', detail: 'Check the println() return in writeLine() and free space at rotation; log an event and stop the sat channel first, since it is 90 % of the volume.'})
CREATE (lev_ladder:Lever {id: 'lever_measure_ladder', name: 'Measure the boot heap ladder and task watermarks', rank: 9, recovery_bytes_estimate: 0, risk: 'none', status: 'open', detail: 'Print free internal after each setup() stage and uxTaskGetStackHighWaterMark for both tasks every status row, as the rover does. Frees nothing; makes every other lever measurable.'})
CREATE (lev_dns:Lever {id: 'lever_async_dns', name: 'Bound the DNS lookup', rank: 10, recovery_bytes_estimate: 0, risk: 'moderate', status: 'open', detail: 'The one unbounded, non-draining block on loop(). Resolve on the WiFi task, or pump processGnssSerial() around a getaddrinfo with a timeout.', risk_detail: 'Touches the caster connect path that took 17 sessions to stabilise.'})

MATCH (a {id: 'lever_psram_queue'}),          (b {id: 'q_sdlog_rows'})      CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_psram_queue'}),          (b {id: 'sb_raw_rtcm'})       CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_psram_queue'}),          (b {id: 'psram'})             CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_capture_rings_psram'}),  (b {id: 'buf_caster_tx_capture'}) CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_capture_rings_psram'}),  (b {id: 'buf_rtcm_valid_capture'}) CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_caster_headers'}),       (b {id: 'buf_casters'})       CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_svin_mode_freshness'}),  (b {id: 'pc_svin_mode'})      CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_verify_poscheck_escape'}), (b {id: 'sm_pos_check'})    CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_poscheck_converged'}),   (b {id: 'sm_pos_check'})      CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_savepar_ack'}),          (b {id: 'flow_pqtm_cmd'})     CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_sd_full_guard'}),        (b {id: 'sd_bus'})            CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_measure_ladder'}),       (b {id: 'gap_boot_heap_ladder'}) CREATE (a)-[:RESOLVES {}]->(b)
MATCH (a {id: 'lever_measure_ladder'}),       (b {id: 'gap_task_stacks'})   CREATE (a)-[:RESOLVES {}]->(b)
MATCH (a {id: 'lever_async_dns'}),            (b {id: 'gap_dns_blocking'})  CREATE (a)-[:RESOLVES {}]->(b)
MATCH (a {id: 'lever_savepar_ack'}),          (b {id: 'gap_pqtm_ack'})      CREATE (a)-[:RESOLVES {partial: true}]->(b)
MATCH (a {id: 'lever_sd_full_guard'}),        (b {id: 'gap_sd_full'})       CREATE (a)-[:RESOLVES {}]->(b)
MATCH (a {id: 'lever_poscheck_converged'}),   (b {id: 'fm_posCheck_premature'}) CREATE (a)-[:RESOLVES {}]->(b)
MATCH (a {id: 'lever_psram_queue'}),          (b {id: 'fm_heap_no_mem'})    CREATE (a)-[:RESOLVES {}]->(b)
MATCH (a {id: 'lever_async_dns'}),            (b {id: 'fm_blocking_caster_write'}) CREATE (a)-[:RISKS_REPEATING {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 18. Runbooks — triage in the order the gates are evaluated
// ─────────────────────────────────────────────────────────────────────────────

CREATE (rb_cast:Runbook {id: 'rb_not_casting', name: 'Base is not casting', steps: ['status CSV: c0_state / c0_err — Held with a reason means a gate; Error with a reason means the caster; Disabled means the operator', 'Held: read the reason in gate order — survey in progress (wait), rover mode (mode never took: check event log for Escape verify / Base mode confirmed), no satellites (antenna), no MSM (output watchdog should repair within 120 s; if not, retained config), CRC rate (UART integrity: loop_max_ms, uart_high_water, nmea_checksum_failures), reference accuracy', 'venueConfirmed false: pc_state is collecting or the check never resolved — pc_fixes, pc_epe, pc_dist, pc_thresh tell which branch', 'base_ready 0 with svin_valid 2: signal corroboration failed — sats column', 'wifi 0: STA rotation; AP occupancy pauses it for 60 s after activity', 'Error: lastError and the verbatim caster banner in /api/status resp; accepted-then-closed-before-any-data is the idle-source shape'], source: 'code'})
CREATE (rb_fixed:Runbook {id: 'rb_stuck_fixed', name: 'Module keeps publishing an old coordinate', steps: ['event log: Escape verify lines — STILL reports rcvrMode=2 svinMode=2 after PQTMSRR is the retained-config case', 'survey echoFault or pc_state timeout with identical fixes confirms it', 'ESP32-side clearing (reflash, erase, /api/clearpos) does not touch it; FORCE_CLEAR_POSITION_ON_BOOT commands the module but can be refused too', 'disconnect the LG290P backup battery AND all power, then boot'], source: 'measured'})
CREATE (rb_ppp:Runbook {id: 'rb_survey_not_converging', name: 'PPP survey never accepts fixes', steps: ['/api/ppp gate note names the rejecting test', 'EPE best 2D >= 1.300 for the whole window is the clamp, not the antenna; scatter sigma is the number that matters', 'scatter not yet measured for over 60 s at 1 Hz means GGA is not arriving at 1 Hz: telemetry rates or a rover-mode PQTMEPE form', 'E6 sats 0 with C/N0 blank means no HAS data — the survey will lock the autonomous mean (src ppp-auto) and the saved accuracy will say so', 'zero valid fixes for the whole window is sky, and recovery retries after 60 s once satellites appear'], source: 'measured'})
CREATE (rb_reboot:Runbook {id: 'rb_reboot_loop', name: 'Base restarts unexpectedly', steps: ['first serial line after boot: reset_reason', 'BROWNOUT is the 3.3 V rail, not software', 'TASK_WDT / INT_WDT: something blocked a core — the last event rows and loop_max_ms before the gap', 'PANIC with no backtrace: stack overflow is the usual shape; both task stacks are unmeasured', 'heap_min_int trend across the session before the reset'], source: 'code'})
CREATE (rb_uart:Runbook {id: 'rb_uart_integrity', name: 'RTCM output dropping', steps: ['diff rtcm_crc_fail / rtcm_frame_fail / nmea_checksum_failures between status rows — nmea_framer_desync is benign background', 'loop_max_ms high AND uart_high_water high: our stall overran the ring (which call: a web download, a connect, a receiver reset)', 'both low with failures climbing: bytes lost on the wire — baud or signal integrity, not scheduling', 'rtcm_fps is a 5 s mean; a single low row is not a dropout', 'sat CSV shows whether C/N0 moved; the receiver going quiet is not an uplink fault'], source: 'measured'})

MATCH (a {id: 'rb_not_casting'}),          (b {id: 'gate_ready_to_stream'}) CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_not_casting'}),          (b {id: 'sm_caster'})            CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_stuck_fixed'}),          (b {id: 'lg290p_nvm'})           CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_stuck_fixed'}),          (b {id: 'fm_fixed_battery_backed'}) CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_survey_not_converging'}), (b {id: 'sm_ppp_survey'})       CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_survey_not_converging'}), (b {id: 'fm_epe_clamped'})      CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_reboot_loop'}),          (b {id: 'inst_boot_line'})       CREATE (a)-[:USES {}]->(b)
MATCH (a {id: 'rb_uart_integrity'}),       (b {id: 'inst_status_csv'})      CREATE (a)-[:USES {}]->(b)
MATCH (a {id: 'rb_uart_integrity'}),       (b {id: 'budget_loop_pass'})     CREATE (a)-[:TRIAGES {}]->(b)
MATCH (r:Runbook), (i {id: 'inst_event_csv'}) CREATE (r)-[:USES {}]->(i)


// ─────────────────────────────────────────────────────────────────────────────
// 19. Worked queries
// ─────────────────────────────────────────────────────────────────────────────
//
// Everything that must be true for a byte of RTCM to leave the device, top down:
//   MATCH p = (:Gate {id: 'gate_ready_to_stream'})-[:REQUIRES|CONSULTS*1..3]->(g)
//   RETURN [n IN nodes(p) | n.name] AS chain
//
// Every probe cache without a freshness bound, and who reads it:
//   MATCH (p:ProbeCache) WHERE p.freshness_bound_ms IS NULL
//   OPTIONAL MATCH (x)-[:READS]->(p)
//   RETURN p.variable, p.consumers, collect(x.name)
//
// Every place a command sequence stamps intent instead of reading back:
//   MATCH (c:CommandSequence)-[s:STAMPS_INTENT_ON]->(p:ProbeCache) RETURN c.name, p.variable, s.notes
//
// Open rule violations:
//   MATCH (x)-[v:VIOLATES]->(r:Rule) WHERE v.status STARTS WITH 'open' RETURN x.id, r.text, v.detail
//
// What consumes internal SRAM, largest first, and what could move to PSRAM:
//   MATCH (x)-[c:ALLOCATED_IN|CONSUMES]->(:Resource {id: 'internal_sram'})
//   RETURN x.name, c.bytes, c.relocatable ORDER BY c.bytes DESC
//
// How long each command sequence blocks loop(), and what it starves:
//   MATCH (c:CommandSequence)-[b:BLOCKS]->(t:Task) RETURN c.name, b.ms, b.starves ORDER BY b.ms DESC
//
// Failure modes still open or awaiting field validation:
//   MATCH (f:FailureMode) WHERE f.status CONTAINS 'open' OR f.status CONTAINS 'awaiting'
//   OPTIONAL MATCH (f)-[:ARISES_FROM]->(n) RETURN f.name, f.status, collect(n.id)
//
// Which failure modes arose from the receiver's retained store:
//   MATCH (f:FailureMode)-[:ARISES_FROM]->(:Store {id: 'lg290p_nvm'}) RETURN f.name, f.status
//
// The rule each failure mode paid for:
//   MATCH (f:FailureMode)-[:MOTIVATED_RULE]->(r:Rule) RETURN r.text, collect(f.name)
//
// Instrumentation gaps by priority, and the lever that would close each:
//   MATCH (g:InstrumentationGap) OPTIONAL MATCH (l:Lever)-[:RESOLVES]->(g)
//   RETURN g.priority, g.subject, g.missing, l.name ORDER BY g.priority
//
// Levers by rank with what each risks repeating:
//   MATCH (l:Lever) OPTIONAL MATCH (l)-[:RISKS_REPEATING]->(f:FailureMode)
//   RETURN l.rank, l.name, l.recovery_bytes_estimate, l.risk, f.name ORDER BY l.rank
//
// Every state machine that commands the receiver, and the transition it asks for:
//   MATCH (s)-[c:COMMANDS|TRANSITIONS]->(:StateMachine {id: 'sm_lg290p_mode'}) RETURN s.name, c.on, c.to
//
// The boot ladder with blocking bounds:
//   MATCH (s:BringUpStage) RETURN s.seq, s.label, s.blocking_bound_ms, s.cost_bytes ORDER BY s.seq
