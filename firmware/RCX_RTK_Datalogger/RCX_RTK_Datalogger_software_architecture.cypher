// RCX RTK Datalogger — software architecture knowledge graph
//
// Companion to RCX_RTK_Datalogger_wiring.cypher. That graph models the physical
// harness; this one models the running software: translation units, RTOS tasks,
// the resources they consume, the budgets governing that consumption, the locks
// and queues they contend over, the data that flows between them, and the failure
// modes already observed in the field.
//
// Provenance convention. Every quantity carries a `source` property:
//     'code'      — read directly out of the firmware
//     'measured'  — read out of a field serial capture (heap trace, watermarks)
//     'derived'   — computed from code or measured values
//     'estimate'  — not yet measured; treat as a hypothesis, not a fact
//     'unknown'   — the value is not currently observable
// A budget whose `measured` is 'unknown' is an instrumentation gap, and those are
// modelled explicitly as InstrumentationGap nodes rather than left implicit.
//
// Units are named in the property (bytes, ms, hz) so nothing is ambiguous.
//
// Layers. §1-§16 model how the firmware runs: tasks and cores, the resources
// and budgets they spend, the locks and queues they share, the data that flows
// between them, and what has failed. §17-§22 add what the firmware is made of
// and how it behaves: cone survey mode, which task runs each module and what
// each module calls, the libraries underneath, the external systems and
// interfaces (including every HTTP endpoint), the state machines, the shared
// state and who writes it, NVS and compile-time configuration, and latent
// defects found by reading the source. Code-sourced nodes carry a `cite`
// (file:line) where one line of source settles the claim.


// ─────────────────────────────────────────────────────────────────────────────
// 1. System and execution substrate
// ─────────────────────────────────────────────────────────────────────────────

CREATE (sys:SoftwareSystem {id: 'rcx_fw', name: 'RCX RTK Datalogger firmware', platform: 'ESP32-S3 / Arduino-ESP32 on ESP-IDF', sketch: 'RCX_RTK_Datalogger.ino', binding_constraint: 'internal SRAM', notes: 'Motorsport datalogger: RTK GNSS, vehicle CAN, IMU, SD logging, BLE telemetry to SoloStorm, and a three-page web configuration UI.'})

CREATE (core0:Core {id: 'core0', name: 'ESP32-S3 Core 0', role: 'protocol and I/O core', hosts: 'CAN task, SD logging, WiFi/NTRIP and the RTCM write to UART1, RaceCapture TCP, Display task (including cone survey), dbcTask', utilisation_status: 'not measured; no starvation observed', source: 'measured'})
CREATE (core1:Core {id: 'core1', name: 'ESP32-S3 Core 1', role: 'application core', hosts: 'Arduino loop(): GNSS NMEA parse, 50 Hz IMU read and publish, die temperature and thermal gates, BLE frame assembly and TX pump, SD row enqueue, heap low-water sampling, loop-phase instrument', utilisation_status: 'adequate', source: 'measured'})


// ─────────────────────────────────────────────────────────────────────────────
// 2. Resources — what is actually scarce
// ─────────────────────────────────────────────────────────────────────────────

CREATE (r_isram:Resource {id: 'internal_sram', name: 'Internal SRAM heap', caps: 'MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT', size_at_boot_bytes: 247224, steady_state_free_bytes: 29732, largest_free_block_bytes: 14324, scarcity: 'binding', relocatable: false, source: 'measured', notes: 'The only resource this design is short of. Task stacks, lwIP pbufs and SDMMC DMA buffers cannot live anywhere else.'})
CREATE (r_dma:Resource {id: 'dma_pool', name: 'DMA-capable internal memory', caps: 'MALLOC_CAP_DMA', scarcity: 'binding', shares_pool_with: 'internal_sram', observed_low_bytes: 440, observed_low_largest_block_bytes: 44, source: 'measured', notes: 'Not a separate pool so much as a stricter view of the same one. SDMMC transfer buffers come from here.'})
CREATE (r_psram:Resource {id: 'psram', name: 'External PSRAM', size_bytes: 8388608, used_bytes: 104000, utilisation_pct: 1.3, scarcity: 'abundant', relocatable: true, source: 'measured', notes: 'Design axiom: anything that can live here, does.', code_changed_since: 'used_bytes matches the allocations at a 32-byte filename width (dir snapshot 36,864 B); at LOG_NAME_MAX 40 and with the BLE TX slot and LCD sprite the steady figure is about 142 KB, plus 20 KB in cone mode and 208 KB once a DBC is uploaded'})
CREATE (r_flash:Resource {id: 'flash_rodata', name: 'Flash / rodata', scarcity: 'abundant', used_by_web_assets_bytes: 60000, web_asset_bytes_code: 69474, source: 'derived', notes: 'Pages and shared assets are served straight out of memory-mapped flash with no RAM copy.'})
CREATE (r_uart1:Resource {id: 'uart1', name: 'UART1 to LG290P GNSS', baud: 460800, framing: '8N1', line_rate_bytes_per_s: 46080, measured_rx_bytes_per_s: 8700, utilisation_pct: 19, scarcity: 'abundant', source: 'derived', notes: 'RX load derived from the wire/5s sentence counters at a typical 70 B NMEA sentence.', code_check: 'the configured message set (GSA every fix across 6 constellations, 8-decimal GGA and RMC, EPE, GSV) is about 14-17 kB/s, 30-37 % of the line; 8,700 B/s fits only if GSA was running at 1 Hz, i.e. the GSA write had not stuck in the measured session'})
CREATE (r_uart0:Resource {id: 'uart0', name: 'USB CDC console (native USB, GPIO19/20)', role: 'serial diagnostics', note: 'The console is the ESP32-S3 native USB Serial/JTAG, not UART0; UART0 on GPIO43/44 is unused by the firmware.', scarcity: 'abundant', hazard: 'Serial must never block loop(): Serial.setTxTimeoutMs(0) under USB CDC, because a blocking console once collapsed BLE to 3-4 Hz. The serial-to-SD tee (DEBUG_SERIAL_TO_SD, default off) is not wired: nothing calls DebugLog::drainToFile() and enabling it fails to compile.', source: 'code'})
CREATE (r_sdbus:Resource {id: 'sd_bus', name: 'SDMMC bus and FatFs volume', scarcity: 'bandwidth abundant, latency contended', sustained_write_class: 'MB/s', measured_aggregate_write_bytes_per_s: 5000, source: 'derived', notes: 'Bandwidth is never the constraint. Mutex hold time and DMA buffer availability are.'})
CREATE (r_twai:Resource {id: 'twai', name: 'TWAI / CAN controller', bitrate: 500000, mode: 'listen-only', scarcity: 'abundant', source: 'code'})

MATCH (a {id: 'dma_pool'}), (b {id: 'internal_sram'}) CREATE (a)-[:SUBSET_OF {notes: 'Exhausting one exhausts the other.'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 3. Modules — translation units and what they own
// ─────────────────────────────────────────────────────────────────────────────

CREATE (m_main:Module {id: 'mod_main', file: 'RCX_RTK_Datalogger.ino', role: 'setup ordering, loop(), task creation, the Display and WiFiNTRIP task bodies, IMU publish, die-temperature sensor, reset reason, CAN TX safety pin, heap and stack diagnostics', owns: 'dataMutex, bring-up sequence, gps/imu/can/status globals, gpsSerial, heap low-water marks'})
CREATE (m_gnss:Module {id: 'mod_gnss', file: 'gnss.cpp', role: 'LG290P configuration and NMEA/PQTM parsing', hazard: 'hot-start contract — see the HOT-START CONTRACT banner above configureLG290P(); wrong command order silently reverts config'})
CREATE (m_can:Module {id: 'mod_can', file: 'can_bus.cpp', role: 'TWAI receive, vehicle profile fingerprinting, sniffer snapshot'})
CREATE (m_imu:Module {id: 'mod_imu', file: 'imu.cpp', role: 'QMI8658 configuration, calibration, sampling'})
CREATE (m_sdlog:Module {id: 'mod_sdlog', file: 'sd_log.cpp', role: 'log channels, row queues, file lifecycle, directory snapshot, tar export; other SD users (web deletes, dbcTask) borrow sdMutex through sdlog_getMutex(); web downloads take no SD lock and rely on the pause requirement and the FatFs volume lock', owns: 'sdMutex, dirSnapLock'})
CREATE (m_web:Module {id: 'mod_web', file: 'webserver.cpp', role: 'three-page UI, JSON endpoints, downloads, chunked write-veto senders'})
CREATE (m_wifi:Module {id: 'mod_wifi', file: 'wifi_mgr.cpp', role: 'NVS network list, AP/STA lifecycle, device identity', hazard: 'ordering contract — re-entering WiFi.mode() rebuilds the lwIP netif'})
CREATE (m_ntrip:Module {id: 'mod_ntrip', file: 'ntrip.cpp', role: 'caster table (defaults plus NVS), direct preferred-mount connect, geographic source-table scan, RTCM relay to the GNSS module, RTCM 1005/1006 baseline check, no-data watchdog, preferred-mount probe, GGA keep-alive'})
CREATE (m_ble:Module {id: 'mod_ble', file: 'ble_racecapture.cpp', role: 'NimBLE GATT server, RaceCapture wire format, 20 Hz frame assembly and TX pump on the loop task; connect and subscribe callbacks on the NimBLE host task'})
CREATE (m_rc:Module {id: 'mod_racecapture', file: 'racecapture.cpp', role: 'RaceCapture protocol over TCP 7223: single-client WiFiServer run from the WiFiNTRIP task'})
CREATE (m_display:Module {id: 'mod_display', file: 'display.cpp', role: 'TFT rendering, status RGB LED, backlight PWM and panel power, cone survey mode; 172x72 16-bit sprite (24,768 B, PSRAM by the TFT_eSprite default)'})
CREATE (m_dbcs:Module {id: 'mod_dbc_store', file: 'dbc_store.cpp', role: 'DBC file storage and the task that owns DBC SD access'})
CREATE (m_dbcp:Module {id: 'mod_dbc_parse', file: 'dbc_parse.cpp', role: 'streaming BO_/SG_ parser and import audit'})
CREATE (m_debug:Module {id: 'mod_debug_log', file: 'debug_log.cpp', role: 'serial-to-SD tee, gated by DEBUG_SERIAL_TO_SD', status: 'compiled out by default; drain unwired; enabling it fails to compile, then to link'})
CREATE (m_thermal:Module {id: 'mod_thermal', file: 'thermal.cpp', role: 'die-temperature Schmitt-trigger gates that dim or blank the LCD and throttle SD logging; never touches GNSS, WiFi, BLE or the user logging flags', gates: 'on/off °C: 100/95 backlight 20 %; 105/100 LCD off and raw CAN dump paused; 112/107 SAT, CAN and IMU logging off; 115/110 GPS rows at 1 Hz; a NAN reading holds the current state', source: 'code', cite: 'thermal.cpp:24-28; thermal.h:34-54'})
CREATE (m_config:Module {id: 'mod_config', file: 'config.cpp / config.h', role: 'compile-time configuration and default casters (NVS may disable or extend them)', hazard: 'the rtk2go username is a placeholder that must be replaced with the operator email, or the caster loses its only feedback channel'})


// ─────────────────────────────────────────────────────────────────────────────
// 4. Tasks — stacks are the largest controllable internal-SRAM line
// ─────────────────────────────────────────────────────────────────────────────

CREATE (t_loop:Task {id: 'task_loop', name: 'loop', core: 'core1', priority: 1, stack_bytes: 8192, stack_high_water_free_bytes: 5784, stack_peak_use_bytes: 2408, owner: 'Arduino core', instrumented: true, source: 'measured'})
CREATE (t_sdlog:Task {id: 'task_sdlog', name: 'SDLog', core: 'core0', priority: 1, stack_bytes: 12288, stack_high_water_free_bytes: 6976, stack_peak_use_bytes: 5312, instrumented: true, trim_verdict: 'leave — FatFs paths run deep', source: 'measured'})
CREATE (t_ntrip:Task {id: 'task_wifintrip', name: 'WiFiNTRIP', core: 'core0', priority: 1, stack_bytes: 12288, stack_high_water_free_bytes: 8684, stack_peak_use_bytes: 3604, instrumented: true, trim_verdict: 'leave — deep path (source-table scan) never exercised in the measured window', source: 'measured'})
CREATE (t_display:Task {id: 'task_display', name: 'Display', core: 'core0', priority: 1, stack_bytes: 6144, stack_high_water_free_bytes: 5624, stack_peak_use_bytes: 2568, stack_known_bad_floor_bytes: 4096, instrumented: true, source: 'measured', notes: 'High-water measured while allocated 8192; trimmed to 6144 on that basis.', deep_path_unmeasured: 'cone survey mode (three nth_element passes over up to 1200 floats, Font 4 at size 2) postdates the watermark'})
CREATE (t_can:Task {id: 'task_can', name: 'CAN', core: 'core0', priority: 2, stack_bytes: 6144, stack_high_water_free_bytes: 5988, stack_peak_use_bytes: 2204, instrumented: true, source: 'measured', notes: 'High-water measured while allocated 8192.'})
CREATE (t_dbc:Task {id: 'task_dbc', name: 'dbcTask', core: 'core0', priority: 1, stack_bytes: 4096, stack_high_water_free_bytes: null, stack_peak_use_bytes: null, instrumented: false, source: 'code'})
CREATE (t_async:Task {id: 'task_async_tcp', name: 'async_tcp', core: 'library-managed', stack_bytes: 8192, stack_high_water_free_bytes: null, stack_peak_use_bytes: null, instrumented: false, owner: 'AsyncTCP library', criticality: 'high', source: 'code', notes: 'Runs every web route handler AND feeds bytes to every in-flight response. Single point of serialisation for the whole web UI.'})
CREATE (t_ble:Task {id: 'task_nimble', name: 'NimBLE host', core: 'library-managed', owner: 'NimBLE', instrumented: false, source: 'code'})
CREATE (t_wifi:Task {id: 'task_wifi_lwip', name: 'WiFi / lwIP', core: 'library-managed', owner: 'ESP-IDF', instrumented: false, source: 'code'})

MATCH (a {id: 'task_loop'}), (b {id: 'core1'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'core0'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'core0'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'core0'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'task_can'}), (b {id: 'core0'}) CREATE (a)-[:RUNS_ON {}]->(b)
MATCH (a {id: 'task_dbc'}), (b {id: 'core0'}) CREATE (a)-[:RUNS_ON {}]->(b)

MATCH (a {id: 'mod_sdlog'}), (b {id: 'task_sdlog'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'task_dbc'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'task_wifintrip'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'task_display'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'task_can'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_HANDLERS_ON {notes: 'The module does not own the task; it registers callbacks that execute on it.'}]->(b)

MATCH (t:Task), (r {id: 'internal_sram'}) WHERE t.stack_bytes IS NOT NULL
CREATE (t)-[:CONSUMES {kind: 'task_stack', bytes: t.stack_bytes, relocatable: false, source: 'code', notes: 'Task stacks cannot be placed in PSRAM.'}]->(r)


// ─────────────────────────────────────────────────────────────────────────────
// 5. Bring-up ladder — where the 247 KB actually goes
// ─────────────────────────────────────────────────────────────────────────────

CREATE (s00:BringUpStage {id: 'stage_boot',        seq: 0,  label: 'boot',                free_after_bytes: 247224, cost_bytes: 0,     largest_block_bytes: 188404, source: 'measured', notes: 'Already includes wifi_init() (NVS read and AP name), which runs before the first trace.'})
CREATE (s01:BringUpStage {id: 'stage_display',     seq: 1,  label: 'display init',         free_after_bytes: 246780, cost_bytes: 444,   source: 'measured', code_changed_since: 'display_init() now also initialises the RGB status LED'})
CREATE (s02:BringUpStage {id: 'stage_imu',         seq: 2,  label: 'imu init',             free_after_bytes: 245768, cost_bytes: 1012,  source: 'measured'})
CREATE (s03:BringUpStage {id: 'stage_gnss',        seq: 3,  label: 'gnss init',            free_after_bytes: 245496, cost_bytes: 272,   source: 'measured'})
CREATE (s04:BringUpStage {id: 'stage_ble',         seq: 4,  label: 'NimBLE init',          free_after_bytes: 179268, cost_bytes: 66228, rank: 1, source: 'measured', notes: 'Largest single consumer in the build.'})
CREATE (s05:BringUpStage {id: 'stage_can',         seq: 5,  label: 'CAN init + task',      free_after_bytes: 170024, cost_bytes: 9244,  source: 'measured', code_changed_since: 'measured with an 8192 B CAN stack; the stack is now 6144'})
CREATE (s06:BringUpStage {id: 'stage_sdlog',       seq: 6,  label: 'SD log init + task',   free_after_bytes: 156560, cost_bytes: 13464, source: 'measured', notes: 'Queues are PSRAM; this is the 12,288 B task stack, its TCB and the two mutexes. The SD card mounts inside the task 800 ms later, so the SDMMC driver cost lands in a later stage.'})
CREATE (s07:BringUpStage {id: 'stage_webroutes',   seq: 7,  label: 'web routes registered',free_after_bytes: 145684, cost_bytes: 10876, source: 'measured', notes: 'Also contains the untraced dbc_init(), which creates dbcTask with a 4096 B stack.'})
CREATE (s08:BringUpStage {id: 'stage_ntriptask',   seq: 8,  label: 'WiFiNTRIP task',       free_after_bytes: 132508, cost_bytes: 13176, source: 'measured'})
CREATE (s09:BringUpStage {id: 'stage_displaytask', seq: 9,  label: 'Display task',         free_after_bytes: 123428, cost_bytes: 9080,  source: 'measured', code_changed_since: 'measured with an 8192 B Display stack; the stack is now 6144. WiFiNTRIP runs ntrip_init() on core 0 while this trace is taken, so the delta can include its allocations.'})
CREATE (s10:BringUpStage {id: 'stage_apbegin',     seq: 10, label: 'WiFi AP bring-up',     free_after_bytes: 71400,  cost_bytes: 52028, rank: 2, source: 'measured', notes: 'Driver init plus AP netif and DHCP server. See lever_ap_netif.'})
CREATE (s11:BringUpStage {id: 'stage_staconnect',  seq: 11, label: 'WiFi STA connect',     free_after_bytes: 48224,  cost_bytes: 23176, rank: 3, source: 'measured'})
CREATE (s12:BringUpStage {id: 'stage_webbegin',    seq: 12, label: 'web server begin',     free_after_bytes: 30208,  cost_bytes: 18016, rank: 4, source: 'measured', notes: 'async_tcp task stack plus listen socket.'})
CREATE (s13:BringUpStage {id: 'stage_rcbegin',     seq: 13, label: 'RaceCapture begin',    free_after_bytes: 29732,  cost_bytes: 476,   source: 'measured'})

MATCH (a:BringUpStage), (b:BringUpStage) WHERE b.seq = a.seq + 1 CREATE (a)-[:NEXT {}]->(b)
MATCH (s:BringUpStage), (r {id: 'internal_sram'}) WHERE s.cost_bytes > 0
CREATE (s)-[:CONSUMES {kind: 'bring_up', bytes: s.cost_bytes, source: 'measured'}]->(r)

MATCH (a {id: 'stage_ble'}),        (b {id: 'mod_ble'})     CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_apbegin'}),    (b {id: 'mod_wifi'})    CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_staconnect'}), (b {id: 'mod_wifi'})    CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_webbegin'}),   (b {id: 'mod_web'})     CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_sdlog'}),      (b {id: 'mod_sdlog'})   CREATE (a)-[:ATTRIBUTED_TO {}]->(b)
MATCH (a {id: 'stage_webroutes'}),  (b {id: 'mod_dbc_store'}) CREATE (a)-[:ATTRIBUTED_TO {part: 'dbcTask stack, about 4.4 KB'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 6. Buffers and queues — the relocation story
// ─────────────────────────────────────────────────────────────────────────────

CREATE (q_gpscan:Queue {id: 'q_gps_can', name: 'GPS+CAN row queue', bytes: 28320, code_bytes: 29280, placement: 'psram', fallback: 'internal on PSRAM failure', producer_rate_hz: 20, source: 'measured', code_changed_since: 'LogRecord grew to 488 B (BLE delivery counters, speed age, DOPs, GSA satellites); 60 x 488 = 29,280 B'})
CREATE (q_imu:Queue    {id: 'q_imu',     name: 'IMU row queue',     bytes: 3360,  placement: 'psram', producer_rate_hz: 50, producer_rate: '50 Hz, 1 Hz after 60 s motionless (not while cone survey mode is on), none while the thermal SAT/CAN/IMU gate is set', source: 'measured'})
CREATE (q_sat:Queue    {id: 'q_sat',     name: 'Satellite row queue', bytes: 2640, placement: 'psram', producer_rate_hz: null, source: 'measured'})
CREATE (q_canraw:Queue {id: 'q_canraw',  name: 'Raw CAN frame queue', bytes: 32768, placement: 'psram', item_bytes: 16, producer_rate: 'bus rate', source: 'measured', notes: 'Throughput-gated: the sniffer forces the typed channels off by default.'})

CREATE (b_dirsnap:Buffer {id: 'buf_dir_snapshot', name: 'Directory snapshot (double-buffered)', entries: 512, bytes_at_name_width_32: 36864, bytes_at_name_width_40: 45056, placement: 'psram', source: 'measured'})
CREATE (b_uartrx:Buffer  {id: 'buf_uart1_rx', name: 'UART1 RX ring', bytes: 4096, placement: 'internal', coverage_ms: 89, coverage_basis: 'full line rate (46,080 B/s); at the configured message set (about 14-17 kB/s) the ring covers about 240-290 ms', source: 'code', notes: 'Default is 256 B, which fills in ~5.5 ms at 460800. 4096 lets loop() stall at least 89 ms without byte loss.'})
CREATE (b_tar:Buffer     {id: 'buf_tar_stage', name: 'Tar copy buffer', bytes: 4096, placement: 'psram', fallback: 'internal', source: 'code'})
CREATE (b_ack:Buffer     {id: 'buf_async_ack', name: 'ESPAsyncWebServer per-ack assembly buffer', bytes_typical: 5744, placement: 'internal', lifetime: 'one ack', owner: 'library', source: 'derived', notes: 'CRITICAL: malloc-ed at the size of the whole TCP send window BEFORE the filler callback is consulted. A filler that declines still pays this allocation.'})
CREATE (b_pbuf:Buffer    {id: 'buf_lwip_pbuf', name: 'lwIP PBUF_RAM segment', bytes: 'chunk + ~90', placement: 'internal', owner: 'lwIP', source: 'derived', notes: 'tcp_write returns ERR_MEM when this cannot be allocated; AsyncClient::write() then returns 0.'})
CREATE (b_sddma:Buffer   {id: 'buf_sdmmc_dma', name: 'SDMMC bounce buffer', bytes_approx: 512, placement: 'dma', owner: 'ESP-IDF', lifetime: 'allocated and freed on every unaligned sector write', source: 'estimate', cite: 'sd_log.cpp:527-531 (source comment describing the IDF driver)', notes: 'Failure prints allocate_dma_buf: not enough mem, err=0x101.'})

MATCH (q:Queue), (r {id: 'psram'}) CREATE (q)-[:ALLOCATED_IN {bytes: q.bytes, source: 'measured'}]->(r)
MATCH (a {id: 'buf_dir_snapshot'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 45056, source: 'derived'}]->(b)
MATCH (a {id: 'buf_tar_stage'}),    (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 4096, source: 'code'}]->(b)
MATCH (a {id: 'buf_uart1_rx'}),     (b {id: 'internal_sram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 4096, relocatable: false, source: 'code'}]->(b)
MATCH (a {id: 'buf_async_ack'}),    (b {id: 'internal_sram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 5744, relocatable: false, transient: true, source: 'derived'}]->(b)
MATCH (a {id: 'buf_lwip_pbuf'}),    (b {id: 'internal_sram'}) CREATE (a)-[:ALLOCATED_IN {relocatable: false, transient: true, source: 'derived'}]->(b)
MATCH (a {id: 'buf_sdmmc_dma'}),    (b {id: 'dma_pool'})      CREATE (a)-[:ALLOCATED_IN {bytes: 512, relocatable: false, transient: true, source: 'estimate'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 7. Locks — hold budgets and the timeouts that enforce them
// ─────────────────────────────────────────────────────────────────────────────

CREATE (l_sd:Lock       {id: 'lock_sdmutex',   name: 'sdMutex', guards: 'SD bus / FatFs volume', hold_budget_periodic_ms: 20, hold_budget_user_ms: 200, acquire_timeouts_ms: [50, 200, 300, 500, 1000, 2000], enforcement: 'log writers wait only 50 ms then DROP the row — exceeding the hold budget costs data silently', source: 'code'})
CREATE (l_snap:Lock     {id: 'lock_dirsnap',   name: 'dirSnapLock', guards: 'published directory snapshot', acquire_timeouts_ms: [50, 100], hold_budget_ms: 10, hold_budget_basis: 'design intent; no hold budget appears in the code', on_timeout: 'the snapshot update is skipped silently', source: 'code'})
CREATE (l_data:Lock     {id: 'lock_datamutex', name: 'dataMutex', guards: 'gps / status / can snapshot structs', hold_budget_ms: 1, acquire_timeouts_ms: [2, 5, 10, 50], intended_scope: 'a memcpy of a snapshot struct and nothing else', scope_exceptions: 'the CAN task writes 48 fields under it (pushToGlobal718), with a wheel-speed sort of at most 4 values when 0x30B display speed is absent', source: 'code'})
CREATE (l_sniff:Lock    {id: 'lock_sniffmux',  name: 'sniffMux', guards: 'CAN sniffer snapshot table', source: 'code'})

MATCH (a {id: 'lock_sdmutex'}),   (b {id: 'sd_bus'})       CREATE (a)-[:GUARDS {}]->(b)
MATCH (a {id: 'lock_dirsnap'}),   (b {id: 'buf_dir_snapshot'}) CREATE (a)-[:GUARDS {}]->(b)

MATCH (a {id: 'task_sdlog'}),     (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'writer', timeout_ms: 50,   on_timeout: 'drop the row', source: 'code'}]->(b)
MATCH (a {id: 'task_sdlog'}),     (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'file lifecycle / dir scan slice', timeout_ms: 200, source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'POST /log/delete batch delete', timeout_ms: 2000, hazard: 'blocking here starves every in-flight HTTP response', source: 'code'}]->(b)
MATCH (a {id: 'task_dbc'}),       (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'DBC commit, parse and audit', timeout_ms: 2000, on_timeout: 'skip the pass and retry in 200 ms', no_mutex: 'runs unlocked when sdMutex does not exist (SD logging compiled out)', hold: 'unbounded: scales with DBC size up to 192 KB', source: 'code'}]->(b)
MATCH (a {id: 'task_loop'}),      (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'publish snapshots', timeout_ms: 5, source: 'code'}]->(b)
MATCH (a {id: 'task_can'}),       (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'publish CAN snapshot', timeout_ms: 5, source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: '/status copy-out', timeout_ms: 50, source: 'code'}]->(b)
MATCH (a {id: 'task_nimble'}),    (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'onConnect/onDisconnect status write', timeout_ms: -1, timeout_label: 'portMAX_DELAY', violates: 'rule_no_unbounded_wait', source: 'code'}]->(b)

MATCH (a {id: 'lock_sdmutex'}), (b {id: 'lock_dirsnap'})
CREATE (a)-[:MUST_NOT_NEST_WITH {reason: 'Snapshot updates are made only after sdMutex is released, deliberately outside openFile(), so the two locks are never held together.', cite: 'sd_log.cpp:442-451; webserver.cpp:1908-1910, 1973, 2208-2210', source: 'code'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 8. Data flows
// ─────────────────────────────────────────────────────────────────────────────

CREATE (f_nmea:DataFlow  {id: 'flow_nmea',   name: 'NMEA/PQTM sentences', transport: 'uart1', direction: 'LG290P -> ESP32', rate_hz: 20, bytes_per_s: 8700, source: 'derived'})
CREATE (f_rtcm:DataFlow  {id: 'flow_rtcm',   name: 'RTCM corrections',    transport: 'uart1', direction: 'ESP32 -> LG290P', origin: 'NTRIP caster', source: 'code'})
CREATE (f_ntrip:DataFlow {id: 'flow_ntrip',  name: 'NTRIP stream',        transport: 'tcp', direction: 'caster -> ESP32', source: 'code'})
CREATE (f_can:DataFlow   {id: 'flow_can',    name: 'Vehicle CAN frames',  transport: 'twai', direction: 'vehicle -> ESP32', mode: 'listen-only', rate: 'bus rate at 500 kbps', source: 'code'})
CREATE (f_snap:DataFlow  {id: 'flow_snapshot', name: 'gps/status/can snapshots', transport: 'shared memory under dataMutex', rate_hz: 20, source: 'code'})
CREATE (f_ble:DataFlow   {id: 'flow_ble',    name: 'RaceCapture BLE frames', transport: 'ble_gatt', direction: 'ESP32 -> SoloStorm', rate_hz: 20, contract: 'frames go out on schedule and must contain valid GNSS+CAN+IMU data', source: 'code'})
CREATE (f_rows:DataFlow  {id: 'flow_sd_rows', name: 'CSV rows', transport: 'PSRAM queues', direction: 'loop, GNSS and CAN producers -> SDLog', source: 'code'})
CREATE (f_http:DataFlow  {id: 'flow_http',   name: 'HTTP responses', transport: 'tcp_80', hazard: 'each in-flight response holds an ack buffer plus an lwIP segment out of internal SRAM', source: 'derived'})

MATCH (a {id: 'mod_gnss'}),   (b {id: 'flow_nmea'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_ntrip'}),  (b {id: 'flow_rtcm'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_ntrip'}),  (b {id: 'flow_ntrip'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_can'}),    (b {id: 'flow_can'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_gnss'}),   (b {id: 'flow_snapshot'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_can'}),    (b {id: 'flow_snapshot'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}),   (b {id: 'flow_snapshot'}) CREATE (a)-[:PRODUCES_FLOW {part: 'IMU sample published by loop()'}]->(b)
MATCH (a {id: 'mod_ble'}),    (b {id: 'flow_snapshot'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_ble'}),    (b {id: 'flow_ble'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_sdlog'}),  (b {id: 'flow_sd_rows'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_web'}),    (b {id: 'flow_http'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'flow_nmea'}),  (b {id: 'uart1'}) CREATE (a)-[:TRAVERSES {utilisation_pct: 19}]->(b)
MATCH (a {id: 'flow_rtcm'}),  (b {id: 'uart1'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'flow_can'}),   (b {id: 'twai'})  CREATE (a)-[:TRAVERSES {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 9. Budgets — the governed limits
// ─────────────────────────────────────────────────────────────────────────────

CREATE (bud_pool:Budget    {id: 'budget_free_pool',   name: 'Free internal pool floor', resource: 'internal_sram', budget_bytes: 40960, measured_bytes: 29732, headroom_bytes: -11228, status: 'OVER', rationale: 'One web response (~11 KB) + one SDMMC DMA buffer (4 KB) + source-table scan worst case + fragmentation slack.', code_check: 'the source comment puts the SDMMC bounce buffer at 512 B per call, not 4 KB', source: 'derived'})
CREATE (bud_stacks:Budget  {id: 'budget_task_stacks', name: 'All task stacks', resource: 'internal_sram', budget_bytes: 40960, measured_bytes: 45056, status: 'OVER', scope: 'the five instrumented stacks (loop, SDLog, WiFiNTRIP, Display, CAN); with dbcTask (4,096 B) and async_tcp (8,192 B) the total is 57,344 B', source: 'derived'})
CREATE (bud_net:Budget     {id: 'budget_net_stack',   name: 'WiFi + lwIP + async_tcp', resource: 'internal_sram', budget_bytes: 97280, measured_bytes: 93220, status: 'THIN', source: 'derived'})
CREATE (bud_ble:Budget     {id: 'budget_ble',         name: 'NimBLE', resource: 'internal_sram', budget_bytes: 67584, measured_bytes: 66228, status: 'FIXED COST', source: 'measured'})
CREATE (bud_uart:Budget    {id: 'budget_uart1_rx',    name: 'UART1 RX utilisation', resource: 'uart1', budget_pct: 60, measured_pct: 19, status: 'OK', spend_note: 'This is the one budget with room to spend — extra constellations or GSV rates come from here.', source: 'derived'})
CREATE (bud_ring:Budget    {id: 'budget_uart1_ring',  name: 'UART1 RX ring coverage', resource: 'uart1', budget_ms: 50, measured_ms: 89, status: 'OK', basis: 'coverage at full line rate, a lower bound; not a measurement', source: 'derived'})
CREATE (bud_epoch:Budget   {id: 'budget_epoch',       name: 'GNSS epoch', budget_ms: 50, blocking_budget_ms: 5, status: 'OK', rationale: '20 Hz fix rate. Parse, snapshot copy-out, BLE frame and four queue enqueues all fit inside one epoch; the deep PSRAM queues absorb SD flush stalls up to 811 ms observed.', source: 'derived'})
CREATE (bud_sdhold:Budget  {id: 'budget_sdmutex_hold', name: 'sdMutex hold', budget_periodic_ms: 20, budget_user_ms: 200, measured_ms: null, status: 'UNMEASURED', source: 'derived'})
CREATE (bud_scan:Budget    {id: 'budget_scan_cost',   name: 'NTRIP source-table scan transient', resource: 'internal_sram', budget_bytes: 30000, measured_bytes: null, status: 'ESTIMATE ONLY', source: 'estimate', notes: 'The 30000 B guard was set from this estimate. Newly instrumented via the low-during-scan figure; recalibrate once one scan completes.'})

MATCH (b:Budget), (r:Resource) WHERE b.resource = r.id CREATE (b)-[:GOVERNS {}]->(r)
MATCH (a {id: 'budget_task_stacks'}), (b {id: 'budget_free_pool'}) CREATE (a)-[:COMPETES_WITH {notes: 'Every byte of stack is a byte the free pool does not have.'}]->(b)
MATCH (a {id: 'budget_scan_cost'}),   (b {id: 'budget_free_pool'}) CREATE (a)-[:SIZES {notes: 'The scan worst case is the largest single term in the free-pool floor.'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 10. Instrumentation — and its gaps
// ─────────────────────────────────────────────────────────────────────────────

CREATE (i_heap:Instrument     {id: 'inst_heap',      marker: 'Heap:',              cadence: '30 s', reports: 'internal free, largest block, per-task stack watermarks'})
CREATE (i_lows:Instrument     {id: 'inst_lows',      marker: 'lows since boot',    cadence: '30 s', reports: 'internal and DMA low-water marks'})
CREATE (i_ladder:Instrument   {id: 'inst_ladder',    marker: '[heap]',             cadence: 'boot', reports: 'bring-up ladder'})
CREATE (i_page:Instrument     {id: 'inst_page',      marker: 'GET <path>',         cadence: 'per page', reports: 'DMA free and largest block at send time'})
CREATE (i_stall:Instrument    {id: 'inst_stall',     marker: 'Send stalled',       cadence: 'on trip', reports: 'response deferred to the bounded limit and forced'})
CREATE (i_scan:Instrument     {id: 'inst_scan',      marker: 'Scan start / low during scan', cadence: 'per scan', reports: 'scan transient cost'})
CREATE (i_wire:Instrument     {id: 'inst_wire',      marker: 'wire/5s',            cadence: '5 s', reports: 'NMEA sentence mix and checksum failures'})

MATCH (a {id: 'budget_free_pool'}),   (b {id: 'inst_heap'})  CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'budget_task_stacks'}), (b {id: 'inst_heap'})  CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'budget_uart1_rx'}),    (b {id: 'inst_wire'})  CREATE (a)-[:REPORTED_BY {}]->(b)
MATCH (a {id: 'budget_scan_cost'}),   (b {id: 'inst_scan'})  CREATE (a)-[:REPORTED_BY {}]->(b)

CREATE (g_async:InstrumentationGap  {id: 'gap_async_stack', subject: 'task_async_tcp', missing: 'stack high-water mark', impact: 'The task that runs every route handler has an invisible 8 KB stack; it cannot be budgeted or trimmed.', priority: 'high'})
CREATE (g_dbc:InstrumentationGap    {id: 'gap_dbc_stack',   subject: 'task_dbc', missing: 'stack high-water mark', priority: 'low'})
CREATE (g_hold:InstrumentationGap   {id: 'gap_sd_hold',     subject: 'lock_sdmutex', missing: 'worst-case hold time', impact: 'The hold budget is unenforceable without it; dropped log rows are the only symptom.', priority: 'high'})
CREATE (g_qdepth:InstrumentationGap {id: 'gap_queue_depth', subject: 'PSRAM queues', missing: 'depth high-water mark', impact: 'Cannot tell whether the queues are absorbing flush stalls or merely large.', priority: 'medium'})

MATCH (g:InstrumentationGap), (t:Task) WHERE g.subject = t.id CREATE (g)-[:GAP_IN {}]->(t)
MATCH (a {id: 'gap_sd_hold'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:GAP_IN {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 11. Standing rules — the invariants the budgets rest on
// ─────────────────────────────────────────────────────────────────────────────

CREATE (rule_psram:Rule    {id: 'rule_psram_first', text: 'Anything that can live in PSRAM does.'})
CREATE (rule_async:Rule    {id: 'rule_async_clean', text: 'No SD I/O, no blocking lock acquisition and no unyielded loop on the async_tcp task.'})
CREATE (rule_nest:Rule     {id: 'rule_no_nesting',  text: 'sdMutex is never held across a network operation and never nested with dirSnapLock.'})
CREATE (rule_epoch:Rule    {id: 'rule_epoch_5ms',   text: 'Nothing on the 20 Hz epoch path blocks for more than 5 ms.'})
CREATE (rule_yield:Rule    {id: 'rule_feed_vs_yield', text: 'Feeding the watchdog is not yielding. Long loops must do both.'})
CREATE (rule_stack:Rule    {id: 'rule_stack_evidence', text: 'A stack is sized from a high-water mark taken over a session that exercised its deep paths, not from a quiet idle window.'})
CREATE (rule_name:Rule     {id: 'rule_log_name_max', text: 'Buffers holding filenames use LOG_NAME_MAX so a width mismatch is a compile error, not a silent truncation.'})
CREATE (rule_floor:Rule    {id: 'rule_pool_floor',  text: 'The free internal pool has a floor. Work that would push below it is refused at the endpoint, not allowed to fail deep in a driver.', realization: 'partial: endpoints refuse by concurrency (one download, pause required, SD busy), and responses veto chunks by largest free block; no endpoint checks a numeric floor'})
CREATE (rule_wait:Rule     {id: 'rule_no_unbounded_wait', text: 'No task waits indefinitely for a lock whose budgeted hold time is measured in milliseconds.'})

MATCH (a {id: 'task_async_tcp'}), (b {id: 'rule_async_clean'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'lock_sdmutex'}),   (b {id: 'rule_no_nesting'})  CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'budget_free_pool'}), (b {id: 'rule_pool_floor'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'task_nimble'}),    (b {id: 'rule_no_unbounded_wait'})
CREATE (a)-[:VIOLATES {detail: 'ble_racecapture.cpp takes dataMutex with portMAX_DELAY in the onConnect and onDisconnect host callbacks; the 20 Hz frame path is bounded at 5 ms.', status: 'open', source: 'code'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 12. Failure modes — what has actually gone wrong, and why
// ─────────────────────────────────────────────────────────────────────────────

CREATE (fm_hole:FailureMode {id: 'fm_response_hole', name: 'Response with a hole in the middle', symptom: 'Page cuts off partway then appears to restart; JSON endpoints return unparseable text and fields stay blank.', root_cause: 'AsyncClient::write() returns 0 on lwIP pbuf exhaustion, but both response classes advance _sentLength before or regardless of the write result, so the unwritten span is discarded silently.', status: 'fixed for the pages, assets and four JSON routes; open for file downloads and plain send() responses', fix: 'Chunked filler callbacks that return RESPONSE_TRY_AGAIN while the internal largest free block cannot support the write.', still_exposed: 'GET /log/<f> and /export.tar fillers (raw file.read, no veto); AsyncBasicResponse routes such as /dbc (up to 4 KB audit text), /wifi and /casters', source: 'measured'})
CREATE (fm_livelock:FailureMode {id: 'fm_veto_livelock', name: 'Write-veto livelock', symptom: 'One page load, then a continuous sdmmc allocate_dma_buf 0x101 storm and a monotonic internal-heap slide from ~25 KB to ~2 KB that never recovers.', root_cause: 'AsyncAbstractResponse::_ack malloc-s a full send window BEFORE consulting the filler, so an unbounded veto repeats a ~5.7 KB allocate/free at poll cadence forever, pinning the pool SDMMC needs.', status: 'fix delivered, awaiting field test', fix: 'Bounded declines with forced progress, reserve lowered, chunk cap removed.', source: 'measured'})
CREATE (fm_starve:FailureMode {id: 'fm_batch_delete_starve', name: 'Batch delete starves in-flight responses', symptom: 'Corrupted page loads correlating with delete bursts.', root_cause: 'The remove loop runs on async_tcp and fed the watchdog without ever yielding.', status: 'fixed', fix: 'Periodic vTaskDelay in all three batch loops.', source: 'measured'})
CREATE (fm_scan:FailureMode {id: 'fm_scan_gate', name: 'Source-table scan permanently gated', symptom: 'rtk2go scan skipped; internal heap low (29xxx B free) against a 30000 B guard.', root_cause: 'Steady-state internal free sits ~700 B below a guard threshold that was itself an estimate.', consistency_note: 'the graph steady state of 29,732 B is 268 B below the 30,000 B guard; the ~700 B figure comes from a different session', also_gates: 'probePreferredLive(), silently, so the 5-minute move back to the RCX1 mount never fires while this holds (ntrip.cpp:780)', status: 'open', fix: 'Stack trims returned ~4 KB; guard to be recalibrated from the new scan measurement.', source: 'measured'})
CREATE (fm_dmawrite:FailureMode {id: 'fm_sd_dma_fail', name: 'SD write fails with 0x101', symptom: 'sdmmc_cmd allocate_dma_buf not enough mem; rows dropped.', root_cause: 'DMA-capable pool exhausted by concurrent web responses.', status: 'mitigated', fix: 'printlnRetry (3 attempts, 5 ms apart) plus the concurrency and veto work.', not_covered: 'the canraw writer and its UTC sync row use a plain println with the result unchecked', source: 'measured'})
CREATE (fm_netif:FailureMode {id: 'fm_netif_orphan', name: 'Web server socket orphaned from the live interface', symptom: 'WiFi connects and NTRIP flows but the web page never responds.', root_cause: 'The listening socket was bound against an interface that a later WiFi.mode()/reconnect rebuilt.', status: 'fixed', fix: 'WiFi.mode() is set exactly once, attempts never disconnect first, and the web server binds after the AP netif is up, so the listen socket is bound once and never orphaned.', source: 'measured', notes: 'This is the precedent that makes lever_ap_netif risky.'})
CREATE (fm_stack:FailureMode {id: 'fm_display_stack', name: 'Display task stack overflow', symptom: 'rst:0xc with no guru meditation; USB CDC drops before the backtrace flushes.', root_cause: 'Display task at 4096 B stack.', status: 'fixed', fix: 'Raised to 8192, since measured down to 6144.', source: 'measured', notes: 'Establishes 4096 as a known-bad floor and shows this failure gives no diagnostic warning.'})

MATCH (a {id: 'fm_response_hole'}),      (b {id: 'buf_lwip_pbuf'})   CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_veto_livelock'}),      (b {id: 'buf_async_ack'})   CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_batch_delete_starve'}),(b {id: 'task_async_tcp'})  CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_scan_gate'}),          (b {id: 'budget_free_pool'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_sd_dma_fail'}),        (b {id: 'dma_pool'})        CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_netif_orphan'}),       (b {id: 'mod_wifi'})        CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_display_stack'}),      (b {id: 'task_display'})    CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'fm_batch_delete_starve'}),(b {id: 'rule_feed_vs_yield'}) CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_response_hole'}),      (b {id: 'rule_pool_floor'})    CREATE (a)-[:MOTIVATED_RULE {}]->(b)
MATCH (a {id: 'fm_display_stack'}),      (b {id: 'rule_stack_evidence'}) CREATE (a)-[:MOTIVATED_RULE {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 13. Recovery levers — the menu the budgets imply
// ─────────────────────────────────────────────────────────────────────────────

CREATE (lev_ap:Lever    {id: 'lever_ap_netif', name: 'Retire the AP netif properly', rank: 1, recovery_bytes_estimate: '5000-15000', confidence: 'low', risk: 'high', status: 'tabled by user', detail: 'apStop() calls WiFi.softAPdisconnect(false). In core 3.3.8 that only reapplies an empty AP configuration (empty SSID, open, channel 1): the AP stays enabled and keeps beaconing, the mode stays WIFI_AP_STA, and the AP netif and DHCP server stay allocated for the rest of the boot. Moving to WIFI_STA would release them and stop the beacons.', risk_detail: 'Re-entering WiFi.mode() rebuilds the lwIP netif — the precedent for fm_netif_orphan.'})
CREATE (lev_async:Lever {id: 'lever_async_stack', name: 'Instrument then trim async_tcp stack', rank: 2, recovery_bytes_estimate: '0-4096', risk: 'low to measure, moderate to trim', status: 'open'})
CREATE (lev_ble:Lever   {id: 'lever_nimble', name: 'Reduce NimBLE footprint', rank: 3, recovery_bytes_estimate: 'potentially large', risk: 'high', status: 'open', detail: 'Trimming the GATT table or lowering the configured connection count would reduce the footprint.', advertising_note: 'The boot log line Cannot add UUID is the 128-bit NUS UUID not fitting beside the name in the 31 B primary packet; the primary packet stays a legal 23 B. NimBLE 2.2 and later moves NUS to the scan response; NimBLE 2.1.x drops it, because enableScanResponse(true) is called after addServiceUUID (ble_racecapture.cpp:1003-1008).', risk_detail: 'Touches the field-verified BLE contract, which is the reason the rover exists.'})
CREATE (lev_static:Lever {id: 'lever_static_to_psram', name: 'Move remaining static tables to PSRAM', rank: 4, recovery_bytes_estimate: 'up to about 27,000', risk: 'low', status: 'open', detail: 'Web tables exportSelectNames, pageNames, snapNames (5,288 B); dir-snapshot fallback tables (1,408 B); BLE meta and sample build buffers (10,240 B), RaceCapture sample buffer (3,072 B), receive buffers and ACK ring (704 B), CAN sniffer table and index (6,656 B).', risk_detail: 'Adds allocation-failure paths where none exist today.'})
CREATE (lev_gate:Lever  {id: 'lever_recalibrate_gate', name: 'Recalibrate the 30000 B scan guard from measurement', rank: 5, recovery_bytes_estimate: 0, risk: 'none', status: 'instrumented, awaiting data', detail: 'Frees nothing but may resolve the symptom outright.'})

MATCH (a {id: 'lever_ap_netif'}),   (b {id: 'stage_apbegin'})    CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_async_stack'}),(b {id: 'task_async_tcp'})   CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_nimble'}),     (b {id: 'stage_ble'})        CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_static_to_psram'}), (b {id: 'internal_sram'}) CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_recalibrate_gate'}), (b {id: 'budget_scan_cost'}) CREATE (a)-[:TARGETS {}]->(b)
MATCH (a {id: 'lever_ap_netif'}),   (b {id: 'fm_netif_orphan'})  CREATE (a)-[:RISKS_REPEATING {}]->(b)
MATCH (a {id: 'lever_recalibrate_gate'}), (b {id: 'fm_scan_gate'}) CREATE (a)-[:RESOLVES {}]->(b)

// ─────────────────────────────────────────────────────────────────────────────
// 14. BLE telemetry loss — root-caused 2026-08-22 (arrival-lattice analysis)
// ─────────────────────────────────────────────────────────────────────────────
// Measured chain, from the 08-22 drive's own device SD log + SoloStorm export:
// producer clean at 20.00 Hz; WiFi/NTRIP up all drive with zero stall overlap;
// the granted CONNECTION INTERVAL recovered from the arrival lattice is 15 ms
// (12 units — lattice score 0.465 vs 0.089 noise floor), i.e. the central
// GRANTS the firmware's fast-interval request. Yet data crossed on only ~629
// connection events in 281 s (~2.2/s) out of ~66/s available, in clumps of
// 2-3 complete samples 2 ms apart. When an event opens, everything queued
// drains instantly — the bottleneck is events OPENING, and the MASTER decides
// that. Delivery: 4.93 Hz of 20 Hz produced (75.4% loss) vs 0.86% on the
// 08-21 commute with the same firmware and same hotspot arrangement.

CREATE (fm_anchor:FailureMode {id: 'fm_ble_master_anchor_skipping', name: 'BLE master skips connection anchors under its own radio contention', symptom: '75.4% SoloStorm sample loss; data-bearing events every 150-900 ms against a granted 15 ms interval; stalls to 3.3 s; two sessions ended in mid-motion disconnects.', root_cause: 'Anchor starvation is measured; WHICH radio steals the anchors is not. Corrected 08-22 config (per operator): phone = hotspot, tablet = SoloStorm BLE central — the master was NOT the hotspot. Open contenders, all producing this identical fingerprint: (a) tablet-side WiFi activity (joined to the hotspot and/or Android periodic scans) making the master skip anchors; (b) rover-side coexistence denying the slave its anchors (a missed slave anchor also carries no data); (c) near-field receiver desense if the hotspot phone sits beside the tablet or rover; (d) general home-channel 2.4 GHz congestion deferring everything. The firmware amplified whichever it is: an effective ~2-sample queue plus the 25 ms abandon-on-congestion policy discarded everything a gap outlasted. The one-slot TX engine (buf_tx_slot) replaced that sender: a gap now keeps the newest sample, delivered late, and still loses the older unsent ones (counted in ble_sup). NimBLE also buffers notifications itself, so the slot only engages once notify() fails after about 10 ms of buffer retries.', environment_factor: 'REFUTED as environmental. The 2026-08-23 race day produced both modes on the same device at the same site within the same afternoon (six runs 19.66-19.87 Hz, one run 5.63 Hz), which no site-level RF explanation survives. The earlier 0.86%-vs-75% comparison is additionally unsafe: those baselines predate delivered-truth counters AND come from a period when the GNSS UART harness was intermittently faulty (see fm_gnss_uart_harness_intermittent), so the produced-rate side of the ratio was not sound.', status: 'mechanism measured (anchor starvation); TRIGGER UNDOCUMENTED — endpoint attribution open, environmental attribution refuted; exit is a connection cycle', source: 'measured'})

CREATE (fm_superv:FailureMode {id: 'fm_ble_supervision_400ms', name: 'Peripheral-requested 400 ms supervision timeout', symptom: 'Would convert every sub-second anchor gap into a hard disconnect on any central that honors the request.', root_cause: 'onSubscribe requested conn params with timeout 40 (400 ms). Field data shows the link surviving 3.3 s gaps, so the observed central kept its own multi-second timeout — the hazard is latent, armed against stricter centrals.', status: 'fixed — request is 500 (5 s)', source: 'measured'})

CREATE (fm_gga:FailureMode {id: 'fm_ntrip_gga_blocking_tx', name: 'NTRIP GGA keep-alive as unbounded blocking uplink write', symptom: 'One dropped BLE frame every ~11 s (NCCAR, July 2026): 10 s keep-alive timer + ~1 s blocking TCP send stall, each stall a coex-arbiter BLE anchor denial.', root_cause: 'ntripClient historically had no socket timeout while the scan clients did; print() of the GGA blocked on a congested hotspot uplink and the timestamp advanced only after return, stretching the period to the observed 11 s.', keepalive_period: '10 s on single-base mounts; 1 s on VRS mounts while moving faster than 1 kn', status: 'partly fixed — the timestamp advances after any send attempt so a stalled send retries next cycle, and blocked sends over 20 ms are logged; but setTimeout(250) sets only the Stream read timeout, so the GGA write is still bounded only by the core write loop (about 1 s x 10 retries, arduino-esp32 3.3.8 NetworkClient::write); 2026-08-22 stalls show zero 10-11 s periodicity, consistent with the fix being effective', source: 'measured'})

CREATE (hyp_rto:Hypothesis {id: 'hyp_uplink_rto_bursts', statement: 'The dominant 2026-08-22 anchor-denial driver is lwIP TCP retransmission bursts on the NTRIP uplink (GGA keep-alives / ACKs stuck against a cell-backhauled hotspot): RTO backoff timers fire aperiodically every 0.5-3 s, matching the measured 1-2 s stall recurrence, and no client-side socket timeout bounds radio occupancy from MAC/TCP retries. Explains uniform loss across RF-quiet and RF-dense sections.', status: 'UNCONFIRMED — uplink health is unlogged (only downlink rtcm_bytes exists); needs TX-side instrumentation or a packet capture.', source: 'derived'})

MATCH (a {id: 'fm_ntrip_gga_blocking_tx'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:ARISES_FROM {site: 'GGA keep-alive send path', source: 'code'}]->(b)
MATCH (a {id: 'hyp_uplink_rto_bursts'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:PROPOSED_DRIVER_OF {evidence: 'aperiodic 1-2 s stall recurrence (period folds all below noise floor), RF-environment independence', source: 'measured'}]->(b)

CREATE (hyp_size:Hypothesis {id: 'hyp_sample_size_amplifier', statement: 'Samples grew ~127 B (pre-DBC) to ~277 B (new DBC, 42 values), crossing the 251 B DLE LL-PDU boundary from 1 to 2 fragments per notification — possibly amplifying loss under sparse master grants.', status: 'UNCONFIRMED — CAN was fully populated on every real row of the 08-22 drive, so no within-drive size contrast exists; needs a controlled contrast or the 08-21 export.', code_check: 'No DBC reaches the BLE sample: it is built from the fixed 66-channel table and the sparse format omits NaN channels. About 127 B is a CAN-silent sample and about 277 B a CAN-populated 718 sample, so the size contrast is CAN present versus absent (or a build that added channels), not a DBC. A notification fits one 251 B LL PDU only up to 244 B.', source: 'derived'})

CREATE (inst_params:Instrument {id: 'inst_ble_conn_params', name: 'Granted conn-param evidence at subscribe', signals: 'interval and supervision timeout converted to ms, latency, and the MTU negotiated in onMTUChange', outputs: 'serial line at each CCCD subscribe or unsubscribe', source: 'code', notes: 'The granted interval was never observable in field logs before; the 08-22 value had to be recovered by lattice-fitting arrival times.'})

CREATE (inst_diffage:Instrument {id: 'inst_diff_age_column', name: 'diff_age GPS-log column', signals: 'GGA field-13 correction age (s), -1 = none in use', outputs: 'gps CSV per-row', source: 'code', notes: 'Direct correction-staleness record; previously parsed and displayed but never logged.'})

MATCH (a {id: 'fm_ble_master_anchor_skipping'}), (b {id: 'mod_ble'})  CREATE (a)-[:AMPLIFIED_BY {mechanism: 'Under the abandon-deadline sender, a 25 ms deadline and ~2-sample queue turned master scheduling gaps into permanent loss. The one-slot TX engine replaced it: across a gap of k epochs, k-1 samples are still lost (ble_sup) and the newest goes out late. The old sender blocked up to 100 ms per chunk; the slot engine blocks one notify() per pass, up to about 10 ms.', status: 'historical: sender replaced', source: 'code comments (ble_racecapture.cpp:367-398)'}]->(b)
MATCH (a {id: 'fm_ble_master_anchor_skipping'}), (b {id: 'flow_ble'}) CREATE (a)-[:DEGRADES {measured: '20 Hz produced -> 4.93 Hz delivered (08-22); 0.86% loss (08-21)', source: 'measured'}]->(b)
MATCH (a {id: 'hyp_sample_size_amplifier'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:MAY_AMPLIFY {}]->(b)
MATCH (a {id: 'fm_ble_supervision_400ms'}), (b {id: 'mod_ble'}) CREATE (a)-[:ARISES_FROM {site: 'onSubscribe updateConnParams', source: 'code'}]->(b)
MATCH (a {id: 'inst_ble_conn_params'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:OBSERVES {}]->(b)
MATCH (a {id: 'inst_diff_age_column'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:SURFACED_BY {column: 'diff_age', source: 'code'}]->(b)

// Refuted this session, with the evidence that killed each — so they are not
// re-litigated: (a) connection-interval margin (interval measured GRANTED at
// 15 ms); (b) WiFi scan/reconnect storms on the rover (WiFi/NTRIP up all
// drive, zero stall overlap); (c) rover-side dataMutex holds (producer at
// 20.00 Hz through every stall); (d) weak base link / correction starvation
// (RTCM smooth, one >2 s gap all drive, RTK FIXED at max distance); (e)
// ambient-RF-on-BLE via distance (delivery flat 4.6-5.2 Hz over the full
// 0-1.4 km range at up to 82 mph).

CREATE (fm_ble_drop:FailureMode {id: 'fm_ble_drop', name: 'BLE telemetry drops out (superseded)', symptom: 'Field-reported SoloStorm drops.', root_cause: 'Superseded by fm_ble_master_anchor_skipping; the dataMutex hold-time hypothesis is REFUTED for the measured drives.', status: 'superseded', source: 'measured'})

CREATE (g_datamutex_hold:InstrumentationGap {id: 'gap_datamutex_hold', subject: 'lock_datamutex', missing: 'per-holder hold-time measurement', impact: 'The unbounded (portMAX_DELAY) takes in the NimBLE host callbacks remain unmeasured rule violations — cleared as the drop cause, still a latent connect/disconnect-path hazard.', priority: 'medium'})

MATCH (a {id: 'fm_ble_drop'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:SUPERSEDED_BY {evidence: '08-22 arrival-lattice fit + clump-drain fingerprint', source: 'measured'}]->(b)
MATCH (a {id: 'gap_datamutex_hold'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:GAP_IN {}]->(b)

// ═══════════════════════════════════════════════════════════════════════════
// §15  BLE TX ENGINE — the single-slot sender, and the wire envelope that is
//      part of the client contract (2026-08-25/26)
// ═══════════════════════════════════════════════════════════════════════════
// The blocking chunk-retry sender was replaced by a slot + pump pair so that
// loop() is never parked (a 9-chunk meta used to cost ~940 ms of blockage,
// caught live as 60 stalls of 1000-1099 ms in the trunk-test IMU log). The
// first cut of that engine also removed the WIRE TIMING of the old sender,
// that removal — not the slot, not the buffer, not the library — caused a
// hard handshake spin. §15 records both the engine and the envelope so the
// envelope is never optimised away again.

CREATE (buf_tx_slot:Buffer {id: 'buf_tx_slot', name: 'BLE TX message slot', where: 'PSRAM', fallback: 'internal; BLE TX disabled if both fail', build_buffers: 'meta and sample are first built in static internal buffers of 6,144 B and 4,096 B, so the PSRAM slot saves no internal RAM', bytes: 6144, depth_messages: 1, policy: 'newest-wins: an UNSTARTED sample is replaced by the next sample; a started message is never abandoned except on disconnect', rationale: 'Preserves the deliberate no-stale-backlog intent of the 25 ms abandon deadline structurally (at most one sample of latency can exist) while changing its failure mode from discard to supersede. SoloStorm keys rows to the sample own Utc, so late delivery is nearly free and LOSS is what hurts.', source: 'code'})

CREATE (mod_tx_stage:Module {id: 'mod_tx_stage', name: 'txStage()', file: 'ble_racecapture.cpp', role: 'Admission into the slot. critical=true (meta/version/ACK) takes an idle slot or displaces an unstarted sample; a sample takes an idle slot or replaces an unstarted sample; otherwise the sample is dropped and counted.', source: 'code'})

CREATE (mod_tx_pump:Module {id: 'mod_tx_pump', name: 'txPump()', file: 'ble_racecapture.cpp', role: 'Advances the in-flight message by exactly ONE notification per call, no sooner than 4 ms after the previous one; fails fast on congestion and retries next pass; stamps the meta guard at completion.', source: 'code'})

CREATE (rule_tx_wire_envelope:Rule {id: 'rule_tx_wire_envelope', name: 'The BLE wire envelope is client-facing contract', statement: 'Multi-chunk messages MUST go out as one notification at a time with >=4 ms spacing; a meta must be followed by ~15 ms of quiet; no sample may be wedged between a getMeta and its meta, nor around a meta in flight. These were established by trial and error against SoloStorm in June 2026 and are properties of the CLIENT reassembly, not internal timing preferences.', enforcement: 'txPump serialization clock + metaQuietMs + the tick sample barrier', source: 'code+measured'})

CREATE (rule_meta_guard_completion:Rule {id: 'rule_meta_guard_completion', name: 'The getMeta guard measures receipt, not intent', statement: 'bleLastMetaMs is stamped when a meta FINISHES on the wire, never when it is staged. An unfinished meta leaves the guard open so the next getMeta is answered immediately — the bench-proven improving shape. Stamping at staging lets a meta that never reached the client silence the client retry that would have fixed it.', source: 'code'})

CREATE (fm_ble_v3_wire_envelope_removal:FailureMode {id: 'fm_ble_v3_wire_envelope_removal', name: 'SoloStorm handshake spin after wire-envelope removal', symptom: 'MTU 512 negotiated, one clean subscribe, setTelemetry accepted, t=0 sample transmitted — then an endless loop of {"getMeta":null} answered by a complete meta every ~250 ms, forever. Streaming never starts.', root_cause: 'The engine sent up to 3 chunks back-to-back per pass with no inter-chunk spacing and no post-meta quiet, and could wedge a sample between a getMeta and its meta. Byte content and chunk boundaries were identical to the proven sender; only the envelope changed. Every complete meta was rejected by the client while single-chunk samples were accepted.', elimination_chain: '(1) MTU-23/ACK starvation dead — MTU 512. (2) Subscription flap dead — exactly one TX SUBSCRIBED, no re-subscribe. (3) t=0-sample-loss dead — that sample transmitted, and it carries no embedded meta. (4) Staging failure dead — a failed stage keeps pendingMeta set and would reprint at loop rate (~300/s); observed 4/s equals the 250 ms guard exactly, so every stage succeeded, so the previous meta had fully drained. (5) Pump gate trips dead — conn/subscribe/pTxChar all stable. Therefore complete byte-identical metas were handed to the stack every 250 ms and every one was rejected.', library_acquitted: 'NimBLE 2.x NimBLECharacteristic::notify() deep-copies (auto value{m_value}) and sendValue() builds a fresh mbuf via ble_hs_mbuf_from_flat at call time, so back-to-back setValue+notify cannot tear a queued chunk. Verified against upstream source, not memory.', status: 'convicted by exhaustive construction; fix implemented (v3.2)', source: 'code+serial'})

CREATE (inst_ble_delivered_truth:Instrument {id: 'inst_ble_delivered_truth', name: 'delivered-truth BLE counters', surfaces: 'blePacketHz from txDelivered; gps CSV columns ble / ble_dlv / ble_sup after diff_age', answers: 'How many samples were handed to the NimBLE host and how many were superseded — row-diffs give the send rate and loss for any window, from the log alone.', limitation: 'notify() returns once the host has queued the notification; notifications are unacknowledged, so ble_dlv is not receipt by the central. Samples never built (not streaming, not subscribed, meta pending or in its quiet window) appear in neither column.', source: 'code'})

CREATE (gap_client_reassembly:InstrumentationGap {id: 'gap_client_reassembly', subject: 'SoloStorm message reassembly', missing: 'the client actual framing rule (timeout? notification-boundary? both?)', impact: 'The envelope is known to work empirically but its necessary and sufficient conditions are inferred from June trial-and-error, not read from client source. Any future change to chunking, spacing or message adjacency is therefore an experiment and must be desk-tested against a real SoloStorm handshake before it drives.', priority: 'high'})

MATCH (a {id: 'mod_tx_stage'}), (b {id: 'buf_tx_slot'}) CREATE (a)-[:WRITES_TO {}]->(b)
MATCH (a {id: 'mod_tx_pump'}), (b {id: 'buf_tx_slot'}) CREATE (a)-[:DRAINS {}]->(b)
MATCH (a {id: 'mod_tx_pump'}), (b {id: 'rule_tx_wire_envelope'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_tx_pump'}), (b {id: 'rule_meta_guard_completion'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'fm_ble_v3_wire_envelope_removal'}), (b {id: 'rule_tx_wire_envelope'}) CREATE (a)-[:VIOLATED {}]->(b)
MATCH (a {id: 'fm_ble_v3_wire_envelope_removal'}), (b {id: 'mod_ble'}) CREATE (a)-[:ARISES_FROM {site: 'txPump chunk loop', source: 'code'}]->(b)
MATCH (a {id: 'gap_client_reassembly'}), (b {id: 'rule_tx_wire_envelope'}) CREATE (a)-[:GAP_IN {}]->(b)
MATCH (a {id: 'inst_ble_delivered_truth'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:SURFACED_BY {source: 'code'}]->(b)
MATCH (a {id: 'buf_tx_slot'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:MITIGATES {how: 'saves one sample per gap: the newest survives with latency, the older ones are superseded and lost (counted)', source: 'derived'}]->(b)

// Worked query — before changing anything on the BLE TX path, read the
// contract and the open gap first:
//   MATCH (r:Rule)-[:GAP_IN|ENFORCES]-(x) WHERE r.id STARTS WITH 'rule_tx' RETURN r, x

// ═══════════════════════════════════════════════════════════════════════════
// §16  DIAGNOSING A DEGRADED EPOCH STREAM — instruments, a convicted physical
//      fault, and what each measurement rules in or out (2026-08-26/28)
// ═══════════════════════════════════════════════════════════════════════════
// A rover that logs below 20 Hz has many possible causes and only a few that
// are distinguishable from the logs alone. This section records the
// instruments that exist, what each one proves, the one fault convicted so
// far, and the questions that remain genuinely unanswered. Every claim here is
// from firmware source or from measured field logs; anything inferred but not
// established is labelled as such rather than asserted.

CREATE (inst_wire_counters:Instrument {id: 'inst_wire_counters', name: 'per-sentence wire counters', surfaces: 'serial, every 5 s: GGA/RMC/GSA/GSV/oth/ckfail', where: 'gnss.cpp handleRawNmeaLine', answers: 'What the receiver ACTUALLY delivered, counted before TinyGPS++ sees it. At a 20 Hz fix rate a healthy link reads GGA=100 RMC=100 per 5 s and ckfail=0. Any shortfall is loss upstream of the parser; any ckfail is corruption on the wire.', limitation: 'Serial only — not carried in the CSV, so it cannot be recovered from a log after the fact.', gsa_expectation: 'GSA is configured every fix, one sentence per constellation, so a healthy count is about 400 per 5 s with the four constellations visible from North America and up to 600 with all six; about 30 means the GSA rate write did not stick. The code comment expecting ~30 is stale.', counts_after_ring: 'the counters see what survived the UART driver ring; lines over 199 B or with a lost terminator are dropped before counting and appear as missing, not as ckfail', source: 'code'})

CREATE (inst_spd_age:Instrument {id: 'inst_spd_age', name: 'spd_age_ms column', surfaces: 'gps CSV', answers: 'Age of the velocity fields at the moment the row was built. A few ms means that epoch was completed by its own RMC. A value at or above the watchdog window means GGA arrived and RMC did not, so the row was completed by the watchdog and its speed/heading are carried over. Distinguishes a PARTIALLY lost epoch (row present, RMC missing) from a WHOLLY lost one (no row at all).', field_result: 'On the 2026-08-26 evening drive: p50 = 3 ms with only 5.6% of rows at or above the watchdog, while 31% of epochs produced no row at all — i.e. the surviving rows were healthy and the losses took GGA and RMC together.', source: 'code+measured', blind_to: 'Null SOG and COG: TinyGPS++ commits the previous value again, so the age stays at a few ms while speed and heading repeat. Consecutive-value comparison is the only reliable staleness test in existing logs.'})

CREATE (inst_loop_phase:Instrument {id: 'inst_loop_phase', name: 'loop() phase timing', surfaces: 'serial, every 5 s: max microseconds per phase', where: 'RCX_RTK_Datalogger.ino', answers: 'Which phase of loop() holds the core longest. Exists because the UART ISR that empties the GNSS hardware FIFO runs on this core, so a phase that defers interrupts past the FIFO depth can cost whole sentences.', phases: 'gnss, imu, temp, ble, sdpush, heapFree, heapLargest, loop (whole pass); a phase at or above 2800 us is flagged, the UART FIFO depth at 460800 baud; the heap slots include the DMA-capable walks', field_result: 'First measurement (2026-08-27 bench): gnss 7705-11722 us, imu 761-880, temp 283-458, ble 51-56, sdpush 124-150, heapFree 15-26, heapLargest 176-198. gnss_loop is the dominant cost in loop() and everything else is minor.', source: 'code+measured'})

CREATE (fm_gnss_uart_harness_intermittent:FailureMode {id: 'fm_gnss_uart_harness_intermittent', name: 'Intermittent GNSS UART harness fault', symptom: 'Sessions are bimodal, and the mode is fixed from the first second of a session to the last: either essentially perfect (20.00 Hz, ckfail ~0) or degraded (epoch rate anywhere from 19 Hz down to 0.7 Hz, with NMEA checksum failures and raw non-ASCII bytes in the sat log). Measured session damage rates ranged 0.00% to 74.03%, and damage rate and epoch rate track each other closely.', wire_fingerprint: 'Corrupted bytes are NOT random: 89.7% match the bit pattern 1_xxx010 (bit7 set in 100% of cases, bit1 in ~91%, bit0 in ~3%), identical across every degraded session. Corruption CASCADES — after the first bad byte, ~33% of the rest of that line is corrupt against a 0.66% baseline, and only 41% of lines recover before end-of-line. Probability of a byte being corrupt rises with its offset in the sentence (0.03% in the first five bytes, 1.31% by byte 55). Corrupted lines are LONGER than clean ones, so the receiver manufactures bytes rather than merely dropping them.', elimination: 'Temperature REFUTED (coolest session 147 F was 24.6% damaged; hottest 201 F was 6.7%). WiFi and BLE REFUTED (24.6% damage with radios off; 0.00% with WiFi carrying 758 B/s). RTCM volume REFUTED. NMEA volume REFUTED (GSV byte rate flat while epoch rate collapsed). GSV as a cause REFUTED (its block lands at GPS time .989, outside the loss windows; and a 1-in-20-epoch event cannot exceed ~5% loss). Firmware REFUTED by construction: identical build produced 20.00 Hz and 13.76 Hz on the same day. Replacing the LG290P module alone did NOT fix it.', resolution: 'Replacing the UART wiring harness resolved it: the session immediately after showed 20.00 Hz with zero gaps at or above 80 ms, 0.00% NMEA damage and zero non-ASCII bytes over 379 s.', not_understood: 'The exact electrical mechanism that turns a marginal connection into this specific framing-error pattern is NOT established. Baud-ratio mismatch and RC edge degradation were both simulated and neither reproduces the observed byte distribution. Recorded as an unexplained signature that is nonetheless a reliable FINGERPRINT of this fault.', source: 'measured'})

CREATE (rule_ble_epoch_cadence:Rule {id: 'rule_ble_epoch_cadence', name: 'BLE sample cadence is driven by the GNSS epoch', statement: 'ble_racecapture_tick sends one sample per NEW gps.epochSeq, not on a free-running millisecond gate, with a 100 ms fallback that only fires if the GNSS goes silent. A timer gate re-armed from now() keeps its loop-latency residual instead of correcting it, so it drifts against the receiver hard 50 ms epoch grid and periodically skips an epoch entirely. The fallback interval is deliberately far from the 50 ms grid so it cannot beat against real epochs, and a 10 Hz stream of stale-position samples is itself the fault signature.', enforcement: 'ble_racecapture.cpp tick()', source: 'code'})

CREATE (gap_raw_byte_visibility:InstrumentationGap {id: 'gap_raw_byte_visibility', subject: 'raw NMEA bytes on the GNSS UART', missing: 'Raw bytes are logged ONLY for GSV (sat CSV). GGA, RMC, GSA and PQTM are parsed and discarded, so their corruption cannot be measured after the fact.', impact: 'Every corruption rate quoted from field logs is a GSV rate extrapolated to the other sentence types. GSV is roughly a sixth of sentence traffic, so a per-second test for "was this second clean?" is badly underpowered — an apparently clean second can still carry damaged GGA/RMC. Conclusions that rest on corruption NOT correlating with loss are weak for this reason.', would_fix: 'Carrying the existing ckfail counter into the CSV, ideally broken down per sentence type.', priority: 'high'})

CREATE (gap_oth_sentence_rate:InstrumentationGap {id: 'gap_oth_sentence_rate', subject: 'unidentified non-GGA/RMC/GSA/GSV traffic', missing: 'The wire counter reports oth=300 per 5 s where the code comment expects ~100 (PQTM EPE at 20 Hz). Roughly 40 sentences per second are arriving that have not been identified.', impact: 'Unaccounted bytes and parse time in gnss_loop, which the loop-phase instrument shows is the dominant cost in loop(). Not known to be harmful; not known to be harmless.', status: 'UNEXPLAINED — observed on a healthy bench session, not yet investigated', priority: 'medium'})

CREATE (gap_imu_low_rate:InstrumentationGap {id: 'gap_imu_low_rate', subject: 'QMI8658 sample rate', missing: 'Cause of the IMU logging at 4.03 Hz instead of ~50 Hz, with 345 gaps of ~1000 ms, on the 2026-08-27 session. imu_readTempC() returned NAN for every row of the preceding session while imu_read() returned valid accelerometer data on the same bus.', code_now: 'imu_readTempC() has no caller and status.imuTempC has no writer; the value is captured into the GPS log record but no CSV has a column for it. A failed IMU read still writes a row (0, 0, 1 g), so an I2C timeout would show as flat rows, not as gaps. IMU rows are now decimated to one per 1000 ms after 60 s motionless (see hyp_imu_gaps_decimation).', impact: 'IMU data is the acceleration and yaw channel; 4 Hz is unusable for motorsport analysis.', established: 'GNSS was simultaneously perfect (20.00 Hz, zero gaps) during the same session, which PROVES the IMU path does not starve the GNSS stream. That directional claim was previously asserted and is now refuted.', status: 'OPEN — cause undocumented; the ~1000 ms period is consistent with an I2C timeout but imu.cpp does not call Wire.setTimeOut(), so the effective timeout value is the core default and has not been verified against this core version', priority: 'high'})

CREATE (rb_gnss_triage:Runbook {id: 'rb_gnss_triage', name: 'Triage: rover logging below 20 Hz', module: 'mod_gnss', step_1: 'Read ckfail and the GGA/RMC counts in the 5 s wire line, then the loop-phase line. ckfail>0 or GGA/RMC below 100 per 5 s means the loss is at or before the UART RX ring: the physical link, or a FIFO or ring overrun caused by a loop() stall. If no phase exceeds the FIFO depth and no web GNSS setting was changed, go to fm_gnss_uart_harness_intermittent and suspect the physical link.', step_2: 'If the wire counters are clean but epochs are missing, the loss is downstream of the counters: check spd_age_ms to separate watchdog-completed rows from wholly missing ones.', code_check: 'the counters sit after the UART driver ring; the code comment attributes ckfail to FIFO overrun (gnss.cpp:732-734), and a PPP read-back flushes the ring (gnss.cpp:288)', step_3: 'Check whether the degradation is fixed for the whole session or varies within it. A session-constant mode points at the physical link; a smooth progressive change points at a resource, and a sharp on/off episode with full recovery points at an intermittent connection.', step_4: 'Do NOT read the sat CSV corruption rate as the whole-link corruption rate — see gap_raw_byte_visibility.', anti_patterns: 'Temperature, WiFi/BLE activity, RTCM volume, NMEA volume and satellite count have each been tested against multi-session field data and REFUTED as causes of epoch loss. Do not re-derive them.', source: 'measured'})

CREATE (rb_ble_triage:Runbook {id: 'rb_ble_triage', name: 'Triage: SoloStorm shows missing samples', module: 'mod_ble', step_1: 'Diff ble_dlv and ble_sup across the window. ble_sup climbing means the firmware had a sample and could not place it — the slot was busy. That is the anchor-starvation fingerprint once NimBLE buffers are exhausted, but a handshake (ACK, version or meta displacing a sample) raises it too, so check the ble column for a reconnect first. ble_sup at zero means the firmware placed every sample it built.', step_2: 'Check the ble column for state changes. A run of non-advancing ble_dlv that ENDS in ble going to 0 is a disconnect, not sample loss — the last ~1 s before a drop delivers nothing while the state column still reads subscribed.', step_3: 'If ble_dlv advanced on every subscribed epoch but SoloStorm still shows gaps, the samples were accepted by the NimBLE host and died past it — compare against the SoloStorm export directly.', field_result: 'On the first fully clean session (2026-08-27, 379 s): 4475 subscribed epochs, 4433 delivered, ble_sup=0, and every one of the 43 non-advancing epochs was the run-up to one of two disconnects. On healthy hardware the sender loses essentially nothing.', source: 'measured'})

MATCH (a {id: 'fm_gnss_uart_harness_intermittent'}), (b {id: 'mod_gnss'}) CREATE (a)-[:PRESENTS_IN {note: 'presents as a firmware symptom but originates in hardware', source: 'measured'}]->(b)
MATCH (a {id: 'inst_wire_counters'}), (b {id: 'fm_gnss_uart_harness_intermittent'}) CREATE (a)-[:DETECTS {}]->(b)
MATCH (a {id: 'inst_spd_age'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:SURFACED_BY {}]->(b)
MATCH (a {id: 'inst_loop_phase'}), (b {id: 'mod_gnss'}) CREATE (a)-[:MEASURES {}]->(b)
MATCH (a {id: 'rule_ble_epoch_cadence'}), (b {id: 'mod_ble'}) CREATE (a)-[:ENFORCED_BY {}]->(b)
MATCH (a {id: 'gap_raw_byte_visibility'}), (b {id: 'mod_gnss'}) CREATE (a)-[:GAP_IN {}]->(b)
MATCH (a {id: 'gap_oth_sentence_rate'}), (b {id: 'mod_gnss'}) CREATE (a)-[:GAP_IN {}]->(b)
MATCH (a {id: 'gap_imu_low_rate'}), (b {id: 'mod_imu'}) CREATE (a)-[:GAP_IN {}]->(b)
MATCH (a {id: 'rb_gnss_triage'}), (b {id: 'mod_gnss'}) CREATE (a)-[:TRIAGES {}]->(b)
MATCH (a {id: 'rb_ble_triage'}), (b {id: 'mod_ble'}) CREATE (a)-[:TRIAGES {}]->(b)

// Worked query — before investigating a degraded stream, read the runbook and
// the open gaps for that module rather than re-deriving from logs:
//   MATCH (r:Runbook)-[:TRIAGES]->(m) RETURN r, m
//   MATCH (g:InstrumentationGap)-[:GAP_IN]->(m {id: 'mod_gnss'}) RETURN g


// ─────────────────────────────────────────────────────────────────────────────
// 17. Cone survey mode — an LCD-only view of the RTK solution while the rover
//     sits on a cone (display.cpp)
// ─────────────────────────────────────────────────────────────────────────────
// The mode is a way of LOOKING at data. It runs entirely inside the Display
// task, reads the snapshot that task already takes, and writes to the panel
// and the status LED only. One side effect reaches the logs: while it is on,
// sd_log keeps the IMU at full rate instead of decimating a motionless unit.
// The pose gate follows West_Course_Cone_Detection_Algorithm_Revision.md §1;
// §4 and §7-§9 of that spec stay in post-processing, where the logs remain
// the source of truth.

CREATE (mod_cone:Module {id: 'mod_cone_survey', name: 'cone survey mode', file: 'display.cpp', role: 'detects the cone-scan pose, tracks each dwell on a cone, computes surveyed accuracy from fixed epochs and drives a large colour-coded status block, an RTK-only banner and the status LED', spec: 'West_Course_Cone_Detection_Algorithm_Revision.md §1 pose gate; §4 and §7-§9 stay in post-processing', cadence: 'tracking every Display pass (about 20 Hz), statistics at 5 Hz, drawing at 5 Hz with the panel on', source: 'code', cite: 'display.cpp:289-904'})
CREATE (buf_cone:Buffer {id: 'buf_cone_state', name: 'Cone survey state', bytes: 20088, placement: 'psram', lifetime: 'allocated on entering cone mode, freed on exit', contents: '1200 fixed samples as local E/N/U (14,400 B, 60 s at 20 Hz), 1200-float median scratch (4,800 B), 14 cached field strings (560 B), 32-sample pose ring (162 B), plus dwell, scatter and cache scalars', on_alloc_failure: 'mode still shows, with the survey figures disabled', source: 'code', cite: 'display.cpp:425-462, 665-670'})

MATCH (a {id: 'mod_display'}), (b {id: 'mod_cone_survey'}) CREATE (a)-[:CONTAINS {}]->(b)
MATCH (a {id: 'mod_cone_survey'}), (b {id: 'buf_cone_state'}) CREATE (a)-[:WRITES_TO {}]->(b)
MATCH (a {id: 'buf_cone_state'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 20088, transient: true, source: 'code'}]->(b)
MATCH (a {id: 'mod_cone_survey'}), (b {id: 'flow_snapshot'}) CREATE (a)-[:CONSUMES_FLOW {fields: 'valid, epochSeq, rtkType, pppActive, latitude, longitude, altMSL, speedKnots, hAccM, vAccM, numSV; accel and gyro; bleConnected'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'flow_snapshot'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)

CREATE (rule_cone_view:Rule {id: 'rule_cone_view_only', name: 'Cone survey mode only changes what the LCD shows', statement: 'The mode writes no shared state and changes nothing about GNSS, BLE, NTRIP or CAN. Everything runs inside the Display task, the only task that touches the panel.', exception: 'sd_log reads display_coneModeActive() and keeps the IMU at full rate while the mode is on; the code comment that it changes nothing about SD is stale', source: 'code', cite: 'display.cpp:292-297; sd_log.cpp:411'})
CREATE (rule_cone_car:Rule {id: 'rule_cone_not_in_car', name: 'Cone survey mode never shows in a car about to run', statement: 'The cone banner ignores BLE, so entry is refused while BLE is connected or above 0.5 m/s, and the mode exits at once on a BLE connect or above 3.0 m/s.', source: 'code', cite: 'display.cpp:306-311, 375-376, 663, 674'})
CREATE (rule_cone_rtk:Rule {id: 'rule_cone_correlated_error', name: 'Surveyed accuracy is not divided by the square root of N', statement: 'RTK errors between epochs are correlated, so scatter about the median is reported as-is rather than as a standard error.', source: 'code', cite: 'display.cpp:362-365'})
MATCH (a {id: 'mod_cone_survey'}), (b {id: 'rule_cone_view_only'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_cone_survey'}), (b {id: 'rule_cone_not_in_car'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_cone_survey'}), (b {id: 'rule_cone_correlated_error'}) CREATE (a)-[:ENFORCES {}]->(b)

CREATE (inst_cone_mode:Instrument {id: 'inst_cone_mode_log', name: 'cone mode serial lines', marker: '📍 LCD: cone survey mode ON / OFF (BLE connected | moving | horizontal)', cadence: 'on each mode change', source: 'code', cite: 'display.cpp:665-683'})
CREATE (inst_cone_result:Instrument {id: 'inst_cone_result_log', name: 'cone result serial line', marker: '📍 Cone DONE|REDO: dwell, fixed epochs of total, scatter H and V mm', cadence: 'when each dwell closes', limitation: 'Serial only; cone results are not written to the SD log', source: 'code', cite: 'display.cpp:516-532'})
MATCH (a {id: 'inst_cone_mode_log'}), (b {id: 'mod_cone_survey'}) CREATE (a)-[:OBSERVES {}]->(b)
MATCH (a {id: 'inst_cone_result_log'}), (b {id: 'mod_cone_survey'}) CREATE (a)-[:OBSERVES {}]->(b)

// LCD mode: datalogger layout or cone layout.
CREATE (sm_lcd:StateMachine {id: 'sm_lcd_mode', name: 'LCD mode', owner: 'Display task', timing: 'leaky accumulators: rise by dt while the condition holds, fall at 2 dt otherwise; dt capped at 500 ms', source: 'code', cite: 'display.cpp:373-383, 650-686'})
CREATE (st_lcd_dl:State {id: 'st_lcd_datalogger', name: 'Datalogger', machine: 'sm_lcd_mode', shows: 'normal layout, rotation 0, banner from WiFi, BLE, NTRIP and RTK health', initial: true})
CREATE (st_lcd_cone:State {id: 'st_lcd_cone', name: 'Cone survey', machine: 'sm_lcd_mode', shows: 'rotation 180°, CONE SURVEY banner and LED from RTK only, large reported HAcc, surveyed or scatter accuracy, status block'})
MATCH (a {id: 'st_lcd_datalogger'}), (b {id: 'sm_lcd_mode'}) CREATE (a)-[:IN_MACHINE {}]->(b)
MATCH (a {id: 'st_lcd_cone'}), (b {id: 'sm_lcd_mode'}) CREATE (a)-[:IN_MACHINE {}]->(b)
MATCH (a {id: 'st_lcd_datalogger'}), (b {id: 'st_lcd_cone'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'upright and still for 10 s', guard: 'Y axis up (ay ≥ 0.80, |ax| and |az| < 0.35), gyro < 20 °/s, BLE not connected, speed ≤ 0.5 m/s', action: 'allocate buf_cone_state in PSRAM', source: 'code'}]->(b)
MATCH (a {id: 'st_lcd_cone'}), (b {id: 'st_lcd_datalogger'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'flat for 10 s', guard: '|az| ≥ 0.80 and |ay| < 0.50; no stillness test', action: 'close any open dwell, free the state, restore the normal layout', source: 'code'}]->(b)
MATCH (a {id: 'st_lcd_cone'}), (b {id: 'st_lcd_datalogger'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'BLE connects or > 3 m/s', guard: 'immediate', source: 'code'}]->(b)

// Cone status block: what someone who stepped back from the cone sees.
CREATE (sm_cone:StateMachine {id: 'sm_cone_status', name: 'Cone status block', owner: 'Display task', timing: 'pose ring over the last 1 s needs at least 5 samples, 65 % in pose; DONE needs 3 s of fresh fixed epochs, at least 10 fixed samples and scatter ≤ 20 mm; a gap over 3 s starts a new placement, as does a return more than 1.5 m away once the dwell has 10 fixed samples and a valid fix; a DONE dwell is never extended; dwells under 0.8 s are ignored; scatter is cumulative over the dwell and freezes after 1200 fixed samples (60 s)', display_rule: 'READY and HOLD are drawn only while RTK is FIXED; otherwise FLOAT is drawn, except while resting on a DONE cone', colours: 'chosen for emitted brightness so they read from a distance: READY grey, HOLD black, DONE blinking green, DRIFT and REDO red, FLOAT yellow-black stripes', source: 'code', cite: 'display.cpp:385-419, 535-646, 863-903'})
CREATE (st_c_ready:State {id: 'st_cone_ready', name: 'READY', machine: 'sm_cone_status', shows: 'set on cone, or last cone scatter after a DONE cone is lifted', initial: true})
CREATE (st_c_hold:State {id: 'st_cone_hold', name: 'HOLD', machine: 'sm_cone_status', shows: 'progress bar over the 3 s survey, or settling with the current scatter; waiting for FIX appears only until the first fixed in-pose epoch, because without a fix FLOAT is drawn; HOLD also stays on screen through the 3 s gap after a lift'})
CREATE (st_c_done:State {id: 'st_cone_done', name: 'DONE', machine: 'sm_cone_status', shows: 'blinking green at about 2-2.5 Hz (under the 3 flashes per second limit), scatter and lift cone'})
CREATE (st_c_drift:State {id: 'st_cone_drift', name: 'DRIFT', machine: 'sm_cone_status', shows: 'red; scatter above 50 mm after at least 1 s fixed'})
CREATE (st_c_float:State {id: 'st_cone_float', name: 'FLOAT', machine: 'sm_cone_status', shows: 'yellow and black hazard stripes: NO FIX, PPP, FLOAT or no RTK'})
CREATE (st_c_redo:State {id: 'st_cone_redo', name: 'REDO', machine: 'sm_cone_status', shows: 'red, lifted early'})
MATCH (s:State), (m {id: 'sm_cone_status'}) WHERE s.machine = 'sm_cone_status' CREATE (s)-[:IN_MACHINE {}]->(m)
MATCH (a {id: 'st_cone_ready'}), (b {id: 'st_cone_hold'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'set on a cone', guard: 'rolling pose: at least 5 samples and 65 % of the last 1 s in pose', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_hold'}), (b {id: 'st_cone_done'}) CREATE (a)-[:TRANSITIONS_TO {trigger: '3 s fixed, scatter ≤ 20 mm', guard: 'at least 10 fixed samples; fixed epochs count only while ≤ 500 ms old; evaluated at 5 Hz', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_hold'}), (b {id: 'st_cone_drift'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'scatter > 50 mm', guard: 'after at least 1 s fixed', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_drift'}), (b {id: 'st_cone_hold'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'scatter back ≤ 50 mm', guard: 'only by dilution with good samples; impossible once 1200 fixed samples are held', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_hold'}), (b {id: 'st_cone_float'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'RTK fix lost', guard: 'not resting on a DONE cone', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_float'}), (b {id: 'st_cone_hold'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'RTK fixed again', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_hold'}), (b {id: 'st_cone_redo'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'out of pose over 3 s before DONE', guard: 'time in pose at least 0.8 s; HOLD stays on screen during the 3 s gap, and a re-set more than 1.5 m away inside the gap opens a new dwell so REDO is never shown', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_done'}), (b {id: 'st_cone_ready'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'lifted', action: 'keep the scatter as last cone', source: 'code'}]->(b)
MATCH (a {id: 'st_cone_redo'}), (b {id: 'st_cone_hold'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'set on a cone again', source: 'code'}]->(b)
MATCH (a {id: 'sm_lcd_mode'}), (b {id: 'mod_cone_survey'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sm_cone_status'}), (b {id: 'mod_cone_survey'}) CREATE (a)-[:OWNED_BY {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 18. Structure — where code runs, what it calls, what it is built on
// ─────────────────────────────────────────────────────────────────────────────
// CALLS edges are aggregated from caller -> callee function pairs across
// modules; `functions` lists the callee functions. RUNS_IN says which task
// executes a module's code; a module can run in more than one.

CREATE (m_types:Module {id: 'mod_types', file: 'types.h', role: 'shared data structs (GnssData, ImuData, CanData, SystemStatus), the global instances and dataMutex, and the English-unit conversion helpers', source: 'code'})
CREATE (m_p987:Module {id: 'mod_can_porsche987', file: 'can_porsche987.h', role: 'header-only 987.2 PT-CAN frame decoder', can_ids: '0x0C2, 0x14A, 0x242, 0x245, 0x246, 0x102, 0x440, 0x24A, 0x441, 0x44B', source: 'code'})
CREATE (m_p718:Module {id: 'mod_can_porsche718', file: 'can_porsche718.h', role: 'header-only 718 DRIVE-CAN and MMI-CAN (TPMS) frame decoder', can_ids: '0x081, 0x086, 0x100-0x107, 0x30B, 0x30E, 0x640, 0x6B2, 0x6B7, 0x673', source: 'code'})
CREATE (m_p718x:Module {id: 'mod_can_porsche718_extra', file: 'can_porsche718_extra.h', role: '718-only channel struct kept apart from CanData, read under dataMutex through can_getPorsche718Extra()', source: 'code'})

MATCH (a {id: 'mod_ble'}), (b {id: 'mod_tx_stage'}) CREATE (a)-[:CONTAINS {}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'mod_tx_pump'}) CREATE (a)-[:CONTAINS {}]->(b)

MATCH (a {id: 'mod_main'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'setup() and loop()'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'task_display'}) CREATE (a)-[:RUNS_IN {what: 'displayTask body: snapshot under dataMutex 10 ms, then display_update()'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'task_wifintrip'}) CREATE (a)-[:RUNS_IN {what: 'wifiNtripTask body: WiFi edges, NTRIP and RaceCapture at 10 ms'}]->(b)
MATCH (a {id: 'mod_gnss'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'gnss_loop(): NMEA drain, 512 B per pass'}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'imu_read() at 50 Hz, calibration requests'}]->(b)
MATCH (a {id: 'mod_thermal'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'thermal_update() at 0.5 Hz'}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'ble_racecapture_tick(): staging, sample cadence, TX pump'}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'task_nimble'}) CREATE (a)-[:RUNS_IN {what: 'connect, disconnect, subscribe and write callbacks: flags and the ACK queue, plus the status write, advertising restart and connection-parameter request'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'task_sdlog'}) CREATE (a)-[:RUNS_IN {what: 'queue drain, file lifecycle, dir scan, tar build'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'row producers sdlog_push_if_new() and sdlog_push_imu()'}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'task_wifintrip'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'task_wifintrip'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'task_wifintrip'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'task_display'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_cone_survey'}), (b {id: 'task_display'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'task_can'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'task_dbc'}) CREATE (a)-[:RUNS_IN {}]->(b)
MATCH (a {id: 'mod_dbc_parse'}), (b {id: 'task_dbc'}) CREATE (a)-[:RUNS_IN {}]->(b)

MATCH (a {id: 'mod_main'}), (b {id: 'mod_wifi'}) CREATE (a)-[:CALLS {functions: 'wifi_init, wifi_apBegin, wifi_tryConnect, wifi_apService, wifi_service'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_display'}) CREATE (a)-[:CALLS {functions: 'display_init, display_update'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_imu'}) CREATE (a)-[:CALLS {functions: 'imu_init, imu_read, imu_pollSerial, imu_serviceCalRequests'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_gnss'}) CREATE (a)-[:CALLS {functions: 'gnss_init, gnss_loop'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_ble'}) CREATE (a)-[:CALLS {functions: 'ble_racecapture_init, ble_racecapture_tick'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_can'}) CREATE (a)-[:CALLS {functions: 'can_init, canBusTask (task entry)'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_init, sdlog_push_imu, sdlog_push_if_new'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_dbc_store'}) CREATE (a)-[:CALLS {functions: 'dbc_init'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_web'}) CREATE (a)-[:CALLS {functions: 'webserver_init, webserver_begin'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:CALLS {functions: 'ntrip_init, ntrip_loop, ntrip_onWifiLost'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_racecapture'}) CREATE (a)-[:CALLS {functions: 'racecapture_beginServer, racecapture_loop'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_thermal'}) CREATE (a)-[:CALLS {functions: 'thermal_update'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'mod_debug_log'}) CREATE (a)-[:CALLS {functions: 'DebugLog::begin', condition: 'compiled only with DEBUG_SERIAL_TO_SD'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_getActiveGpsSize'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'mod_wifi'}) CREATE (a)-[:CALLS {functions: 'wifi_deviceName, wifi_apIp'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'mod_thermal'}) CREATE (a)-[:CALLS {functions: 'thermal_lcdOff, thermal_backlightDim'}]->(b)
MATCH (a {id: 'mod_gnss'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_push_sat'}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'mod_gnss'}) CREATE (a)-[:CALLS {functions: 'gnss_buildGGA'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'mod_ble'}) CREATE (a)-[:CALLS {functions: 'ble_linkState, ble_txDelivered, ble_txSuperseded'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'mod_can'}) CREATE (a)-[:CALLS {functions: 'can_getPorsche718Extra, can_getSniffer'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'mod_thermal'}) CREATE (a)-[:CALLS {functions: 'thermal_satCanImuInhibit, thermal_gpsReduce1Hz, thermal_canSniffInhibit'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'mod_display'}) CREATE (a)-[:CALLS {functions: 'display_coneModeActive'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_gnss'}) CREATE (a)-[:CALLS {functions: 'gnss_getEleMask, gnss_getCnrMask, gnss_getPppMode, gnss_requestMasks, gnss_requestPppMode'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_can'}) CREATE (a)-[:CALLS {functions: 'can_getSniffer, can_setSniffer, can_getSniffSnapshot, can_getSniffOverflow'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_wifi'}) CREATE (a)-[:CALLS {functions: 'wifi_deviceName, wifi_apIp, wifi_count, wifi_getSsid, wifi_getEnabled, wifi_setDeviceName, wifi_add, wifi_remove, wifi_setEnabled, wifi_move'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:CALLS {functions: 'ntrip_casterCount, ntrip_casterInfo, ntrip_setCasterEnabled, ntrip_saveCaster, ntrip_requestReset, ntrip_removeCaster'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_display'}) CREATE (a)-[:CALLS {functions: 'display_setEnabled, display_isEnabled'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_imu'}) CREATE (a)-[:CALLS {functions: 'imu_requestCalibration, imu_requestClearCalibration, imu_calibrationState, imu_isCalibrated'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_dbc_store'}) CREATE (a)-[:CALLS {functions: 'dbc_getSnapshot, dbc_getActive, dbc_scanPending, dbc_lastStatus, dbc_getAudited, dbc_uploadBegin, dbc_uploadChunk, dbc_uploadEnd, dbc_setActive, dbc_requestDelete'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_dbc_parse'}) CREATE (a)-[:CALLS {functions: 'dbc_auditText'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_getMutex, sdlog_isReady, sdlog_getActiveFiles, sdlog_getDirSnapshotPage, sdlog_isActiveFile, sdlog_noteFileDeleted, sdlog_getCapacity, sdlog_requestExport, sdlog_getExportState, sdlog_downloadBegin, sdlog_downloadEnd, sdlog_setConfig, sdlog_setPaused, sdlog_getDirSnapshot, sdlog_dirScanPending, sdlog_getFileCount, sdlog_getActiveName, sdlog_getLogGps/Imu/Can/Sat, sdlog_isPaused, sdlog_getExportPath, sdlog_exportIsSubset, sdlog_getCanRawDrops'}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'mod_debug_log'}) CREATE (a)-[:CALLS {functions: 'DebugLog::setEnabled, isEnabled, isForced, currentFileName', condition: 'compiled only with DEBUG_SERIAL_TO_SD; currentFileName is not declared in webserver.cpp'}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'mod_dbc_parse'}) CREATE (a)-[:CALLS {functions: 'dbc_parseAndAudit, dbc_auditSummary'}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_isReady, sdlog_getMutex'}]->(b)
MATCH (a {id: 'mod_debug_log'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_takeMutex, sdlog_giveMutex, sdlog_sessionStamp', status: 'declared but never defined: unresolved if DEBUG_SERIAL_TO_SD is enabled'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:CALLS {functions: 'sdlog_push_can_raw (non-blocking)'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'mod_can_porsche987'}) CREATE (a)-[:CALLS {functions: 'decodePorsche9872CanFrame'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'mod_can_porsche718'}) CREATE (a)-[:CALLS {functions: 'decodePorsche718CanFrame'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'mod_can_porsche718_extra'}) CREATE (a)-[:CALLS {functions: 'writes g718Extra'}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'mod_can'}) CREATE (a)-[:CALLS {functions: 'can_getPorsche718Extra'}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'mod_can'}) CREATE (a)-[:CALLS {functions: 'can_getPorsche718Extra'}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'mod_types'}) CREATE (a)-[:CALLS {functions: 'kphToMph, cToF, barToPsi, kpaToPsi'}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'mod_types'}) CREATE (a)-[:CALLS {functions: 'kphToMph, cToF, barToPsi, kpaToPsi'}]->(b)

CREATE (lib_core:Library {id: 'lib_esp32_core', name: 'Arduino-ESP32 core', version: '3.3.8', provides: 'FreeRTOS, heap_caps, LEDC, rgbLedWriteOrdered, temperature_sensor, HardwareSerial', note: 'core 3.x APIs and memory layout are required', source: 'code'})
CREATE (lib_wifi:Library {id: 'lib_wifi', name: 'WiFi / lwIP', provides: 'WiFi, WiFiClient, WiFiServer', part_of: 'Arduino-ESP32 core', source: 'code'})
CREATE (lib_sd:Library {id: 'lib_sd_mmc', name: 'SD_MMC / FatFs', part_of: 'Arduino-ESP32 core', source: 'code'})
CREATE (lib_nvs:Library {id: 'lib_preferences', name: 'Preferences (NVS)', part_of: 'Arduino-ESP32 core', source: 'code'})
CREATE (lib_wire:Library {id: 'lib_wire', name: 'Wire (I2C)', part_of: 'Arduino-ESP32 core', source: 'code'})
CREATE (lib_twai:Library {id: 'lib_twai', name: 'TWAI driver (driver/twai.h)', part_of: 'ESP-IDF', source: 'code'})
CREATE (lib_tft:Library {id: 'lib_tft_espi', name: 'TFT_eSPI', author: 'Bodmer', version: '2.5.43', config: 'User_Setup.h: ST7789 172x320; LOAD_FONT4 required or display.cpp fails to compile', source: 'owner setup (version and driver are not in the source tree); LOAD_FONT4 from code'})
CREATE (lib_nimble:Library {id: 'lib_nimble', name: 'NimBLE-Arduino', author: 'h2zero', version: '2.x; assumed the latest release, 2.5.1', version_basis: 'installed version not recorded; owner says to assume the latest unless it is very new', creates: 'NimBLE host task', source: 'code'})
CREATE (lib_tgps:Library {id: 'lib_tinygpsplus', name: 'TinyGPSPlus', author: 'Mikal Hart', source: 'code'})
CREATE (lib_aws:Library {id: 'lib_async_webserver', name: 'ESP Async WebServer', author: 'ESP32Async fork', note: 'the original me-no-dev versions do not compile against core 3.x', source: 'code'})
CREATE (lib_atcp:Library {id: 'lib_asynctcp', name: 'AsyncTCP', author: 'ESP32Async fork', creates: 'async_tcp task', source: 'code'})
MATCH (a {id: 'lib_async_webserver'}), (b {id: 'lib_asynctcp'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'lib_asynctcp'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'lib_nimble'}), (b {id: 'task_nimble'}) CREATE (a)-[:CREATES_TASK {}]->(b)
MATCH (a {id: 'lib_wifi'}), (b {id: 'task_wifi_lwip'}) CREATE (a)-[:CREATES_TASK {}]->(b)

MATCH (a {id: 'mod_main'}), (b {id: 'lib_esp32_core'}) CREATE (a)-[:USES_LIBRARY {headers: 'esp_heap_caps.h, esp_system.h, driver/temperature_sensor.h'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'lib_wifi'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'lib_tft_espi'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'lib_esp32_core'}) CREATE (a)-[:USES_LIBRARY {headers: 'LEDC backlight, rgbLedWriteOrdered, esp_heap_caps.h'}]->(b)
MATCH (a {id: 'mod_gnss'}), (b {id: 'lib_tinygpsplus'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_gnss'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'lib_wifi'}) CREATE (a)-[:USES_LIBRARY {headers: 'WiFi.h, WiFiClient.h'}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'lib_wifi'}) CREATE (a)-[:USES_LIBRARY {headers: 'WiFi.h, esp_wifi.h, esp_mac.h'}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'lib_wire'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'lib_sd_mmc'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'lib_async_webserver'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'lib_sd_mmc'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'lib_sd_mmc'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_dbc_parse'}), (b {id: 'lib_sd_mmc'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_debug_log'}), (b {id: 'lib_sd_mmc'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'lib_nimble'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'lib_wifi'}) CREATE (a)-[:USES_LIBRARY {headers: 'WiFiServer'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'lib_twai'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'lib_preferences'}) CREATE (a)-[:USES_LIBRARY {}]->(b)
MATCH (a {id: 'mod_types'}), (b {id: 'lib_esp32_core'}) CREATE (a)-[:USES_LIBRARY {headers: 'freertos/semphr.h'}]->(b)


// Code that runs outside a module's main task: setup, cross-task getters and web handlers.
MATCH (a {id: 'mod_display'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'display_init() in setup; display_coneModeActive() through sdlog_push_imu'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'display_setEnabled / display_isEnabled from /lcd'}]->(b)
MATCH (a {id: 'mod_thermal'}), (b {id: 'task_display'}) CREATE (a)-[:RUNS_IN {what: 'thermal_lcdOff, thermal_backlightDim'}]->(b)
MATCH (a {id: 'mod_thermal'}), (b {id: 'task_sdlog'}) CREATE (a)-[:RUNS_IN {what: 'SD throttling gates'}]->(b)
MATCH (a {id: 'mod_gnss'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'gnss_request* and getters from /gnss_config (flags only)'}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'caster table edits with NVS commits, reset flag, caster info'}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'network list and device name edits with NVS commits'}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'wifi_init() in setup'}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'task_display'}) CREATE (a)-[:RUNS_IN {what: 'wifi_deviceName, wifi_apIp'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'channel config with NVS commits, export request parsing, noteFileDeleted'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'task_can'}) CREATE (a)-[:RUNS_IN {what: 'sdlog_push_can_raw'}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'upload staging, dbc_setActive with an NVS commit'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'task_loop'}) CREATE (a)-[:RUNS_IN {what: 'can_init in setup; can_getPorsche718Extra for BLE and SD rows'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'task_wifintrip'}) CREATE (a)-[:RUNS_IN {what: 'can_getPorsche718Extra for RaceCapture'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'sniffer control and snapshot'}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'task_async_tcp'}) CREATE (a)-[:RUNS_IN {what: 'calibration request flags and state from /imu/cal'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'mod_types'}) CREATE (a)-[:CALLS {functions: 'cToF'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'mod_types'}) CREATE (a)-[:CALLS {functions: 'kphToMph, cToF, barToPsi, kpaToPsi'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 19. Context — the systems outside the firmware and the interfaces to them
// ─────────────────────────────────────────────────────────────────────────────
// External systems also produce or consume the data flows of §8, so the
// data-flow picture runs from source to sink.

CREATE (x_gnss:ExternalSystem {id: 'ext_lg290p', name: 'Quectel LG290P GNSS', kind: 'device', note: 'firmware v2.02 or newer', source: 'code'})
CREATE (x_imu:ExternalSystem {id: 'ext_qmi8658', name: 'QMI8658 IMU', kind: 'device', settings: '±8 g and ±512 °/s at 1024 Hz; gz follows the right-hand rule, opposite to compass heading', source: 'code'})
CREATE (x_lcd:ExternalSystem {id: 'ext_lcd', name: 'ST7789 LCD 172x320', kind: 'device', source: 'owner setup file; 172x320 from code'})
CREATE (x_led:ExternalSystem {id: 'ext_rgb_led', name: 'WS2812 status LED', kind: 'device', note: 'RGB byte order; the core default GRB would show red as green', source: 'code'})
CREATE (x_sd:ExternalSystem {id: 'ext_sd_card', name: 'microSD card', kind: 'device', source: 'code'})
CREATE (x_sn65:ExternalSystem {id: 'ext_sn65hvd230', name: 'SN65HVD230 CAN transceiver', kind: 'device', rule: 'CAN TX is driven high before anything else at boot so the transceiver stays recessive', txd_pullup: 'built into the module, covering power-on until setup() runs', termination: 'on-board 120 Ω removed for the mid-bus tap', source: 'code; pull-up and termination from the owner'})
CREATE (x_car:ExternalSystem {id: 'ext_vehicle_can', name: 'Vehicle CAN bus', kind: 'vehicle', variants: 'Porsche 987.2 PT-CAN; 718 DRIVE-CAN and MMI-CAN (TPMS)', source: 'code'})
CREATE (x_casters:ExternalSystem {id: 'ext_ntrip_casters', name: 'NTRIP casters', kind: 'network service', defaults: 'RCX1 base AP 192.168.4.1:2101 (Local_Wifi), crtk.net Centipede mount RCX1, rtk2go.com', source: 'code'})
CREATE (x_base:ExternalSystem {id: 'ext_rcx1_base', name: 'RCX1 base station', kind: 'companion device', note: 'reaches the rover through its own AP caster or through Centipede', source: 'code'})
CREATE (x_ss:ExternalSystem {id: 'ext_solostorm', name: 'SoloStorm', kind: 'app', note: 'BLE central as a RaceCapture BLE logger; Android GATT caching constrains any GATT change', source: 'code'})
CREATE (x_rcapp:ExternalSystem {id: 'ext_racecapture_app', name: 'RaceCapture app', kind: 'app', source: 'code'})
CREATE (x_browser:ExternalSystem {id: 'ext_browser', name: 'Web browser', kind: 'user agent', pages: 'dashboard, setup, logs', source: 'code'})
CREATE (x_ap:ExternalSystem {id: 'ext_wifi_network', name: 'WiFi network or phone hotspot', kind: 'network', source: 'code'})
CREATE (x_usb:ExternalSystem {id: 'ext_usb_host', name: 'USB serial console', kind: 'host', commands: 'imucal, imucalclear', source: 'code'})
CREATE (x_surveyor:ExternalSystem {id: 'ext_surveyor', name: 'Cone surveyor', kind: 'person', note: 'reads the LCD colour from a step back', source: 'code'})
CREATE (x_post:ExternalSystem {id: 'ext_postprocessing', name: 'Post-processing tools', kind: 'offline software', note: 'cone detection §4 and §7-§9, run analysis', source: 'code'})
MATCH (a {id: 'ext_rcx1_base'}), (b {id: 'ext_ntrip_casters'}) CREATE (a)-[:FEEDS {}]->(b)
MATCH (a {id: 'ext_vehicle_can'}), (b {id: 'ext_sn65hvd230'}) CREATE (a)-[:FEEDS {}]->(b)
MATCH (a {id: 'ext_sd_card'}), (b {id: 'ext_postprocessing'}) CREATE (a)-[:FEEDS {}]->(b)
MATCH (a {id: 'ext_lcd'}), (b {id: 'ext_surveyor'}) CREATE (a)-[:FEEDS {}]->(b)
MATCH (a {id: 'ext_rgb_led'}), (b {id: 'ext_surveyor'}) CREATE (a)-[:FEEDS {}]->(b)

CREATE (if_uart1:Interface {id: 'if_uart1_gnss', name: 'UART1 to LG290P', kind: 'UART', detail: '460800 8N1, RX GPIO4, TX GPIO5, RX ring 4096 B', writers: 'loop (PQTM commands) and WiFiNTRIP (RTCM) share the TX side', source: 'code'})
CREATE (if_i2c:Interface {id: 'if_i2c_imu', name: 'I2C to QMI8658', kind: 'I2C', detail: 'SDA GPIO48, SCL GPIO47, 400 kHz, address 0x6B, 12-byte burst from 0x35', source: 'code'})
CREATE (if_spi:Interface {id: 'if_spi_lcd', name: 'SPI to LCD', kind: 'SPI', detail: 'TFT_eSPI HSPI, MOSI 45, SCLK 40, CS 42, DC 41, RST 39; backlight GPIO46 LEDC 5 kHz', source: 'owner setup file TFT_UserSetup.h (not in the source tree); backlight from code'})
CREATE (if_led:Interface {id: 'if_led_gpio38', name: 'GPIO38 LED data', kind: 'GPIO', detail: 'RMT-driven WS2812, brightness 64 of 255, written on change or every 5 s', source: 'code'})
CREATE (if_sdmmc:Interface {id: 'if_sdmmc', name: 'SDMMC 1-bit', kind: 'SDMMC', detail: 'CLK 14, CMD 15, D0 16, D3 21, mount /sdcard, 10 open files', source: 'code'})
CREATE (if_twai:Interface {id: 'if_twai', name: 'TWAI RX', kind: 'CAN', detail: '500 kbit/s listen-only, RX GPIO8; TX GPIO9 is driven high (recessive) from the first line of setup() and never dominant; RX queue 64, standard IDs only', source: 'code'})
CREATE (if_ble:Interface {id: 'if_ble_nus', name: 'BLE Nordic UART service', kind: 'BLE GATT', detail: 'NUS 6E400001-B5A3-F393-E0A9-E50E24DCCA9E; TX 6E400003 notify, RX 6E400002 write; also 0x1800 and 0x180A; MTU 512; connection request 15-30 ms, latency 0, timeout 5 s', messages: 'ver, meta, s (samples), command ACKs; commands getMeta, getVer, setTelemetry, startStreaming, stopStreaming', device_name: 'RCX Datalogger (do not rename)', source: 'code'})
CREATE (if_rc:Interface {id: 'if_tcp_7223', name: 'RaceCapture TCP 7223', kind: 'TCP server', detail: 'one client; getCapabilities, getChannels, startStreaming, stopStreaming; full channel array at 20 Hz with null for NaN', source: 'code'})
CREATE (if_http:Interface {id: 'if_http_80', name: 'HTTP on port 80', kind: 'HTTP server', detail: '39 routes on AsyncWebServer, grouped into Endpoint nodes', source: 'code'})
CREATE (if_ntrip:Interface {id: 'if_ntrip_client', name: 'NTRIP client', kind: 'TCP client', detail: 'port 2101, HTTP/1.0 GET with Basic auth and Ntrip-GGA header; ICY 200 or HTTP 200; SOURCETABLE means the mount is not live', source: 'code'})
CREATE (if_wifi:Interface {id: 'if_wifi', name: 'WiFi AP and station', kind: 'WiFi', detail: 'AP named after the device at 192.168.5.1/24, channel 1, 4 clients, open by default; on the first station association it is reconfigured to an empty-SSID open AP that stays enabled and keeps beaconing; station list from NVS', source: 'code'})
CREATE (if_serial:Interface {id: 'if_usb_serial', name: 'USB CDC serial', kind: 'serial', detail: '115200, TX timeout 0 so prints never block loop()', source: 'code'})

MATCH (a {id: 'ext_lg290p'}), (b {id: 'if_uart1_gnss'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_qmi8658'}), (b {id: 'if_i2c_imu'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_lcd'}), (b {id: 'if_spi_lcd'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_rgb_led'}), (b {id: 'if_led_gpio38'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_sd_card'}), (b {id: 'if_sdmmc'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_sn65hvd230'}), (b {id: 'if_twai'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_solostorm'}), (b {id: 'if_ble_nus'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_racecapture_app'}), (b {id: 'if_tcp_7223'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_browser'}), (b {id: 'if_http_80'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_ntrip_casters'}), (b {id: 'if_ntrip_client'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_wifi_network'}), (b {id: 'if_wifi'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)
MATCH (a {id: 'ext_browser'}), (b {id: 'if_wifi'}) CREATE (a)-[:CONNECTS_VIA {via: 'the device AP until a station link exists'}]->(b)
MATCH (a {id: 'ext_usb_host'}), (b {id: 'if_usb_serial'}) CREATE (a)-[:CONNECTS_VIA {}]->(b)

MATCH (a {id: 'mod_gnss'}), (b {id: 'if_uart1_gnss'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'if_i2c_imu'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'if_spi_lcd'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'if_led_gpio38'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'if_sdmmc'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'if_twai'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'if_ble_nus'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'if_tcp_7223'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_web'}), (b {id: 'if_http_80'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'if_ntrip_client'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'if_wifi'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'if_usb_serial'}) CREATE (a)-[:PROVIDES {}]->(b)
MATCH (a {id: 'if_uart1_gnss'}), (b {id: 'uart1'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'if_twai'}), (b {id: 'twai'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'if_sdmmc'}), (b {id: 'sd_bus'}) CREATE (a)-[:TRAVERSES {}]->(b)
MATCH (a {id: 'if_usb_serial'}), (b {id: 'uart0'}) CREATE (a)-[:TRAVERSES {}]->(b)

// Flows completed to their external ends, and the flows §8 did not name.
MATCH (a {id: 'ext_lg290p'}), (b {id: 'flow_nmea'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_lg290p'}), (b {id: 'flow_rtcm'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'ext_ntrip_casters'}), (b {id: 'flow_ntrip'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_sn65hvd230'}), (b {id: 'flow_can'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_solostorm'}), (b {id: 'flow_ble'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'ext_browser'}), (b {id: 'flow_http'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'flow_sd_rows'}) CREATE (a)-[:PRODUCES_FLOW {part: 'GPS+CAN rows per epoch and IMU rows'}]->(b)
MATCH (a {id: 'mod_gnss'}), (b {id: 'flow_sd_rows'}) CREATE (a)-[:PRODUCES_FLOW {part: 'raw GSV lines'}]->(b)
MATCH (a {id: 'mod_can'}), (b {id: 'flow_sd_rows'}) CREATE (a)-[:PRODUCES_FLOW {part: 'raw sniffer frames'}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'flow_snapshot'}) CREATE (a)-[:CONSUMES_FLOW {via: 'WiFiNTRIP task snapshot, 5 ms'}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'flow_snapshot'}) CREATE (a)-[:CONSUMES_FLOW {via: 'WiFiNTRIP task snapshot, 5 ms'}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'flow_snapshot'}) CREATE (a)-[:CONSUMES_FLOW {via: 'loop producers, 2 ms and 5 ms takes'}]->(b)

CREATE (f_imu:DataFlow {id: 'flow_imu', name: 'IMU samples', transport: 'i2c', direction: 'QMI8658 -> ESP32', rate_hz: 50, source: 'code'})
CREATE (f_gga:DataFlow {id: 'flow_gga_uplink', name: 'GGA keep-alive', transport: 'tcp', direction: 'ESP32 -> caster', rate: 'every 10 s; every 1 s on VRS mounts while moving', source: 'code'})
CREATE (f_rc:DataFlow {id: 'flow_rc_tcp', name: 'RaceCapture TCP stream', transport: 'tcp_7223', direction: 'ESP32 -> RaceCapture app', rate: 'nominal 20 Hz; the 50 ms gate re-arms from the time of sending in a 10 ms task loop, so intervals run about 50-60 ms', cadence: 'free-running timer, not the GNSS epoch', source: 'code'})
CREATE (f_files:DataFlow {id: 'flow_sd_files', name: 'CSV log files', transport: 'sdmmc', direction: 'SDLog -> microSD', files: 'gps, can, imu, sat, canraw CSVs; export.tar', source: 'code'})
CREATE (f_lcd:DataFlow {id: 'flow_lcd', name: 'LCD frames and LED colour', transport: 'spi and gpio38', direction: 'Display task -> panel and LED', rate_hz: 5, source: 'code'})
MATCH (a {id: 'ext_qmi8658'}), (b {id: 'flow_imu'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'flow_imu'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_ntrip'}), (b {id: 'flow_gga_uplink'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_ntrip_casters'}), (b {id: 'flow_gga_uplink'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_racecapture'}), (b {id: 'flow_rc_tcp'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_racecapture_app'}), (b {id: 'flow_rc_tcp'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_sdlog'}), (b {id: 'flow_sd_files'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_sd_card'}), (b {id: 'flow_sd_files'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'flow_lcd'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'ext_lcd'}), (b {id: 'flow_lcd'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)

// HTTP endpoints. Every handler runs on the async_tcp task.
CREATE (:Endpoint {id: 'ep_get_root', method: 'GET', path: '/', purpose: 'dashboard page from flash, vetoed chunks', page: 'dashboard', calls_module: 'mod_web', source: 'code', cite: 'webserver.cpp:1476'})
CREATE (:Endpoint {id: 'ep_get_setup', method: 'GET', path: '/setup', purpose: 'setup page', page: 'setup', calls_module: 'mod_web', source: 'code', cite: 'webserver.cpp:1480'})
CREATE (:Endpoint {id: 'ep_get_logs', method: 'GET', path: '/logs', purpose: 'log files page', page: 'logs', calls_module: 'mod_web', source: 'code', cite: 'webserver.cpp:1484'})
CREATE (:Endpoint {id: 'ep_get_assets', method: 'GET', path: '/app.css, /app.js, /favicon.ico', purpose: 'shared assets with ETag and 304; favicon 204', page: 'all', calls_module: 'mod_web', source: 'code', cite: 'webserver.cpp:1498-1517'})
CREATE (:Endpoint {id: 'ep_get_status', method: 'GET', path: '/status', purpose: 'live JSON: WiFi, NTRIP, RTK, accuracy, BLE, CAN, rates, temperature, heap lows', page: 'dashboard', calls_module: 'mod_main', locks: 'dataMutex 50 ms', source: 'code', cite: 'webserver.cpp:1520'})
CREATE (:Endpoint {id: 'ep_get_sd', method: 'GET', path: '/sd', purpose: 'card capacity and active file from cache, no SD I/O', page: 'logs', calls_module: 'mod_sdlog', source: 'code', cite: 'webserver.cpp:1576'})
CREATE (:Endpoint {id: 'ep_get_files', method: 'GET', path: '/files', purpose: 'active, recent or paged file list from the RAM snapshot', page: 'logs', calls_module: 'mod_sdlog', locks: 'dirSnapLock', source: 'code', cite: 'webserver.cpp:1608'})
CREATE (:Endpoint {id: 'ep_get_log', method: 'GET', path: '/log/<file>', purpose: 'stream a CSV or debug file; 409 unless logging is paused; one download at a time', page: 'logs', calls_module: 'mod_sdlog', sd_io_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:1734'})
CREATE (:Endpoint {id: 'ep_post_log_delete', method: 'POST', path: '/log/delete', purpose: 'batch delete up to 25 files', page: 'logs', calls_module: 'mod_sdlog', locks: 'sdMutex 2000 ms, then dirSnapLock 50 ms', sd_io_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:1844'})
CREATE (:Endpoint {id: 'ep_delete_log', method: 'DELETE', path: '/log/<file>', purpose: 'delete one CSV; refuses the active file', page: 'logs', calls_module: 'mod_sdlog', locks: 'sdMutex 2000 ms, then dirSnapLock 50 ms', sd_io_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:1922'})
CREATE (:Endpoint {id: 'ep_log_config', method: 'GET|POST', path: '/log_config', purpose: 'GPS, IMU, CAN and SAT channel flags; any request with parameters writes NVS, GET included', page: 'setup', calls_module: 'mod_sdlog', nvs_write_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:1984'})
CREATE (:Endpoint {id: 'ep_log_pause', method: 'GET|POST', path: '/log_pause', purpose: 'master pause, RAM only', page: 'logs', calls_module: 'mod_sdlog', source: 'code', cite: 'webserver.cpp:2038'})
CREATE (:Endpoint {id: 'ep_lcd', method: 'GET|POST', path: '/lcd', purpose: 'LCD on or off, RAM only', page: 'setup', calls_module: 'mod_display', source: 'code', cite: 'webserver.cpp:2059'})
CREATE (:Endpoint {id: 'ep_imu_cal', method: 'GET|POST', path: '/imu/cal', purpose: 'start or clear IMU calibration; state', page: 'setup', calls_module: 'mod_imu', source: 'code', cite: 'webserver.cpp:2079'})
CREATE (:Endpoint {id: 'ep_debug_log', method: 'GET|POST', path: '/debug_log', purpose: 'serial-to-SD tee on or off', page: 'setup', calls_module: 'mod_debug_log', source: 'code', cite: 'webserver.cpp:2105'})
CREATE (:Endpoint {id: 'ep_gnss_config', method: 'GET|POST', path: '/gnss_config', purpose: 'elevation and C/N0 masks, PPP mode; applied later in loop()', page: 'dashboard', calls_module: 'mod_gnss', source: 'code', cite: 'webserver.cpp:2128'})
CREATE (:Endpoint {id: 'ep_sd_deleteold', method: 'POST', path: '/sd/deleteold', purpose: 'delete every non-active CSV in one pass; replies deleted 0 if the card is busy', page: 'logs', calls_module: 'mod_sdlog', locks: 'sdMutex 1000 ms, held for the whole walk, then dirSnapLock 50 ms', sd_io_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2162'})
CREATE (:Endpoint {id: 'ep_export_build', method: 'POST', path: '/export/build', purpose: 'queue a tar of selected files, or every CSV when no names are given; built by SDLog', page: 'logs', calls_module: 'mod_sdlog', source: 'code', cite: 'webserver.cpp:2222'})
CREATE (:Endpoint {id: 'ep_export_status', method: 'GET', path: '/export/status', purpose: 'tar build state', page: 'logs', calls_module: 'mod_sdlog', source: 'code', cite: 'webserver.cpp:2235'})
CREATE (:Endpoint {id: 'ep_export_tar', method: 'GET', path: '/export.tar', purpose: 'stream the archive; needs ready, paused and no other download', page: 'logs', calls_module: 'mod_sdlog', sd_io_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2251'})
CREATE (:Endpoint {id: 'ep_can_sniff', method: 'POST', path: '/can/sniff', purpose: 'sniffer on or off; without force=1 it turns every log channel off and saves that', page: 'dashboard', calls_module: 'mod_can', locks: 'sniffMux portMAX_DELAY (only when enabling, so effectively bounded)', nvs_write_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2294'})
CREATE (:Endpoint {id: 'ep_can_snapshot', method: 'GET', path: '/can/snapshot', purpose: 'sniffer ID table', page: 'dashboard', calls_module: 'mod_can', locks: 'sniffMux 50 ms', allocates: '2560 B internal per request', source: 'code', cite: 'webserver.cpp:2325'})
CREATE (:Endpoint {id: 'ep_wifi', method: 'GET', path: '/wifi', purpose: 'device name, AP IP, active SSID, network list', page: 'setup', calls_module: 'mod_wifi', locks: 'dataMutex 50 ms', source: 'code', cite: 'webserver.cpp:2362'})
CREATE (:Endpoint {id: 'ep_wifi_edit', method: 'POST', path: '/wifi/add, /wifi/remove, /wifi/enable, /wifi/move', purpose: 'edit the station list; writes NVS', page: 'setup', calls_module: 'mod_wifi', nvs_write_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2399-2446'})
CREATE (:Endpoint {id: 'ep_device_name', method: 'POST', path: '/device/name', purpose: 'rename the device and AP', page: 'setup', calls_module: 'mod_wifi', nvs_write_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2384'})
CREATE (:Endpoint {id: 'ep_dbc', method: 'GET', path: '/dbc', purpose: 'DBC files, active file and audit text', page: 'setup', calls_module: 'mod_dbc_store', source: 'code', cite: 'webserver.cpp:2456'})
CREATE (:Endpoint {id: 'ep_dbc_edit', method: 'POST', path: '/dbc/upload, /dbc/select, /dbc/delete', purpose: 'upload into the PSRAM stage, select, delete; dbcTask does the SD work', page: 'setup', calls_module: 'mod_dbc_store', nvs_write_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2486-2503'})
CREATE (:Endpoint {id: 'ep_casters', method: 'GET', path: '/casters', purpose: 'caster table', page: 'setup', calls_module: 'mod_ntrip', source: 'code', cite: 'webserver.cpp:2509'})
CREATE (:Endpoint {id: 'ep_casters_edit', method: 'POST', path: '/casters/enable, /casters/add, /casters/remove, /casters/reset', purpose: 'edit casters (NVS) or reset cooldowns and backoff', page: 'setup', calls_module: 'mod_ntrip', nvs_write_on_async_tcp: true, source: 'code', cite: 'webserver.cpp:2529-2571'})
MATCH (e:Endpoint), (i {id: 'if_http_80'}) CREATE (i)-[:EXPOSES {}]->(e)
MATCH (e:Endpoint), (m:Module) WHERE e.calls_module = m.id CREATE (e)-[:INVOKES {}]->(m)


// ─────────────────────────────────────────────────────────────────────────────
// 20. Behaviour — the state machines other than cone survey (§17)
// ─────────────────────────────────────────────────────────────────────────────

CREATE (:StateMachine {id: 'sm_gnss_boot', name: 'GNSS boot configuration', owner: 'setup (gnss_init)', timing: 'RMC rate measured over 1.5 s, extended to 6 s if only 1-2 epochs seen', verification_gap: 'a hot-start boot reads back only PPP and NAVMODE, so GSA rate, constellations, VTG and GLL are never verified on a normal boot', source: 'code', cite: 'gnss.cpp:807-1034'})
CREATE (:State {id: 'st_gb_measure', name: 'Measure rate', machine: 'sm_gnss_boot', initial: true})
CREATE (:State {id: 'st_gb_skip', name: 'Hot start (no writes)', machine: 'sm_gnss_boot', shows: 'PPP and NAVMODE read-backs only'})
CREATE (:State {id: 'st_gb_full', name: 'Full configure', machine: 'sm_gnss_boot', shows: 'restart-triggering commands first, then the rest, SAVEPAR, read-backs, PQTMSRR; about 5-10 s'})
CREATE (:State {id: 'st_gb_run', name: 'Running', machine: 'sm_gnss_boot'})
MATCH (a {id: 'st_gb_measure'}), (b {id: 'st_gb_full'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'config version differs', guard: 'NVS cfg_ver != GNSS_CONFIG_VERSION, including fresh NVS', source: 'code'}]->(b)
MATCH (a {id: 'st_gb_measure'}), (b {id: 'st_gb_full'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'rate under 15 Hz', guard: 'measured > 0 and below 75 % of target, e.g. after a base-firmware cross-flash', source: 'code'}]->(b)
MATCH (a {id: 'st_gb_measure'}), (b {id: 'st_gb_skip'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'version matches, rate ok or unknown', note: 'after a mask-defaults bump the masks are still written on the first loop pass', source: 'code'}]->(b)
MATCH (a {id: 'st_gb_full'}), (b {id: 'st_gb_run'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'written', action: 'stamp cfg_ver only if the config was written', source: 'code'}]->(b)
MATCH (a {id: 'st_gb_skip'}), (b {id: 'st_gb_run'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'read-backs done', source: 'code'}]->(b)
MATCH (a {id: 'st_gb_run'}), (b {id: 'st_gb_run'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'web mask or PPP change', action: 'apply in gnss_loop: a mask change blocks loop() at least 0.45 s; a PPP change 0.9-1.6 s, because it re-applies the masks and its read-back flushes and discards up to 0.85 s of NMEA', source: 'code'}]->(b)

CREATE (:StateMachine {id: 'sm_ntrip', name: 'NTRIP session', owner: 'WiFiNTRIP task', timing: 'backoff 4 s first, then 30-60 s; source table at most every 5 min per caster; preferred cooldown 10 min, probe 5 min; hourly rescan at standstill; 20 s no-data watchdog', source: 'code', cite: 'ntrip.cpp:1101-1447'})
CREATE (:State {id: 'st_nt_nomount', name: 'No mount', machine: 'sm_ntrip', initial: true})
CREATE (:State {id: 'st_nt_select', name: 'Selecting', machine: 'sm_ntrip', shows: 'Phase 0 direct preferred mount, else geographic source-table scan'})
CREATE (:State {id: 'st_nt_connect', name: 'Connecting', machine: 'sm_ntrip'})
CREATE (:State {id: 'st_nt_stream', name: 'Streaming', machine: 'sm_ntrip', shows: 'RTCM to UART1, GGA keep-alive, baseline check'})
CREATE (:State {id: 'st_nt_backoff', name: 'Backoff', machine: 'sm_ntrip'})
MATCH (a {id: 'st_nt_nomount'}), (b {id: 'st_nt_select'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'good fix, backoff elapsed', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_select'}), (b {id: 'st_nt_connect'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'mount chosen', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_connect'}), (b {id: 'st_nt_stream'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'ICY 200 or HTTP 200', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_connect'}), (b {id: 'st_nt_backoff'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'connect fails', guard: 'fewer than 5 failures', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_connect'}), (b {id: 'st_nt_nomount'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'SOURCETABLE or 5th failure', action: 'clear the mount; a stale preferred mount cools down 5 min and selection re-arms at once without counting a failure', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_stream'}), (b {id: 'st_nt_backoff'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'drop or 20 s silence', guard: 'a drop inside 30 s counts as a failure; silence always does', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_stream'}), (b {id: 'st_nt_nomount'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'base > 100 km, or second silent session', action: 'clear the mount; a preferred mount cools down 10 min', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_stream'}), (b {id: 'st_nt_connect'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'preferred mount live, or hourly rescan finds closer', guard: 'standstill under 2 kn; the probe needs a valid fix and runs only on a public mount; the rescan needs 25 % less distance squared', action: 'switch the mount directly and reconnect', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_backoff'}), (b {id: 'st_nt_nomount'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'mount dropped', source: 'code'}]->(b)
MATCH (a {id: 'st_nt_backoff'}), (b {id: 'st_nt_connect'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'backoff elapsed', source: 'code'}]->(b)

CREATE (:StateMachine {id: 'sm_wifi', name: 'WiFi AP and station', owner: 'WiFiNTRIP task', timing: '30 s loyalty window with 10 s attempts on the last good network, then sweeps (seen networks first) at 20 s per attempt, 15 s between sweeps', source: 'code', cite: 'wifi_mgr.cpp:207-490'})
CREATE (:State {id: 'st_wf_ap', name: 'AP only', machine: 'sm_wifi', initial: true, shows: 'configuration AP up, station attempts running'})
CREATE (:State {id: 'st_wf_connected', name: 'Station connected', machine: 'sm_wifi', shows: 'AP renamed to an empty SSID for the rest of the boot; it still beacons'})
CREATE (:State {id: 'st_wf_loyal', name: 'Loyalty window', machine: 'sm_wifi'})
CREATE (:State {id: 'st_wf_sweep', name: 'Sweep', machine: 'sm_wifi'})
MATCH (a {id: 'st_wf_ap'}), (b {id: 'st_wf_connected'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'first association', action: 'apply an empty AP configuration (the AP stays up)', source: 'code'}]->(b)
MATCH (a {id: 'st_wf_connected'}), (b {id: 'st_wf_loyal'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'link lost', action: 'keep the NTRIP mount; AP stays retired', source: 'code'}]->(b)
MATCH (a {id: 'st_wf_loyal'}), (b {id: 'st_wf_connected'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'reconnected', source: 'code'}]->(b)
MATCH (a {id: 'st_wf_loyal'}), (b {id: 'st_wf_sweep'}) CREATE (a)-[:TRANSITIONS_TO {trigger: '30 s elapsed', guard: 'checked before each attempt, so the last attempt can end near 40 s; the sweep starts in the same call', source: 'code'}]->(b)
MATCH (a {id: 'st_wf_connected'}), (b {id: 'st_wf_sweep'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'link lost', guard: 'last good network no longer listed or enabled', source: 'code'}]->(b)
MATCH (a {id: 'st_wf_sweep'}), (b {id: 'st_wf_connected'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'network joined', source: 'code'}]->(b)

CREATE (:StateMachine {id: 'sm_ble_link', name: 'BLE link and TX slot', owner: 'NimBLE host task (link) and loop task (slot)', timing: 'one notification per 4 ms or more; 15 ms quiet after a meta; 250 ms meta resend guard; one sample per GNSS epoch with a 100 ms fallback', source: 'code', cite: 'ble_racecapture.cpp:417-495, 837-941, 1039-1133'})
CREATE (:State {id: 'st_bl_adv', name: 'Advertising', machine: 'sm_ble_link', initial: true})
CREATE (:State {id: 'st_bl_conn', name: 'Connected', machine: 'sm_ble_link'})
CREATE (:State {id: 'st_bl_sub', name: 'Subscribed', machine: 'sm_ble_link', shows: 'meta, version and samples flow through the one-message slot'})
MATCH (a {id: 'st_bl_adv'}), (b {id: 'st_bl_conn'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'central connects', action: 'streaming on; status.bleConnected under portMAX_DELAY', source: 'code'}]->(b)
MATCH (a {id: 'st_bl_conn'}), (b {id: 'st_bl_sub'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'CCCD subscribe', action: 'request 15-30 ms, latency 0, timeout 5 s; log the granted parameters', source: 'code'}]->(b)
MATCH (a {id: 'st_bl_sub'}), (b {id: 'st_bl_adv'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'disconnect', action: 'MTU back to 23 and re-advertise in the callback; the loop then drops any staged message, started or not', source: 'code'}]->(b)
MATCH (a {id: 'st_bl_sub'}), (b {id: 'st_bl_conn'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'CCCD unsubscribe', action: 'txSubscribed cleared; a partly sent message is dropped', source: 'code'}]->(b)
MATCH (a {id: 'st_bl_conn'}), (b {id: 'st_bl_adv'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'disconnect', source: 'code'}]->(b)

CREATE (:StateMachine {id: 'sm_vehicle', name: 'Vehicle profile', owner: 'CAN task', timing: 'first detection needs 2 distinct exclusive IDs; a swap needs 3; 5 s of silence clears data but keeps the profile', source: 'code', cite: 'can_bus.cpp:422-554'})
CREATE (:State {id: 'st_vh_unknown', name: 'Unknown', machine: 'sm_vehicle', initial: true})
CREATE (:State {id: 'st_vh_987', name: 'Porsche 987.2', machine: 'sm_vehicle'})
CREATE (:State {id: 'st_vh_718', name: 'Porsche 718', machine: 'sm_vehicle'})
MATCH (a {id: 'st_vh_unknown'}), (b {id: 'st_vh_987'}) CREATE (a)-[:TRANSITIONS_TO {trigger: '987 IDs seen', guard: 'at least 2 distinct exclusive IDs and more than the 718 count', action: 'save to NVS', source: 'code'}]->(b)
MATCH (a {id: 'st_vh_unknown'}), (b {id: 'st_vh_718'}) CREATE (a)-[:TRANSITIONS_TO {trigger: '718 IDs seen', guard: 'at least 2 distinct exclusive IDs and more than the 987 count', action: 'save to NVS', source: 'code'}]->(b)
MATCH (a {id: 'st_vh_987'}), (b {id: 'st_vh_718'}) CREATE (a)-[:TRANSITIONS_TO {trigger: '3 or more 718 IDs', guard: 'also when VEHICLE_FORCE pinned the profile', action: 'clear data, save to NVS', source: 'code'}]->(b)
MATCH (a {id: 'st_vh_718'}), (b {id: 'st_vh_987'}) CREATE (a)-[:TRANSITIONS_TO {trigger: '3 or more 987 IDs', action: 'clear data, save to NVS', source: 'code'}]->(b)

CREATE (:StateMachine {id: 'sm_export', name: 'Tar export', owner: 'SDLog task (web sets the request)', source: 'code', cite: 'sd_log.cpp:604-810, 1558-1587'})
CREATE (:State {id: 'st_ex_idle', name: 'Idle', machine: 'sm_export', initial: true})
CREATE (:State {id: 'st_ex_build', name: 'Building', machine: 'sm_export', shows: 'holds sdMutex for the whole copy; no queue drain meanwhile'})
CREATE (:State {id: 'st_ex_ready', name: 'Ready', machine: 'sm_export'})
CREATE (:State {id: 'st_ex_error', name: 'Error', machine: 'sm_export'})
MATCH (a {id: 'st_ex_idle'}), (b {id: 'st_ex_build'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'POST /export/build', source: 'code'}]->(b)
MATCH (a {id: 'st_ex_build'}), (b {id: 'st_ex_ready'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'archive written', source: 'code'}]->(b)
MATCH (a {id: 'st_ex_build'}), (b {id: 'st_ex_error'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'no buffer, sdMutex timeout or tar open failure', note: 'read and write failures still end in Ready, with a zero-filled or degraded archive', source: 'code'}]->(b)
MATCH (a {id: 'st_ex_ready'}), (b {id: 'st_ex_build'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'new build request', source: 'code'}]->(b)
MATCH (a {id: 'st_ex_error'}), (b {id: 'st_ex_build'}) CREATE (a)-[:TRANSITIONS_TO {trigger: 'new build request', source: 'code'}]->(b)

MATCH (s:State), (m:StateMachine) WHERE s.machine = m.id AND m.id <> 'sm_cone_status' AND m.id <> 'sm_lcd_mode' CREATE (s)-[:IN_MACHINE {}]->(m)
MATCH (a {id: 'sm_gnss_boot'}), (b {id: 'mod_gnss'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sm_ntrip'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sm_wifi'}), (b {id: 'mod_wifi'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sm_ble_link'}), (b {id: 'mod_ble'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sm_vehicle'}), (b {id: 'mod_can'}) CREATE (a)-[:OWNED_BY {}]->(b)
MATCH (a {id: 'sm_export'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:OWNED_BY {}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 21. Shared state and configuration
// ─────────────────────────────────────────────────────────────────────────────
// Who writes and who reads each piece of cross-task state, and what protects
// it. `protection: 'none'` marks a race the design accepts or has not noticed.

CREATE (:SharedState {id: 'ss_gps', name: 'gps (GnssData)', protection: 'dataMutex', source: 'code'})
CREATE (:SharedState {id: 'ss_imu', name: 'imu (ImuData)', protection: 'dataMutex', source: 'code'})
CREATE (:SharedState {id: 'ss_can', name: 'can (CanData) and g718Extra', protection: 'dataMutex', source: 'code'})
CREATE (:SharedState {id: 'ss_status', name: 'status (SystemStatus)', protection: 'dataMutex', note: 'status.imuTempC has no writer and is always NAN', source: 'code'})
CREATE (:SharedState {id: 'ss_gps_serial', name: 'gpsSerial (UART1)', protection: 'HardwareSerial internal lock only', hazard: 'a PQTM write from loop() can land between two 512 B RTCM chunks of one frame', source: 'code'})
CREATE (:SharedState {id: 'ss_cone_mode', name: 'coneModeActive', protection: 'none (plain bool; one pass of lag accepted)', source: 'code'})
CREATE (:SharedState {id: 'ss_lcd_enabled', name: 'lcdUserEnabled', protection: 'none', source: 'code'})
CREATE (:SharedState {id: 'ss_thermal', name: 'thermal gate flags', protection: 'volatile', source: 'code'})
CREATE (:SharedState {id: 'ss_rtcm_bytes', name: 'rtcmBytesTotal', protection: 'volatile', source: 'code'})
CREATE (:SharedState {id: 'ss_casters', name: 'caster table', protection: 'none: the web writes casters[].enabled directly', source: 'code'})
CREATE (:SharedState {id: 'ss_wifi_list', name: 'WiFi network list and device name', protection: 'none: web edits shift the array the WiFiNTRIP task reads', source: 'code'})
CREATE (:SharedState {id: 'ss_log_flags', name: 'log channel flags and pause', protection: 'volatile', source: 'code'})
CREATE (:SharedState {id: 'ss_export', name: 'export request and selection', protection: 'none: a second request rewrites the selection during a build', source: 'code'})
CREATE (:SharedState {id: 'ss_capacity', name: 'SD capacity cache', protection: 'none: 64-bit values can tear', source: 'code'})
CREATE (:SharedState {id: 'ss_dbc', name: 'DBC snapshot, status and report', protection: 'none: two writers (dbcTask, and async_tcp for status, active file and the upload stage); /dbc can read a half-written audit report', source: 'code'})
CREATE (:SharedState {id: 'ss_ble_flags', name: 'BLE link flags and ACK ring', protection: 'volatile, except bleClientConn, bleLastMetaMs and sampleCount, which both tasks write', source: 'code'})

MATCH (a {id: 'lock_datamutex'}), (b:SharedState) WHERE b.protection = 'dataMutex' CREATE (a)-[:GUARDS {}]->(b)

MATCH (a {id: 'task_loop'}), (b {id: 'ss_gps'}) CREATE (a)-[:WRITES {by: 'gnss_loop'}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_imu'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_can'}), (b {id: 'ss_can'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_status'}) CREATE (a)-[:WRITES {fields: 'espTempC, gnssHz, blePacketHz'}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_status'}) CREATE (a)-[:WRITES {fields: 'WiFi and NTRIP state (portMAX_DELAY); RaceCapture client state (5 ms)', lock_wait: 'portMAX_DELAY'}]->(b)
MATCH (a {id: 'task_nimble'}), (b {id: 'ss_status'}) CREATE (a)-[:WRITES {fields: 'bleConnected', lock_wait: 'portMAX_DELAY'}]->(b)
MATCH (a {id: 'task_can'}), (b {id: 'ss_status'}) CREATE (a)-[:WRITES {fields: 'canBusOk, canHz'}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_gps'}) CREATE (a)-[:READS {timeout_ms: 10}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_imu'}) CREATE (a)-[:READS {timeout_ms: 10}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_status'}) CREATE (a)-[:READS {timeout_ms: 10}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_gps'}) CREATE (a)-[:READS {timeout_ms: 5}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_can'}) CREATE (a)-[:READS {timeout_ms: 5}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_can'}) CREATE (a)-[:READS {by: 'BLE snapshot and SD rows', timeout_ms: 5}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_status'}) CREATE (a)-[:READS {by: '/status', timeout_ms: 50}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_gps_serial'}) CREATE (a)-[:READS {by: 'NMEA drain; also writes PQTM commands'}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_gps_serial'}) CREATE (a)-[:WRITES {by: 'RTCM relay, up to 8 x 512 B per pass'}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_cone_mode'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_cone_mode'}) CREATE (a)-[:READS {by: 'sdlog_push_imu'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_lcd_enabled'}) CREATE (a)-[:WRITES {by: '/lcd'}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_lcd_enabled'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_thermal'}) CREATE (a)-[:WRITES {by: 'thermal_update'}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_thermal'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'ss_thermal'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_rtcm_bytes'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_rtcm_bytes'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_casters'}) CREATE (a)-[:WRITES {by: '/casters/*'}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_casters'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_wifi_list'}) CREATE (a)-[:WRITES {by: '/wifi/*, /device/name'}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_wifi_list'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'ss_wifi_list'}) CREATE (a)-[:READS {field: 'device name'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_log_flags'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'ss_log_flags'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_log_flags'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_export'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'ss_export'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'ss_capacity'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_capacity'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_dbc'}), (b {id: 'ss_dbc'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_dbc'}) CREATE (a)-[:READS {}]->(b)
MATCH (a {id: 'task_nimble'}), (b {id: 'ss_ble_flags'}) CREATE (a)-[:WRITES {}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_ble_flags'}) CREATE (a)-[:READS {}]->(b)

CREATE (:NvsNamespace {id: 'nvs_rcx_gnss', name: 'rcx_gnss', keys: 'cfg_ver, mask_ver, ele, cnr, ppp', owner: 'mod_gnss', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_ntrip', name: 'rcx_ntrip', keys: 'n; h, p, u, w, m, e per added caster; d per default caster', owner: 'mod_ntrip', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_wifi', name: 'rcx_wifi', keys: 'n; s, p, e per network; name', owner: 'mod_wifi', hazard: 'saving the network list clears the namespace and erases the device name', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_imu', name: 'rcx_imu', keys: 'valid, abx, aby, abz, gbx, gby, gbz', owner: 'mod_imu', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_log', name: 'rcx_log', keys: 'gps, imu, can, sat', owner: 'mod_sdlog', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_dbc', name: 'rcx_dbc', keys: 'active', owner: 'mod_dbc_store', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_dbg', name: 'rcx_dbg', keys: 'en', owner: 'mod_debug_log', source: 'code'})
CREATE (:NvsNamespace {id: 'nvs_rcx_veh', name: 'rcx_veh', keys: 'prof', owner: 'mod_can', source: 'code'})
MATCH (n:NvsNamespace), (m:Module) WHERE n.owner = m.id CREATE (m)-[:PERSISTS_IN {}]->(n)

CREATE (:ConfigFlag {id: 'cfg_gnss_config_version', name: 'GNSS_CONFIG_VERSION', value: '9 (+100 with PPP_NAV_DEBUG)', effect: 'a change forces a full LG290P reconfigure at the next boot', configures: 'mod_gnss', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_gnss_rate', name: 'GNSS_RATE_MS', value: '50', effect: '20 Hz fix rate. A module measured below 75 % of it is reconfigured automatically; changing this value to a slower rate takes effect only with a GNSS_CONFIG_VERSION bump', configures: 'mod_gnss', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_drain_budget', name: 'GNSS_DRAIN_BUDGET_BYTES', value: '512', effect: 'NMEA bytes parsed per loop() pass', configures: 'mod_gnss', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_epoch_watchdog', name: 'GNSS_EPOCH_WATCHDOG_MS', value: '45', effect: 'publishes a GPS row if RMC never completes the epoch', configures: 'mod_sdlog', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_ntrip_io_timeout', name: 'NTRIP_CLIENT_IO_TIMEOUT_MS', value: '250', effect: 'sets the Stream read timeout after the handshake; it bounds neither the GGA write nor the RTCM reads, which are available()-guarded (arduino-esp32 3.3.8)', configures: 'mod_ntrip', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_modem_sleep', name: 'WIFI_MODEM_SLEEP_FOR_COEX', value: 'true', effect: 'none: referenced nowhere; WiFi.setSleep(false) runs unconditionally', configures: 'mod_wifi', status: 'dead', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_debug_serial_sd', name: 'DEBUG_SERIAL_TO_SD', value: 'false', effect: 'serial-to-SD tee; enabling it fails to compile (webserver.cpp:2114), then to link', configures: 'mod_debug_log', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_debug_watermarks', name: 'DEBUG_WATERMARKS', value: 'true', effect: '30 s heap and stack print', configures: 'mod_main', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_racecapture', name: 'RACECAPTURE_ENABLE', value: 'true', effect: 'TCP 7223 server', configures: 'mod_racecapture', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_sd_log', name: 'SD_LOG_ENABLE', value: 'true', effect: 'SD logging and SDLog task', configures: 'mod_sdlog', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_can', name: 'CAN_ENABLE', value: 'true', effect: 'false compiles CAN stubs', configures: 'mod_can', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_imu_motionless', name: 'IMU_MOTIONLESS_*', value: 'gyro 3 °/s, accel 0.03 g, dwell 60 s, then one row per 1000 ms', effect: 'IMU SD decimation while still; off in cone survey mode', configures: 'mod_sdlog', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_dbc_max_channels', name: 'DBC_MAX_CHANNELS', value: '100', effect: 'DBC signal table size', configures: 'mod_dbc_parse', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_ppp_nav_debug', name: 'PPP_NAV_DEBUG', value: 'false', effect: 'PQTMPPPNAV output and extra read-backs; bumps the config version by 100', configures: 'mod_gnss', source: 'code'})
CREATE (:ConfigFlag {id: 'cfg_ble_tx_debug', name: 'BLE_TX_DEBUG', value: '0', effect: 'per-send serial logs; defined in ble_racecapture.cpp, not config.h', configures: 'mod_ble', source: 'code', cite: 'ble_racecapture.cpp:219-226'})
CREATE (:ConfigFlag {id: 'cfg_vehicle_force', name: 'VEHICLE_FORCE', value: 'undefined', effect: 'pins the vehicle profile at boot; a runtime swap can still override it', configures: 'mod_can', source: 'code'})
MATCH (c:ConfigFlag), (m:Module) WHERE c.configures = m.id CREATE (c)-[:CONFIGURES {}]->(m)


MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_imu'}) CREATE (a)-[:READS {by: 'RaceCapture snapshot', timeout_ms: 5}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_status'}) CREATE (a)-[:READS {by: 'SD rows (espTempC, imuTempC)', timeout_ms: 5}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_thermal'}) CREATE (a)-[:READS {by: 'IMU and SAT row gates'}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_rtcm_bytes'}) CREATE (a)-[:READS {by: 'GPS rows'}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'ss_casters'}) CREATE (a)-[:WRITES {by: 'loadCasters() on reload, while /casters may hold pointers into the table'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'ss_dbc'}) CREATE (a)-[:WRITES {by: 'upload stage, status, active file'}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'ss_ble_flags'}) CREATE (a)-[:WRITES {by: 'clears pending meta and version, advances the ACK ring, bleLastMetaMs, sampleCount'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 22. Locks, rules and latent defects found in the source
// ─────────────────────────────────────────────────────────────────────────────
// A Defect is found by reading the code and has not been seen in the field;
// a FailureMode (§12-§16) has been observed. Each Defect cites file:line.

MATCH (a {id: 'task_wifintrip'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'snapshot for NTRIP and RaceCapture', timeout_ms: 5, on_timeout: 'skip the consumers this pass', source: 'code'}]->(b)
MATCH (a {id: 'task_wifintrip'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'NTRIP and WiFi status writes (15 sites)', timeout_ms: -1, timeout_label: 'portMAX_DELAY', violates: 'rule_no_unbounded_wait', source: 'code'}]->(b)
MATCH (a {id: 'task_display'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'gps, imu and status snapshot', timeout_ms: 10, on_timeout: 'draws default-constructed data', source: 'code'}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'capacity refresh, canraw open', timeout_ms: 300, source: 'code'}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'dir-scan open', timeout_ms: 500, source: 'code'}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'close file; tar build holds it for the whole copy', timeout_ms: 1000, on_timeout: 'closes the file without the mutex', source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'DELETE /log/<file>', timeout_ms: 2000, source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'POST /sd/deleteold, whole walk', timeout_ms: 1000, hazard: 'unbounded hold on the web task', source: 'code'}]->(b)
MATCH (a {id: 'task_can'}), (b {id: 'lock_sniffmux'}) CREATE (a)-[:ACQUIRES {role: 'sniffer update per drain batch', timeout_ms: 2, on_timeout: 'skip the batch', source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_sniffmux'}) CREATE (a)-[:ACQUIRES {role: 'GET /can/snapshot', timeout_ms: 50, source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_sniffmux'}) CREATE (a)-[:ACQUIRES {role: 'POST /can/sniff enable', timeout_ms: -1, timeout_label: 'portMAX_DELAY', violates: 'rule_no_unbounded_wait', source: 'code'}]->(b)

MATCH (a {id: 'task_wifintrip'}), (b {id: 'rule_no_unbounded_wait'}) CREATE (a)-[:VIOLATES {detail: 'portMAX_DELAY dataMutex takes at ntrip.cpp:305 (on the RTCM forward path), 836, 941, 1149, 1172, 1290, 1323, 1392, 1473, 1498; wifi_mgr.cpp:325, 359, 385, 432; RCX_RTK_Datalogger.ino:184', status: 'open', source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'rule_no_unbounded_wait'}) CREATE (a)-[:VIOLATES {detail: 'can_setSniffer takes sniffMux with portMAX_DELAY from POST /can/sniff', status: 'open', source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'rule_async_clean'}) CREATE (a)-[:VIOLATES {detail: 'deletes, /sd/deleteold and downloads do SD I/O on async_tcp; sdMutex waits of 1000-2000 ms; NVS commits from /log_config, /can/sniff, /wifi/*, /casters/*, /device/name, /dbc/select and (when compiled in) /debug_log', status: 'open', source: 'code'}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'rule_epoch_5ms'}) CREATE (a)-[:VIOLATES {detail: 'a web mask change blocks loop() at least 450 ms; a PPP change 0.9-1.6 s including the forced mask re-apply, and its read-back flushes and discards up to 0.85 s of NMEA whatever the ring size. notify() can also wait about 10 ms for a NimBLE buffer on every BLE pass.', trigger: 'operator action, or a mask-defaults bump on the first pass after boot; the notify() wait under BLE congestion', status: 'open', source: 'code'}]->(b)

CREATE (:Rule {id: 'rule_ble_callback_context', name: 'NimBLE host callbacks only set flags', statement: 'Never notify() or vTaskDelay() in a host callback; callbacks set flags or enqueue and the loop task does the work.', source: 'code', cite: 'ble_racecapture.cpp:93-98, 829'})
CREATE (:Rule {id: 'rule_serial_never_blocks', name: 'Serial never blocks loop()', statement: 'Serial.setTxTimeoutMs(0) under USB CDC; a blocking console once collapsed BLE to 3-4 Hz.', source: 'code', cite: 'RCX_RTK_Datalogger.ino:261-271'})
CREATE (:Rule {id: 'rule_can_tx_first', name: 'CAN TX pin is driven high first', statement: 'GPIO9 is set HIGH before anything else so the SN65HVD230 stays recessive until TWAI owns it; a dominant bus can put the engine in limp mode.', source: 'code', cite: 'RCX_RTK_Datalogger.ino:239-257'})
CREATE (:Rule {id: 'rule_wifi_after_gnss', name: 'No WiFi mode change before gnss_init()', statement: 'wifi_init() reads NVS only; WiFi.mode() and softAP() come after the GNSS is configured.', source: 'code', cite: 'RCX_RTK_Datalogger.ino:317-323; wifi_mgr.h:29-33'})
CREATE (:Rule {id: 'rule_radio_discipline', name: 'WiFi radio discipline', statement: 'Set WiFi.mode() once; never disconnect before an attempt; WiFi.persistent(false) because flash commits stall the GNSS drain; auto-reconnect off.', source: 'code', cite: 'wifi_mgr.cpp:10-24'})
CREATE (:Rule {id: 'rule_wire_same_context', name: 'The I2C bus has one context', statement: 'Wire is used only from the imu_read() context, so it needs no lock; other tasks request calibration through flags.', source: 'code', cite: 'imu.h:35-49, 71-74'})
CREATE (:Rule {id: 'rule_backlight_after_tft', name: 'Backlight PWM attaches after tft.init()', statement: 'ledcAttach() must come after tft.init(); TFT_BL stays undefined to avoid an IO 46 conflict.', source: 'code', cite: 'display.cpp:226-232; readme.md:57'})
CREATE (:Rule {id: 'rule_english_units', name: 'CAN data and speed in English units; GNSS quality in metric', statement: 'CAN-derived channels and speed go out in English units (mph, °F, psi) through the types.h helpers, with NaN propagating. GNSS accuracy and position quality (HAcc, VAcc, scatter, altitude, base distance) stay metric.', conformance: 'the code conforms: GPS speed is mph on BLE, RaceCapture TCP and the SD log (knots x 1.15078); CAN channels are converted; h_acc_m, v_acc_m and alt_msl_m are metric', source: 'owner', cite: 'types.h:158-171; ble_racecapture.cpp:269, 662; racecapture.cpp:131, 290; sd_log.cpp:309, 1176'})
CREATE (:Rule {id: 'rule_reserved_pins', name: 'Native USB pins are reserved', statement: 'GPIO19 and GPIO20 carry native USB, the flashing and console path. No firmware or model change may assign them. Pin assignments come only from the Waveshare ESP32-S3-LCD-1.47B wiring graph, never from another ESP32 board or revision.', conformance: 'no source file uses GPIO19 or GPIO20', source: 'owner', cite: 'config.h pin map; wiring graph esp_io19, esp_io20'})
MATCH (a {id: 'rule_reserved_pins'}), (b {id: 'mod_config'}) CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'rule_reserved_pins'}), (b {id: 'if_usb_serial'}) CREATE (a)-[:GOVERNS {}]->(b)
MATCH (a {id: 'mod_ble'}), (b {id: 'rule_ble_callback_context'}) CREATE (a)-[:ENFORCES {status: 'partial: onConnect and onDisconnect take dataMutex with portMAX_DELAY, onDisconnect restarts advertising, onSubscribe requests connection parameters'}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'rule_serial_never_blocks'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'rule_can_tx_first'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_main'}), (b {id: 'rule_wifi_after_gnss'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_wifi'}), (b {id: 'rule_radio_discipline'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_imu'}), (b {id: 'rule_wire_same_context'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'rule_backlight_after_tft'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_types'}), (b {id: 'rule_english_units'}) CREATE (a)-[:ENFORCES {}]->(b)

CREATE (:Defect {id: 'dfx_latlon_swapped', name: 'Latitude and longitude drawn under the wrong labels', severity: 'low', where: 'display.cpp:1015-1016 (labels at 240, 743)', detail: 'The normal layout draws latitude at y=164 and longitude at y=144, under the Lon and Lat labels. Cone mode draws them correctly.', subject: 'mod_display', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_imu_fail_reads_flat', name: 'A failed IMU read looks like a flat, still unit', severity: 'medium', where: 'imu.cpp:216-222; types.h:72', detail: 'out = {} leaves az = 1.0, so a failed read counts as horizontal toward cone-mode exit and as still toward IMU decimation, and the (0, 0, 1 g) sample is published as real data to BLE, RaceCapture and the SD IMU log. Only a short read is detected; endTransmission() is unchecked.', subject: 'mod_imu', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_display_default_snapshot', name: 'Display draws empty data when its lock take fails', severity: 'low', where: 'RCX_RTK_Datalogger.ino:109-119', detail: 'The snapshots are default-constructed inside the loop, so a missed 10 ms take feeds valid=false and az=1.0: the banner and LED turn red if that pass redraws, the WiFi rows flip to the AP text, and the pass counts toward the 10 s cone-mode exit. WiFiNTRIP skips instead.', subject: 'mod_main', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_cone_memcpy_alias', name: 'Cone median copies a buffer onto itself', severity: 'low', where: 'display.cpp:482, 491-496', detail: 'coneMedian(scratch, n, scratch) calls memcpy with the same source and destination: undefined behaviour, a no-op in practice. memmove or a pointer check fixes it.', subject: 'mod_cone_survey', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_uart1_two_writers', name: 'Two tasks write UART1', severity: 'medium', where: 'ntrip.cpp:1269; gnss.cpp:989-990, 1103-1104', detail: 'Comments say the GNSS code is the only UART writer, but the WiFiNTRIP task writes RTCM concurrently. A PQTM command from loop() can split one RTCM frame; the LG290P resyncs on 0xD3.', subject: 'mod_gnss', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_modem_sleep_dead', name: 'Modem-sleep coexistence flag does nothing', severity: 'medium', where: 'config.h:53-57; wifi_mgr.cpp:189', detail: 'WIFI_MODEM_SLEEP_FOR_COEX is true but referenced nowhere, and ensureMode() calls WiFi.setSleep(false) unconditionally, so WiFi power save is always off.', relevance: 'bears on contender (b), rover-side coexistence, of fm_ble_master_anchor_skipping', subject: 'mod_wifi', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_device_name_erased', name: 'Editing the WiFi list erases the device name', severity: 'medium', where: 'wifi_mgr.cpp:106-117, 171', detail: 'saveNetworks() clears the rcx_wifi namespace, which also holds the device name, so any list edit reverts the AP SSID and the name on the LCD and in /wifi at the next boot. The BLE name is a compile-time constant and is unaffected.', subject: 'mod_wifi', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_imu_sat_only_never_log', name: 'IMU or SAT alone never logs', severity: 'medium', where: 'sd_log.cpp:272, 1122-1245', detail: 'With GPS and CAN both off no GPS/CAN record is queued, and files only open when one arrives, so IMU and SAT channels silently record nothing.', subject: 'mod_sdlog', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_name_width_32', name: 'Filename copies use 32 bytes, not LOG_NAME_MAX', severity: 'medium', where: 'sd_log.cpp:214, 1486, 1503-1504, 1537', detail: 'The 32-byte compares (214, 1503-1504) and copies (1486, 1537) are safe for GPS, CAN, IMU and SAT names (at most 31 characters), but canraw names with an ordinal reach 32-34, so the copy into the snapshot page drops the terminator and /files can show garbled names.', subject: 'mod_sdlog', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_export_select_race', name: 'Second export request rewrites a running build', severity: 'low', where: 'sd_log.cpp:1563-1586', detail: 'sdlog_requestExport writes the selection before checking that no build is running.', subject: 'mod_sdlog', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_canraw_silent_drop', name: 'Raw CAN rows lost to a lock timeout are not counted', severity: 'low', where: 'sd_log.cpp:1081-1082, 105', detail: 'Only queue-full drops are counted; a missed 50 ms sdMutex take loses the row silently, and those writes skip printlnRetry.', subject: 'mod_sdlog', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_deleteold_unbounded', name: '/sd/deleteold holds sdMutex for the whole card', severity: 'medium', where: 'webserver.cpp:2162-2210', detail: 'One sdMutex hold covers a full root walk and every delete, on async_tcp; log writers drop rows after 50 ms.', subject: 'mod_web', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_dbc_parse_holds_sd', name: 'DBC parse holds sdMutex for the whole file', severity: 'medium', where: 'dbc_store.cpp:284-297; dbc_parse.cpp:127-205', detail: 'The line-by-line parse of up to 192 KB runs under sdMutex, so log writers drop rows for its duration.', subject: 'mod_dbc_store', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_dbc_select_no_audit', name: 'Selecting a DBC file never audits it', severity: 'low', where: 'dbc_store.cpp:191-195, 282', detail: 'dbc_setActive sets the audit request, but dbcTask only checks commit, delete and scan, so the status stays auditing until another request arrives. dbc_requestScan has no caller.', subject: 'mod_dbc_store', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_dbc_not_decoded', name: 'DBC signals are parsed but never decoded', severity: 'info', where: 'dbc_parse.h: dbc_signalCount, dbc_signalAt', detail: 'No caller reads the signal table; BLE, RaceCapture and SD still use the fixed channel tables. This is the unbuilt second half of the DBC feature.', subject: 'mod_dbc_parse', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_debug_tee_link', name: 'Serial-to-SD tee cannot be enabled', severity: 'low', where: 'debug_log.cpp:132-190; sd_log.h:157-159', detail: 'drainToFile() and closeFile() have no caller; webserver.cpp:2114 calls DebugLog::currentFileName() without a declaration, so DEBUG_SERIAL_TO_SD fails to compile, and sdlog_takeMutex, sdlog_giveMutex and sdlog_sessionStamp are never defined, so it would then fail to link. sd_log.cpp does not include debug_log.h, so its own prints would not be captured.', subject: 'mod_debug_log', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_rc_blocking_write', name: 'RaceCapture TCP writes block the WiFiNTRIP task', severity: 'medium', where: 'racecapture.cpp:247-264, 364', detail: 'No timeout is set on rcClient, so print/write use the core write loop (up to about 10 s), the same class as the GGA keep-alive stall; the channel list alone is 68 print() calls. racecapture.h says no blocking I/O. Its 50 ms timer also drifts, which the BLE path dropped.', subject: 'mod_racecapture', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_rc_semantics', name: 'RaceCapture TCP fields differ from BLE', severity: 'low', where: 'racecapture.cpp:120-121, 281-295, 374', detail: 'Interval, Utc and GPSQual mean different things than on BLE; unknown commands get no reply.', subject: 'mod_racecapture', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_vehicle_force_swappable', name: 'VEHICLE_FORCE can be overridden at runtime', severity: 'low', where: 'can_bus.cpp:430-442, 508-527', detail: 'The forced profile still swaps after 3 IDs from the other platform and saves the swap to NVS.', subject: 'mod_can', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_static_internal_buffers', name: 'About 27 KB of static buffers sit in internal RAM', severity: 'medium', where: 'ble_racecapture.cpp:584, 600; racecapture.cpp:270; can_bus.cpp:64-65; webserver.cpp:1672-1696; sd_log.cpp:174-175, 617', detail: 'BLE meta and sample build buffers, the RaceCapture sample buffer, the CAN sniffer table and index, and web name tables are .bss in the binding pool. /can/snapshot also mallocs 2,560 B internal per request.', subject: 'internal_sram', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_mask_redundant_write', name: 'Masks written twice after a rate repair', severity: 'info', where: 'gnss.cpp:991', detail: 'When a mask-defaults version bump coincides with a measured rate repair on the same boot, the masks are written again on the first loop pass; the condition should test configWritten.', subject: 'mod_gnss', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_rescan_private', name: 'Hourly rescan spends table budget on private casters', severity: 'info', where: 'ntrip.cpp:705-716, 820-826', detail: 'rescanForBetter() scans every enabled caster, while the first selection skips casters with a preferred mount. The fetch blocks the WiFiNTRIP task for up to about 28 s, so RTCM forwarding stops meanwhile, and it can move the rover to a public mount the first selection would never choose.', severity_note: 'raised from info: RTCM stops during the fetch', subject: 'mod_ntrip', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_imu_temp_unwired', name: 'IMU temperature is never read', severity: 'info', where: 'imu.cpp:264; types.h:138; sd_log.cpp:357', detail: 'imu_readTempC() has no caller and status.imuTempC has no writer; the value is captured into the GPS log record but no CSV writes it.', subject: 'mod_imu', status: 'open', source: 'code'})
MATCH (d:Defect), (m) WHERE d.subject = m.id CREATE (d)-[:ARISES_FROM {}]->(m)
MATCH (a {id: 'dfx_name_width_32'}), (b {id: 'rule_log_name_max'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'dfx_static_internal_buffers'}), (b {id: 'rule_psram_first'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'dfx_deleteold_unbounded'}), (b {id: 'rule_async_clean'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'dfx_dbc_parse_holds_sd'}), (b {id: 'budget_sdmutex_hold'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'dfx_rc_blocking_write'}), (b {id: 'fm_ntrip_gga_blocking_tx'}) CREATE (a)-[:RISKS_REPEATING {}]->(b)
MATCH (a {id: 'dfx_modem_sleep_dead'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:MAY_AMPLIFY {contender: '(b) rover-side coexistence'}]->(b)
MATCH (a {id: 'dfx_imu_fail_reads_flat'}), (b {id: 'sm_lcd_mode'}) CREATE (a)-[:DEGRADES {how: 'a failed read counts toward the 10 s exit'}]->(b)
MATCH (a {id: 'dfx_uart1_two_writers'}), (b {id: 'flow_rtcm'}) CREATE (a)-[:DEGRADES {}]->(b)
MATCH (a {id: 'lever_static_to_psram'}), (b {id: 'dfx_static_internal_buffers'}) CREATE (a)-[:RESOLVES {}]->(b)

CREATE (:Hypothesis {id: 'hyp_imu_gaps_decimation', statement: 'The 1000 ms IMU gaps in gap_imu_low_rate match the motionless decimation, which writes one IMU row per 1000 ms after 60 s still. A bench session is still.', status: 'UNCONFIRMED — check whether that build already had IMU_MOTIONLESS_* decimation', source: 'derived'})
CREATE (:Hypothesis {id: 'hyp_oth_vtg_gll', statement: 'oth=300 per 5 s equals EPE 100 + VTG 100 + GLL 100: the VTG-off and GLL-off writes may not have stuck. The boot read-backs check GSA and constellations but never VTG or GLL.', status: 'UNCONFIRMED — read back GNVTG and GNGLL rates', source: 'derived'})
MATCH (a {id: 'hyp_imu_gaps_decimation'}), (b {id: 'gap_imu_low_rate'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_oth_vtg_gll'}), (b {id: 'gap_oth_sentence_rate'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'inst_loop_phase'}), (b {id: 'mod_imu'}) CREATE (a)-[:MEASURES {}]->(b)
MATCH (a {id: 'inst_loop_phase'}), (b {id: 'mod_ble'}) CREATE (a)-[:MEASURES {}]->(b)
MATCH (a {id: 'inst_loop_phase'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:MEASURES {}]->(b)
MATCH (a {id: 'inst_loop_phase'}), (b {id: 'mod_thermal'}) CREATE (a)-[:MEASURES {}]->(b)

MATCH (a {id: 'task_loop'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'gnssHz write in gnss_loop', timeout_ms: 10, source: 'code', cite: 'gnss.cpp:1386'}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'SD row epoch key; 718 extras for BLE and SD rows', timeout_ms: 2, on_timeout: 'a missed 718-extras take drops TPS and every 718-only channel from that sample', source: 'code', cite: 'sd_log.cpp:274; can_bus.cpp:625-632'}]->(b)
MATCH (a {id: 'task_loop'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'can_init at boot', timeout_ms: -1, timeout_label: 'portMAX_DELAY', source: 'code', cite: 'can_bus.cpp:225'}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'lock_dirsnap'}) CREATE (a)-[:ACQUIRES {role: 'snapshot note open or close', timeout_ms: 50, on_timeout: 'skip the update', source: 'code', cite: 'sd_log.cpp:455, 472'}]->(b)
MATCH (a {id: 'task_sdlog'}), (b {id: 'lock_dirsnap'}) CREATE (a)-[:ACQUIRES {role: 'boot-scan merge', timeout_ms: 100, source: 'code', cite: 'sd_log.cpp:970'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_dirsnap'}) CREATE (a)-[:ACQUIRES {role: 'note a deleted file', timeout_ms: 50, source: 'code', cite: 'sd_log.cpp:486'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_dirsnap'}) CREATE (a)-[:ACQUIRES {role: 'file list page', timeout_ms: 100, source: 'code', cite: 'sd_log.cpp:1534'}]->(b)

// PSRAM users that §6 did not list.
CREATE (:Buffer {id: 'buf_tft_sprite', name: 'LCD g-meter sprite', bytes: 24768, placement: 'psram', note: 'placed by the TFT_eSprite default when PSRAM exists', source: 'code', cite: 'display.cpp:236'})
CREATE (:Buffer {id: 'buf_dbc_stage', name: 'DBC upload stage', bytes: 196608, placement: 'psram', fallback: 'internal', lifetime: 'allocated on the first upload and kept', source: 'code', cite: 'dbc_store.cpp:108-112'})
CREATE (:Buffer {id: 'buf_dbc_signals', name: 'DBC signal table', bytes: 7200, placement: 'psram', fallback: 'internal', source: 'code', cite: 'dbc_parse.cpp:112-116'})
CREATE (:Buffer {id: 'buf_dbc_report', name: 'DBC audit report', bytes: 4096, placement: 'psram', fallback: 'internal', source: 'code', cite: 'dbc_parse.cpp:117-120'})
MATCH (a {id: 'buf_tft_sprite'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 24768, source: 'code'}]->(b)
MATCH (a {id: 'buf_tx_slot'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 6144, source: 'code'}]->(b)
MATCH (a {id: 'buf_dbc_stage'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 196608, transient: true, source: 'code'}]->(b)
MATCH (a {id: 'buf_dbc_signals'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 7200, source: 'code'}]->(b)
MATCH (a {id: 'buf_dbc_report'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 4096, source: 'code'}]->(b)
MATCH (a {id: 'mod_display'}), (b {id: 'buf_tft_sprite'}) CREATE (a)-[:WRITES_TO {}]->(b)
MATCH (a {id: 'mod_dbc_store'}), (b {id: 'buf_dbc_stage'}) CREATE (a)-[:WRITES_TO {}]->(b)
MATCH (a {id: 'mod_dbc_parse'}), (b {id: 'buf_dbc_signals'}) CREATE (a)-[:WRITES_TO {}]->(b)
MATCH (a {id: 'mod_dbc_parse'}), (b {id: 'buf_dbc_report'}) CREATE (a)-[:WRITES_TO {}]->(b)

// Defects found on the second, adversarial pass.
CREATE (:Defect {id: 'dfx_ntrip_write_unbounded', name: 'NTRIP and RaceCapture socket writes are not bounded by the 250 ms timeout', severity: 'medium', where: 'ntrip.cpp:935, 1352-1360; racecapture.cpp:247-264, 364', detail: 'setTimeout() sets only the Stream read timeout in arduino-esp32 3.3.8. NetworkClient::write() retries select() at 1 s up to 10 times, so a GGA keep-alive or RaceCapture write against a full send buffer can block the WiFiNTRIP task for about 10 s, stopping RTCM forwarding. setConnectionTimeout() or a non-blocking availableForWrite() check would bound it.', subject: 'mod_ntrip', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_ap_still_beaconing', name: 'The retired AP keeps beaconing', severity: 'medium', where: 'wifi_mgr.cpp:284-307 (core 3.3.8 WiFiAP.cpp:80-88, AP.cpp:260-275)', detail: 'Retiring the AP calls softAPdisconnect(false), which applies an empty-SSID open AP configuration and leaves the AP enabled, so it keeps beaconing on channel 1 for the whole boot. The design intent is that the AP is retired permanently.', subject: 'mod_wifi', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_probe_heap_gated', name: 'Preferred-mount probe silently gated by the scan heap guard', severity: 'medium', where: 'ntrip.cpp:780', detail: 'The 5-minute probe that moves the rover back to the RCX1 mount uses the same 30,000 B guard as the source-table scan and skips without a log, so while fm_scan_gate holds the rover never moves back.', subject: 'mod_ntrip', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_ap_only_idle_lock', name: 'AP-only operation takes dataMutex without bound every 10 ms', severity: 'low', where: 'wifi_mgr.cpp:385, 446; config.h:48-50', detail: 'With no stored networks, wifi_service() calls markIdle() before its rate gate, and markIdle() takes dataMutex with portMAX_DELAY on every WiFiNTRIP pass. The config.h note that the retry interval bounds the idle cost is false.', subject: 'mod_wifi', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_readback_flushes_nmea', name: 'GNSS read-backs flush the NMEA ring', severity: 'medium', where: 'gnss.cpp:98, 138, 184, 220, 288, 332', detail: 'Each read-back empties the UART RX ring before waiting for its reply and consumes incoming bytes while it waits. At boot this is harmless; at runtime the PPP apply discards up to 0.85 s of NMEA.', subject: 'mod_gnss', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_dbc_upload_race', name: 'A second DBC upload during a pending commit saves an empty file', severity: 'medium', where: 'dbc_store.cpp:97-100, 210-213', detail: 'dbc_uploadBegin zeroes the staged length before checking for a pending commit; the commit then writes 0 bytes, passes its length check and saves an empty DBC.', subject: 'mod_dbc_store', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_export_invalid_names', name: 'An export with only invalid names archives the whole card', severity: 'low', where: 'sd_log.cpp:752-781, 1563-1583', detail: 'If every requested name fails validation the selection is empty but still marked as a subset, so the build walks every CSV and names the archive rcx_logs_selected.tar.', subject: 'mod_sdlog', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_export_degraded_ready', name: 'A degraded tar archive is reported as ready', severity: 'low', where: 'sd_log.cpp:694-700, 794-806', detail: 'Read and write failures zero-fill or degrade the archive but the build still ends in Ready; only a console line warns.', subject: 'mod_sdlog', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_deleteold_busy_zero', name: 'Delete-old reports zero deletions when the card is busy', severity: 'low', where: 'webserver.cpp:2172, 2211-2213', detail: 'When the 1000 ms sdMutex take fails the handler replies 200 with deleted 0, so a busy card looks empty of old logs.', subject: 'mod_web', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_sniff_disables_logging', name: 'A CAN sniff session leaves logging off after reboot', severity: 'medium', where: 'webserver.cpp:2310-2311; sd_log.cpp:1621-1627', detail: 'Starting the sniffer without force=1 turns every log channel off and saves that to NVS, while the sniffer itself is RAM-only; after a reboot the unit logs nothing until the channels are turned back on.', subject: 'mod_web', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_718_extras_lock_miss', name: 'A missed lock take drops the 718 channels from a sample', severity: 'low', where: 'can_bus.cpp:625-632; ble_racecapture.cpp:694-698; racecapture.cpp:308-309', detail: 'can_getPorsche718Extra takes dataMutex separately with 5 ms; on a miss the sample treats the car as non-718, so TPS falls back to a NaN field and all 21 718-only channels vanish from that sample. The extras are also not taken at the same instant as the CAN snapshot.', subject: 'mod_can', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_rc_ap_unserviced', name: 'RaceCapture on the device AP is never serviced', severity: 'low', where: 'RCX_RTK_Datalogger.ino:164-168, 208-229', detail: 'The server starts in AP-only mode, but racecapture_loop() runs only when a station link is up, so an app joined to the device AP never gets data.', subject: 'mod_main', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_notify_blocks_loop', name: 'BLE notify() can block loop() for about 10 ms', severity: 'low', where: 'ble_racecapture.cpp:466-484; NimBLECharacteristic::sendValue', detail: 'NimBLE waits up to 10 x 1 ms for a free buffer before failing a notification, inside the loop task, which exceeds the 5 ms epoch rule under BLE congestion.', subject: 'mod_ble', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_nus_uuid_version', name: 'NUS may not be advertised, depending on the NimBLE version', severity: 'check', where: 'ble_racecapture.cpp:1003-1008', detail: 'The NUS UUID does not fit the primary advertising packet. NimBLE 2.2 and later puts it in the scan response; NimBLE 2.1.x drops it, because enableScanResponse(true) is called after addServiceUUID. The installed version is recorded only as 2.x.', subject: 'mod_ble', status: 'resolved on the assumed NimBLE 2.5.1: NUS goes in the scan response; reopen if an older 2.1.x library is found', source: 'code'})
CREATE (:Defect {id: 'dfx_thermal_sensor_silent', name: 'Thermal throttling silently off if the die sensor fails', severity: 'low', where: 'RCX_RTK_Datalogger.ino:421-435', detail: 'If the 50-125 °C temperature sensor fails to install, every reading is NAN for the rest of the boot and the gates hold their state, so no throttling happens and nothing is logged.', subject: 'mod_main', status: 'open', source: 'code'})
CREATE (:Defect {id: 'dfx_watermark_null_handle', name: 'Watermark print shows the loop stack under a missing task label', severity: 'info', where: 'RCX_RTK_Datalogger.ino:627-633', detail: 'When the CAN or SDLog task does not exist, xTaskGetHandle returns NULL and uxTaskGetStackHighWaterMark(NULL) reports the calling loop task instead.', subject: 'mod_main', status: 'open', source: 'code'})
MATCH (a {id: 'dfx_ntrip_write_unbounded'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_ap_still_beaconing'}), (b {id: 'mod_wifi'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_probe_heap_gated'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_ap_only_idle_lock'}), (b {id: 'mod_wifi'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_readback_flushes_nmea'}), (b {id: 'mod_gnss'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_dbc_upload_race'}), (b {id: 'mod_dbc_store'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_export_invalid_names'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_export_degraded_ready'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_deleteold_busy_zero'}), (b {id: 'mod_web'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_sniff_disables_logging'}), (b {id: 'mod_web'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_718_extras_lock_miss'}), (b {id: 'mod_can'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_rc_ap_unserviced'}), (b {id: 'mod_main'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_notify_blocks_loop'}), (b {id: 'mod_ble'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_nus_uuid_version'}), (b {id: 'mod_ble'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_thermal_sensor_silent'}), (b {id: 'mod_main'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_watermark_null_handle'}), (b {id: 'mod_main'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_ntrip_write_unbounded'}), (b {id: 'fm_ntrip_gga_blocking_tx'}) CREATE (a)-[:RISKS_REPEATING {}]->(b)
MATCH (a {id: 'dfx_ap_still_beaconing'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:MAY_AMPLIFY {contender: '(b) rover-side coexistence: beacons on channel 1 every 100 ms'}]->(b)
MATCH (a {id: 'dfx_probe_heap_gated'}), (b {id: 'fm_scan_gate'}) CREATE (a)-[:AMPLIFIED_BY {}]->(b)
MATCH (a {id: 'dfx_readback_flushes_nmea'}), (b {id: 'flow_nmea'}) CREATE (a)-[:DEGRADES {}]->(b)
MATCH (a {id: 'dfx_notify_blocks_loop'}), (b {id: 'rule_epoch_5ms'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'dfx_ap_only_idle_lock'}), (b {id: 'rule_no_unbounded_wait'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'lever_ap_netif'}), (b {id: 'dfx_ap_still_beaconing'}) CREATE (a)-[:RESOLVES {}]->(b)

// ─────────────────────────────────────────────────────────────────────────────
// 23. Velocity validity: how a receiver-invalid speed/heading reaches every output
// ─────────────────────────────────────────────────────────────────────────────
// The snapshot flow carries speedKnots and headingDeg but no velocity-valid field.
// Every consumer gates speed and heading on position validity (g.valid) alone.
CREATE (:FailureMode {id: 'fm_velocity_null_published_fresh', name: 'Receiver-invalid speed and heading published as fresh', symptom: 'Speed and heading repeat exactly on consecutive rows while position advances every row; only under RTK; rises with speed (30-45 % of moving rows on the 2026-09-09 Lincoln runs, 60-80 mph worst); episodes last 1 to 19 epochs; spd_age_ms reads a few ms throughout.', root_cause: 'The LG290P sends an RMC whose SOG and COG do not advance: null, which the protocol specification RMC table defines for an invalid velocity, or a repeated value; the two are not yet distinguished. TinyGPS++ skips empty terms but commits speed and course on every checksum-valid RMC whose status is A, so the previous value is committed again with isUpdated() true. gnss.cpp uses isUpdated() as its only freshness test, so velUpdateMillis advances and spd_age reads fresh, and the comment there states the opposite premise. BLE, RaceCapture TCP, the LCD and the SD row carry the repeated value as valid, because none has a velocity-valid signal to test.', proof: 'Host test of TinyGPS++ 1.0.3a: an RMC with empty SOG and COG after one carrying 52.143 kn and 87.25 deg returns speed.isUpdated() = 1 and course.isUpdated() = 1 with 52.14 kn and 87.25 deg.', side_effect: 'TinyGPS++ parseDecimal keeps two decimal places, so speed resolves to 0.01 kn although PQTMCFGNMEADP requests three.', status: 'open', source: 'code+measured', cite: 'gnss.cpp:1255-1277; TinyGPS++.cpp endOfTermHandler (term[0] guard, sentence-level commit); ble_racecapture.cpp:644,662-663; racecapture.cpp:290-291; display.cpp:558,657; sd_log.cpp:309-311'})
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'lib_tinygpsplus'}) CREATE (a)-[:ARISES_FROM {site: 'empty terms are skipped; speed and course commit per sentence, not per field'}]->(b)
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'mod_gnss'}) CREATE (a)-[:ARISES_FROM {site: 'gnss.cpp:1255-1277 treats isUpdated() as field freshness'}]->(b)
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'mod_ble'}) CREATE (a)-[:PRESENTS_IN {note: 'speed and heading gated on g.valid only'}]->(b)
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'mod_racecapture'}) CREATE (a)-[:PRESENTS_IN {note: 'speed and heading gated on g.valid only'}]->(b)
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:PRESENTS_IN {note: 'spd_age_ms reads fresh'}]->(b)
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'mod_display'}) CREATE (a)-[:PRESENTS_IN {note: 'speed shown and used by the cone-mode stillness guard'}]->(b)
MATCH (a {id: 'fm_velocity_null_published_fresh'}), (b {id: 'flow_snapshot'}) CREATE (a)-[:PRESENTS_IN {note: 'no velocity-valid field in the snapshot'}]->(b)
CREATE (:Hypothesis {id: 'hyp_lg290p_velocity_invalid_under_rtk', statement: 'The LG290P declares velocity invalid on some epochs while its RTK solution is being updated from fresh corrections at speed.', evidence: 'RTK is necessary (speed-matched 45-65 mph: 0.00 % at rtk_type 0, 11.08 % at rtk_type 2); rises with speed; higher with diff_age 1-3 s than above 10 s; about twice as frequent within 2 s of an rtk_type change; episodes end within one second (19 epochs).', status: 'UNCONFIRMED: no raw RMC line has been captured during an episode', test: 'Count RMC lines with an empty SOG or COG field in the 5 s wire line; a count near the number of repeated rows confirms null output rather than a held value.', source: 'measured'})
MATCH (a {id: 'hyp_lg290p_velocity_invalid_under_rtk'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'ext_lg290p'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:TRIGGERS {condition: 'null SOG and COG in RMC'}]->(b)
MATCH (a {id: 'gap_raw_byte_visibility'}), (b {id: 'hyp_lg290p_velocity_invalid_under_rtk'}) CREATE (a)-[:BLOCKS {note: 'RMC is parsed and discarded, so null fields have never been observed directly'}]->(b)

// Alternative causes checked against the code path. A row repeats speed and
// heading while position advances only if its epoch was published without a
// new RMC (watchdog: spd_age at or above 45 ms) or the new RMC carried the same
// SOG and COG. Every ESP32-side mechanism below either loses or delays whole
// lines, which the field signature excludes.
CREATE (:Hypothesis {id: 'hyp_alt_uart_loss', statement: 'RMC lost or corrupted on UART1 (FIFO or ring overrun), so the row is completed by the epoch watchdog with the previous velocity.', status: 'RULED OUT: frozen rows carry spd_age of a few ms, the same as clean rows, so a checksum-valid RMC with status A was applied just before each; the sat CSV shows zero checksum failures in the same sessions, and missing epochs stay near 0.1 %.', source: 'code+measured'})
CREATE (:Hypothesis {id: 'hyp_alt_loop_backlog', statement: 'A loop() stall builds an RX backlog and the drain parses several epochs in one pass.', status: 'RULED OUT as the freeze: a backlog delays whole lines, so a row can miss an epoch or pair position N+1 with the speed of epoch N, but consecutive rows still come from different RMCs and never repeat a velocity. It remains a one-epoch misalignment that spd_age cannot see.', source: 'code'})
CREATE (:Hypothesis {id: 'hyp_alt_snapshot_race', statement: 'Speed and position are copied from gps at different moments.', status: 'RULED OUT: the SD row, the BLE sample and the TCP line each copy gps once under dataMutex.', source: 'code'})
CREATE (:Hypothesis {id: 'hyp_alt_ble_fallback', statement: 'The BLE 100 ms fallback resends the last sample when no epoch completes.', status: 'RULED OUT as the freeze: a fallback sample repeats position as well, and the freeze is in the SD log, which has no fallback.', source: 'code'})
CREATE (:Hypothesis {id: 'hyp_null_vs_held', statement: 'The LG290P sends SOG and COG empty rather than repeating the last value.', status: 'UNTESTED: decides whether an empty-field check can detect the condition at all; needs an empty-field count or a raw RMC capture.', source: 'code'})
CREATE (:Hypothesis {id: 'hyp_module_load', statement: 'Our configuration drives the receiver past its compute budget: 20 Hz with six constellations, RTK and the NMEA message set, so it skips velocity on some epochs while updating RTK.', status: 'UNTESTED: compare the freeze rate at 10 Hz and 20 Hz on the same drive; if it falls sharply the cause is our configuration, not a fixed receiver limitation.', source: 'code'})
CREATE (:Defect {id: 'dfx_rtcm_write_blocks_loop', name: 'RTCM writes hold the UART1 lock and stall loop()', severity: 'medium', where: 'ntrip.cpp:1263-1269; gnss.cpp:864 (RX buffer set, TX buffer left at 0); esp32-hal-uart.c uartWriteBuf, uartAvailable, uartRead', detail: 'With no TX buffer, uart_write_bytes returns only after every byte has entered the 128 B FIFO, and uartWriteBuf holds uart->lock throughout: about 11 ms per 512 B chunk at 460800 baud, up to 8 chunks per pass. loop() takes the same lock in serial.available() and serial.read(), so it stalls for up to one chunk at a time whenever corrections are arriving. RX bytes wait in the 4 KB ring, so nothing is lost. It coincides with fresh corrections, which makes it a confounder for the velocity freeze, not its cause.', fix: 'gpsSerial.setTxBufferSize() before begin(), so a write returns once the bytes are copied into the driver ring.', subject: 'mod_ntrip', status: 'open', source: 'code'})
MATCH (a {id: 'dfx_rtcm_write_blocks_loop'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:ARISES_FROM {}]->(b)
MATCH (a {id: 'dfx_rtcm_write_blocks_loop'}), (b {id: 'rule_epoch_5ms'}) CREATE (a)-[:VIOLATES {}]->(b)
MATCH (a {id: 'dfx_rtcm_write_blocks_loop'}), (b {id: 'flow_nmea'}) CREATE (a)-[:DEGRADES {note: 'delays parsing; no loss'}]->(b)
MATCH (a {id: 'dfx_rtcm_write_blocks_loop'}), (b {id: 'hyp_alt_loop_backlog'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_alt_uart_loss'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_alt_loop_backlog'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_alt_snapshot_race'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_alt_ble_fallback'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_null_vs_held'}), (b {id: 'fm_velocity_null_published_fresh'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)
MATCH (a {id: 'hyp_module_load'}), (b {id: 'hyp_lg290p_velocity_invalid_under_rtk'}) CREATE (a)-[:PROPOSED_DRIVER_OF {}]->(b)

// Worked queries:
//   Which tasks can write a piece of state, and under what lock?
//     MATCH (t:Task)-[w:WRITES]->(s:SharedState) RETURN s.name, t.name, s.protection
//   What does the web UI reach into?
//     MATCH (e:Endpoint)-[:INVOKES]->(m:Module) RETURN e.method, e.path, m.file
//   Everything that runs on core 0:
//     MATCH (m:Module)-[:RUNS_IN]->(t:Task)-[:RUNS_ON]->(:Core {id: 'core0'}) RETURN m.file, t.name
