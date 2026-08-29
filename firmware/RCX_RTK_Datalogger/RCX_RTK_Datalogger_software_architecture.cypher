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


// ─────────────────────────────────────────────────────────────────────────────
// 1. System and execution substrate
// ─────────────────────────────────────────────────────────────────────────────

CREATE (sys:SoftwareSystem {id: 'rcx_fw', name: 'RCX RTK Datalogger firmware', platform: 'ESP32-S3 / Arduino-ESP32 on ESP-IDF', sketch: 'RCX_RTK_Datalogger.ino', binding_constraint: 'internal SRAM', notes: 'Motorsport datalogger: RTK GNSS, vehicle CAN, IMU, SD logging, BLE telemetry to SoloStorm, and a three-page web configuration UI.'})

CREATE (core0:Core {id: 'core0', name: 'ESP32-S3 Core 0', role: 'protocol and I/O core', hosts: 'GNSS/CAN/SD/WiFi/NTRIP/display/async_tcp', utilisation_status: 'not measured; no starvation observed', source: 'measured'})
CREATE (core1:Core {id: 'core1', name: 'ESP32-S3 Core 1', role: 'application core', hosts: 'Arduino loop() and BLE frame assembly', utilisation_status: 'adequate', source: 'measured'})


// ─────────────────────────────────────────────────────────────────────────────
// 2. Resources — what is actually scarce
// ─────────────────────────────────────────────────────────────────────────────

CREATE (r_isram:Resource {id: 'internal_sram', name: 'Internal SRAM heap', caps: 'MALLOC_CAP_INTERNAL|MALLOC_CAP_8BIT', size_at_boot_bytes: 247224, steady_state_free_bytes: 29732, largest_free_block_bytes: 14324, scarcity: 'binding', relocatable: false, source: 'measured', notes: 'The only resource this design is short of. Task stacks, lwIP pbufs and SDMMC DMA buffers cannot live anywhere else.'})
CREATE (r_dma:Resource {id: 'dma_pool', name: 'DMA-capable internal memory', caps: 'MALLOC_CAP_DMA', scarcity: 'binding', shares_pool_with: 'internal_sram', observed_low_bytes: 440, observed_low_largest_block_bytes: 44, source: 'measured', notes: 'Not a separate pool so much as a stricter view of the same one. SDMMC transfer buffers come from here.'})
CREATE (r_psram:Resource {id: 'psram', name: 'External PSRAM', size_bytes: 8388608, used_bytes: 104000, utilisation_pct: 1.3, scarcity: 'abundant', relocatable: true, source: 'measured', notes: 'Design axiom: anything that can live here, does.'})
CREATE (r_flash:Resource {id: 'flash_rodata', name: 'Flash / rodata', scarcity: 'abundant', used_by_web_assets_bytes: 60000, source: 'derived', notes: 'Pages and shared assets are served straight out of memory-mapped flash with no RAM copy.'})
CREATE (r_uart1:Resource {id: 'uart1', name: 'UART1 to LG290P GNSS', baud: 460800, framing: '8N1', line_rate_bytes_per_s: 46080, measured_rx_bytes_per_s: 8700, utilisation_pct: 19, scarcity: 'abundant', source: 'derived', notes: 'RX load derived from the wire/5s sentence counters at a typical 70 B NMEA sentence.'})
CREATE (r_uart0:Resource {id: 'uart0', name: 'UART0 / USB CDC console', role: 'serial diagnostics', scarcity: 'abundant', hazard: 'When DEBUG_SERIAL_TO_SD is enabled every print becomes an SD write, so a diagnostic burst can feed back into the SD path it is diagnosing.', source: 'code'})
CREATE (r_sdbus:Resource {id: 'sd_bus', name: 'SDMMC bus and FatFs volume', scarcity: 'bandwidth abundant, latency contended', sustained_write_class: 'MB/s', measured_aggregate_write_bytes_per_s: 5000, source: 'derived', notes: 'Bandwidth is never the constraint. Mutex hold time and DMA buffer availability are.'})
CREATE (r_twai:Resource {id: 'twai', name: 'TWAI / CAN controller', bitrate: 500000, mode: 'listen-only', scarcity: 'abundant', source: 'code'})

MATCH (a {id: 'dma_pool'}), (b {id: 'internal_sram'}) CREATE (a)-[:SUBSET_OF {notes: 'Exhausting one exhausts the other.'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 3. Modules — translation units and what they own
// ─────────────────────────────────────────────────────────────────────────────

CREATE (m_main:Module {id: 'mod_main', file: 'RCX_RTK_Datalogger.ino', role: 'setup ordering, loop(), task creation, heap and stack diagnostics', owns: 'dataMutex, bring-up sequence'})
CREATE (m_gnss:Module {id: 'mod_gnss', file: 'gnss.cpp', role: 'LG290P configuration and NMEA/PQTM parsing', hazard: 'hot-start contract — see file header banner; wrong command order silently reverts config'})
CREATE (m_can:Module {id: 'mod_can', file: 'can_bus.cpp', role: 'TWAI receive, vehicle profile fingerprinting, sniffer snapshot'})
CREATE (m_imu:Module {id: 'mod_imu', file: 'imu.cpp', role: 'QMI8658 configuration, calibration, sampling'})
CREATE (m_sdlog:Module {id: 'mod_sdlog', file: 'sd_log.cpp', role: 'owns ALL SD access: queues, file lifecycle, directory snapshot, tar export', owns: 'sdMutex, dirSnapLock'})
CREATE (m_web:Module {id: 'mod_web', file: 'webserver.cpp', role: 'three-page UI, JSON endpoints, downloads, chunked write-veto senders'})
CREATE (m_wifi:Module {id: 'mod_wifi', file: 'wifi_mgr.cpp', role: 'NVS network list, AP/STA lifecycle, device identity', hazard: 'ordering contract — re-entering WiFi.mode() rebuilds the lwIP netif'})
CREATE (m_ntrip:Module {id: 'mod_ntrip', file: 'ntrip.cpp', role: 'caster selection, source-table scan, RTCM relay to the GNSS module'})
CREATE (m_ble:Module {id: 'mod_ble', file: 'ble_racecapture.cpp', role: 'NimBLE GATT server, RaceCapture wire format, 20 Hz frame assembly'})
CREATE (m_rc:Module {id: 'mod_racecapture', file: 'racecapture.cpp', role: 'RaceCapture protocol over TCP 7223'})
CREATE (m_display:Module {id: 'mod_display', file: 'display.cpp', role: 'TFT rendering; sprite lives in PSRAM'})
CREATE (m_dbcs:Module {id: 'mod_dbc_store', file: 'dbc_store.cpp', role: 'DBC file storage and the task that owns DBC SD access'})
CREATE (m_dbcp:Module {id: 'mod_dbc_parse', file: 'dbc_parse.cpp', role: 'streaming BO_/SG_ parser and import audit'})
CREATE (m_debug:Module {id: 'mod_debug_log', file: 'debug_log.cpp', role: 'serial-to-SD tee, gated by DEBUG_SERIAL_TO_SD'})
CREATE (m_thermal:Module {id: 'mod_thermal', file: 'thermal.cpp', role: 'die and IMU temperature thresholds'})
CREATE (m_config:Module {id: 'mod_config', file: 'config.cpp / config.h', role: 'compile-time configuration and caster defaults', hazard: 'contains rtk2go credentials — scrub before public release'})


// ─────────────────────────────────────────────────────────────────────────────
// 4. Tasks — stacks are the largest controllable internal-SRAM line
// ─────────────────────────────────────────────────────────────────────────────

CREATE (t_loop:Task {id: 'task_loop', name: 'loop', core: 'core1', priority: 1, stack_bytes: 8192, stack_high_water_free_bytes: 5784, stack_peak_use_bytes: 2408, owner: 'Arduino core', instrumented: true, source: 'measured'})
CREATE (t_sdlog:Task {id: 'task_sdlog', name: 'SDLog', core: 'core0', priority: 1, stack_bytes: 12288, stack_high_water_free_bytes: 6976, stack_peak_use_bytes: 5312, instrumented: true, trim_verdict: 'leave — FatFs paths run deep', source: 'measured'})
CREATE (t_ntrip:Task {id: 'task_wifintrip', name: 'WiFiNTRIP', core: 'core0', priority: 1, stack_bytes: 12288, stack_high_water_free_bytes: 8684, stack_peak_use_bytes: 3604, instrumented: true, trim_verdict: 'leave — deep path (source-table scan) never exercised in the measured window', source: 'measured'})
CREATE (t_display:Task {id: 'task_display', name: 'Display', core: 'core0', priority: 1, stack_bytes: 6144, stack_high_water_free_bytes: 5624, stack_peak_use_bytes: 2568, stack_known_bad_floor_bytes: 4096, instrumented: true, source: 'measured', notes: 'High-water measured while allocated 8192; trimmed to 6144 on that basis.'})
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

CREATE (s00:BringUpStage {id: 'stage_boot',        seq: 0,  label: 'boot',                free_after_bytes: 247224, cost_bytes: 0,     largest_block_bytes: 188404, source: 'measured'})
CREATE (s01:BringUpStage {id: 'stage_display',     seq: 1,  label: 'display init',         free_after_bytes: 246780, cost_bytes: 444,   source: 'measured'})
CREATE (s02:BringUpStage {id: 'stage_imu',         seq: 2,  label: 'imu init',             free_after_bytes: 245768, cost_bytes: 1012,  source: 'measured'})
CREATE (s03:BringUpStage {id: 'stage_gnss',        seq: 3,  label: 'gnss init',            free_after_bytes: 245496, cost_bytes: 272,   source: 'measured'})
CREATE (s04:BringUpStage {id: 'stage_ble',         seq: 4,  label: 'NimBLE init',          free_after_bytes: 179268, cost_bytes: 66228, rank: 1, source: 'measured', notes: 'Largest single consumer in the build.'})
CREATE (s05:BringUpStage {id: 'stage_can',         seq: 5,  label: 'CAN init + task',      free_after_bytes: 170024, cost_bytes: 9244,  source: 'measured'})
CREATE (s06:BringUpStage {id: 'stage_sdlog',       seq: 6,  label: 'SD log init + task',   free_after_bytes: 156560, cost_bytes: 13464, source: 'measured', notes: 'Queues are PSRAM; this is stack plus SDMMC driver.'})
CREATE (s07:BringUpStage {id: 'stage_webroutes',   seq: 7,  label: 'web routes registered',free_after_bytes: 145684, cost_bytes: 10876, source: 'measured'})
CREATE (s08:BringUpStage {id: 'stage_ntriptask',   seq: 8,  label: 'WiFiNTRIP task',       free_after_bytes: 132508, cost_bytes: 13176, source: 'measured'})
CREATE (s09:BringUpStage {id: 'stage_displaytask', seq: 9,  label: 'Display task',         free_after_bytes: 123428, cost_bytes: 9080,  source: 'measured'})
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


// ─────────────────────────────────────────────────────────────────────────────
// 6. Buffers and queues — the relocation story
// ─────────────────────────────────────────────────────────────────────────────

CREATE (q_gpscan:Queue {id: 'q_gps_can', name: 'GPS+CAN row queue', bytes: 28320, placement: 'psram', fallback: 'internal on PSRAM failure', producer_rate_hz: 20, source: 'measured'})
CREATE (q_imu:Queue    {id: 'q_imu',     name: 'IMU row queue',     bytes: 3360,  placement: 'psram', producer_rate_hz: 50, source: 'measured'})
CREATE (q_sat:Queue    {id: 'q_sat',     name: 'Satellite row queue', bytes: 2640, placement: 'psram', producer_rate_hz: null, source: 'measured'})
CREATE (q_canraw:Queue {id: 'q_canraw',  name: 'Raw CAN frame queue', bytes: 32768, placement: 'psram', item_bytes: 16, producer_rate: 'bus rate', source: 'measured', notes: 'Throughput-gated: the sniffer forces the typed channels off by default.'})

CREATE (b_dirsnap:Buffer {id: 'buf_dir_snapshot', name: 'Directory snapshot (double-buffered)', entries: 512, bytes_at_name_width_32: 36864, bytes_at_name_width_40: 45056, placement: 'psram', source: 'measured'})
CREATE (b_uartrx:Buffer  {id: 'buf_uart1_rx', name: 'UART1 RX ring', bytes: 4096, placement: 'internal', coverage_ms: 89, source: 'code', notes: 'Default is 256 B, which fills in ~5.5 ms at 460800. 4096 lets loop() stall ~89 ms without byte loss.'})
CREATE (b_tar:Buffer     {id: 'buf_tar_stage', name: 'Tar copy buffer', bytes: 4096, placement: 'psram', fallback: 'internal', source: 'code'})
CREATE (b_ack:Buffer     {id: 'buf_async_ack', name: 'ESPAsyncWebServer per-ack assembly buffer', bytes_typical: 5744, placement: 'internal', lifetime: 'one ack', owner: 'library', source: 'derived', notes: 'CRITICAL: malloc-ed at the size of the whole TCP send window BEFORE the filler callback is consulted. A filler that declines still pays this allocation.'})
CREATE (b_pbuf:Buffer    {id: 'buf_lwip_pbuf', name: 'lwIP PBUF_RAM segment', bytes: 'chunk + ~90', placement: 'internal', owner: 'lwIP', source: 'derived', notes: 'tcp_write returns ERR_MEM when this cannot be allocated; AsyncClient::write() then returns 0.'})
CREATE (b_sddma:Buffer   {id: 'buf_sdmmc_dma', name: 'SDMMC transfer buffer', bytes_approx: 4096, placement: 'dma', owner: 'ESP-IDF', source: 'derived', notes: 'Failure prints allocate_dma_buf: not enough mem, err=0x101.'})

MATCH (q:Queue), (r {id: 'psram'}) CREATE (q)-[:ALLOCATED_IN {bytes: q.bytes, source: 'measured'}]->(r)
MATCH (a {id: 'buf_dir_snapshot'}), (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 45056, source: 'derived'}]->(b)
MATCH (a {id: 'buf_tar_stage'}),    (b {id: 'psram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 4096, source: 'code'}]->(b)
MATCH (a {id: 'buf_uart1_rx'}),     (b {id: 'internal_sram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 4096, relocatable: false, source: 'code'}]->(b)
MATCH (a {id: 'buf_async_ack'}),    (b {id: 'internal_sram'}) CREATE (a)-[:ALLOCATED_IN {bytes: 5744, relocatable: false, transient: true, source: 'derived'}]->(b)
MATCH (a {id: 'buf_lwip_pbuf'}),    (b {id: 'internal_sram'}) CREATE (a)-[:ALLOCATED_IN {relocatable: false, transient: true, source: 'derived'}]->(b)
MATCH (a {id: 'buf_sdmmc_dma'}),    (b {id: 'dma_pool'})      CREATE (a)-[:ALLOCATED_IN {bytes: 4096, relocatable: false, transient: true, source: 'derived'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 7. Locks — hold budgets and the timeouts that enforce them
// ─────────────────────────────────────────────────────────────────────────────

CREATE (l_sd:Lock       {id: 'lock_sdmutex',   name: 'sdMutex', guards: 'SD bus / FatFs volume', hold_budget_periodic_ms: 20, hold_budget_user_ms: 200, acquire_timeouts_ms: [50, 200, 300, 500, 1000, 2000], enforcement: 'log writers wait only 50 ms then DROP the row — exceeding the hold budget costs data silently', source: 'code'})
CREATE (l_snap:Lock     {id: 'lock_dirsnap',   name: 'dirSnapLock', guards: 'published directory snapshot', acquire_timeouts_ms: [50, 100], hold_budget_ms: 10, source: 'code'})
CREATE (l_data:Lock     {id: 'lock_datamutex', name: 'dataMutex', guards: 'gps / status / can snapshot structs', hold_budget_ms: 1, acquire_timeouts_ms: [5, 10, 50], intended_scope: 'a memcpy of a snapshot struct and nothing else', source: 'code'})
CREATE (l_sniff:Lock    {id: 'lock_sniffmux',  name: 'sniffMux', guards: 'CAN sniffer snapshot table', source: 'code'})

MATCH (a {id: 'lock_sdmutex'}),   (b {id: 'sd_bus'})       CREATE (a)-[:GUARDS {}]->(b)
MATCH (a {id: 'lock_dirsnap'}),   (b {id: 'buf_dir_snapshot'}) CREATE (a)-[:GUARDS {}]->(b)

MATCH (a {id: 'task_sdlog'}),     (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'writer', timeout_ms: 50,   on_timeout: 'drop the row', source: 'code'}]->(b)
MATCH (a {id: 'task_sdlog'}),     (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'file lifecycle / dir scan slice', timeout_ms: 200, source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'batch delete / tar build', timeout_ms: 2000, hazard: 'blocking here starves every in-flight HTTP response', source: 'code'}]->(b)
MATCH (a {id: 'task_dbc'}),       (b {id: 'lock_sdmutex'}) CREATE (a)-[:ACQUIRES {role: 'DBC file access', source: 'code'}]->(b)
MATCH (a {id: 'task_loop'}),      (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'publish snapshots', timeout_ms: 5, source: 'code'}]->(b)
MATCH (a {id: 'task_can'}),       (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'publish CAN snapshot', timeout_ms: 5, source: 'code'}]->(b)
MATCH (a {id: 'task_async_tcp'}), (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: '/status copy-out', timeout_ms: 50, source: 'code'}]->(b)
MATCH (a {id: 'task_nimble'}),    (b {id: 'lock_datamutex'}) CREATE (a)-[:ACQUIRES {role: 'frame assembly', timeout_ms: -1, timeout_label: 'portMAX_DELAY', violates: 'rule_no_unbounded_wait', source: 'code'}]->(b)

MATCH (a {id: 'lock_sdmutex'}), (b {id: 'lock_dirsnap'})
CREATE (a)-[:MUST_NOT_NEST_WITH {reason: 'Nesting was introduced once when snapshot updates moved inside openFile(); fixed by moving them after the sdMutex release.', source: 'code'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 8. Data flows
// ─────────────────────────────────────────────────────────────────────────────

CREATE (f_nmea:DataFlow  {id: 'flow_nmea',   name: 'NMEA/PQTM sentences', transport: 'uart1', direction: 'LG290P -> ESP32', rate_hz: 20, bytes_per_s: 8700, source: 'derived'})
CREATE (f_rtcm:DataFlow  {id: 'flow_rtcm',   name: 'RTCM corrections',    transport: 'uart1', direction: 'ESP32 -> LG290P', origin: 'NTRIP caster', source: 'code'})
CREATE (f_ntrip:DataFlow {id: 'flow_ntrip',  name: 'NTRIP stream',        transport: 'tcp', direction: 'caster -> ESP32', source: 'code'})
CREATE (f_can:DataFlow   {id: 'flow_can',    name: 'Vehicle CAN frames',  transport: 'twai', direction: 'vehicle -> ESP32', mode: 'listen-only', rate: 'bus rate at 500 kbps', source: 'code'})
CREATE (f_snap:DataFlow  {id: 'flow_snapshot', name: 'gps/status/can snapshots', transport: 'shared memory under dataMutex', rate_hz: 20, source: 'code'})
CREATE (f_ble:DataFlow   {id: 'flow_ble',    name: 'RaceCapture BLE frames', transport: 'ble_gatt', direction: 'ESP32 -> SoloStorm', rate_hz: 20, contract: 'frames go out on schedule and must contain valid GNSS+CAN+IMU data', source: 'code'})
CREATE (f_rows:DataFlow  {id: 'flow_sd_rows', name: 'CSV rows', transport: 'PSRAM queues', direction: 'producers -> SDLog', source: 'code'})
CREATE (f_http:DataFlow  {id: 'flow_http',   name: 'HTTP responses', transport: 'tcp_80', hazard: 'each in-flight response holds an ack buffer plus an lwIP segment out of internal SRAM', source: 'derived'})

MATCH (a {id: 'mod_gnss'}),   (b {id: 'flow_nmea'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_ntrip'}),  (b {id: 'flow_rtcm'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_ntrip'}),  (b {id: 'flow_ntrip'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_can'}),    (b {id: 'flow_can'}) CREATE (a)-[:CONSUMES_FLOW {}]->(b)
MATCH (a {id: 'mod_gnss'}),   (b {id: 'flow_snapshot'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_can'}),    (b {id: 'flow_snapshot'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
MATCH (a {id: 'mod_imu'}),    (b {id: 'flow_snapshot'}) CREATE (a)-[:PRODUCES_FLOW {}]->(b)
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

CREATE (bud_pool:Budget    {id: 'budget_free_pool',   name: 'Free internal pool floor', resource: 'internal_sram', budget_bytes: 40960, measured_bytes: 29732, headroom_bytes: -11228, status: 'OVER', rationale: 'One web response (~11 KB) + one SDMMC DMA buffer (4 KB) + source-table scan worst case + fragmentation slack.', source: 'derived'})
CREATE (bud_stacks:Budget  {id: 'budget_task_stacks', name: 'All task stacks', resource: 'internal_sram', budget_bytes: 40960, measured_bytes: 45056, status: 'OVER', source: 'derived'})
CREATE (bud_net:Budget     {id: 'budget_net_stack',   name: 'WiFi + lwIP + async_tcp', resource: 'internal_sram', budget_bytes: 97280, measured_bytes: 93220, status: 'THIN', source: 'derived'})
CREATE (bud_ble:Budget     {id: 'budget_ble',         name: 'NimBLE', resource: 'internal_sram', budget_bytes: 67584, measured_bytes: 66228, status: 'FIXED COST', source: 'measured'})
CREATE (bud_uart:Budget    {id: 'budget_uart1_rx',    name: 'UART1 RX utilisation', resource: 'uart1', budget_pct: 60, measured_pct: 19, status: 'OK', spend_note: 'This is the one budget with room to spend — extra constellations or GSV rates come from here.', source: 'derived'})
CREATE (bud_ring:Budget    {id: 'budget_uart1_ring',  name: 'UART1 RX ring coverage', resource: 'uart1', budget_ms: 50, measured_ms: 89, status: 'OK', source: 'derived'})
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
CREATE (rule_floor:Rule    {id: 'rule_pool_floor',  text: 'The free internal pool has a floor. Work that would push below it is refused at the endpoint, not allowed to fail deep in a driver.'})
CREATE (rule_wait:Rule     {id: 'rule_no_unbounded_wait', text: 'No task waits indefinitely for a lock whose budgeted hold time is measured in milliseconds.'})

MATCH (a {id: 'task_async_tcp'}), (b {id: 'rule_async_clean'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'lock_sdmutex'}),   (b {id: 'rule_no_nesting'})  CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'budget_free_pool'}), (b {id: 'rule_pool_floor'}) CREATE (a)-[:GOVERNED_BY {}]->(b)
MATCH (a {id: 'task_nimble'}),    (b {id: 'rule_no_unbounded_wait'})
CREATE (a)-[:VIOLATES {detail: 'ble_racecapture.cpp takes dataMutex with portMAX_DELAY in two places, on the 20 Hz frame path.', status: 'open', source: 'code'}]->(b)


// ─────────────────────────────────────────────────────────────────────────────
// 12. Failure modes — what has actually gone wrong, and why
// ─────────────────────────────────────────────────────────────────────────────

CREATE (fm_hole:FailureMode {id: 'fm_response_hole', name: 'Response with a hole in the middle', symptom: 'Page cuts off partway then appears to restart; JSON endpoints return unparseable text and fields stay blank.', root_cause: 'AsyncClient::write() returns 0 on lwIP pbuf exhaustion, but both response classes advance _sentLength before or regardless of the write result, so the unwritten span is discarded silently.', status: 'fixed', fix: 'Chunked filler callbacks that return RESPONSE_TRY_AGAIN while the internal largest free block cannot support the write.', source: 'measured'})
CREATE (fm_livelock:FailureMode {id: 'fm_veto_livelock', name: 'Write-veto livelock', symptom: 'One page load, then a continuous sdmmc allocate_dma_buf 0x101 storm and a monotonic internal-heap slide from ~25 KB to ~2 KB that never recovers.', root_cause: 'AsyncAbstractResponse::_ack malloc-s a full send window BEFORE consulting the filler, so an unbounded veto repeats a ~5.7 KB allocate/free at poll cadence forever, pinning the pool SDMMC needs.', status: 'fix delivered, awaiting field test', fix: 'Bounded declines with forced progress, reserve lowered, chunk cap removed.', source: 'measured'})
CREATE (fm_starve:FailureMode {id: 'fm_batch_delete_starve', name: 'Batch delete starves in-flight responses', symptom: 'Corrupted page loads correlating with delete bursts.', root_cause: 'The remove loop runs on async_tcp and fed the watchdog without ever yielding.', status: 'fixed', fix: 'Periodic vTaskDelay in all three batch loops.', source: 'measured'})
CREATE (fm_scan:FailureMode {id: 'fm_scan_gate', name: 'Source-table scan permanently gated', symptom: 'rtk2go scan skipped; internal heap low (29xxx B free) against a 30000 B guard.', root_cause: 'Steady-state internal free sits ~700 B below a guard threshold that was itself an estimate.', status: 'open', fix: 'Stack trims returned ~4 KB; guard to be recalibrated from the new scan measurement.', source: 'measured'})
CREATE (fm_dmawrite:FailureMode {id: 'fm_sd_dma_fail', name: 'SD write fails with 0x101', symptom: 'sdmmc_cmd allocate_dma_buf not enough mem; rows dropped.', root_cause: 'DMA-capable pool exhausted by concurrent web responses.', status: 'mitigated', fix: 'printlnRetry (3 attempts, 5 ms apart) plus the concurrency and veto work.', source: 'measured'})
CREATE (fm_netif:FailureMode {id: 'fm_netif_orphan', name: 'Web server socket orphaned from the live interface', symptom: 'WiFi connects and NTRIP flows but the web page never responds.', root_cause: 'The listening socket was bound against an interface that a later WiFi.mode()/reconnect rebuilt.', status: 'fixed', fix: 'Rebind on the interface that actually connected.', source: 'measured', notes: 'This is the precedent that makes lever_ap_netif risky.'})
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

CREATE (lev_ap:Lever    {id: 'lever_ap_netif', name: 'Retire the AP netif properly', rank: 1, recovery_bytes_estimate: '5000-15000', confidence: 'low', risk: 'high', status: 'tabled by user', detail: 'apStop() calls WiFi.softAPdisconnect(false), which drops the interface but leaves the mode at WIFI_AP_STA, so the AP netif and DHCP server stay allocated for the rest of the boot. Moving to WIFI_STA would release them.', risk_detail: 'Re-entering WiFi.mode() rebuilds the lwIP netif — the precedent for fm_netif_orphan.'})
CREATE (lev_async:Lever {id: 'lever_async_stack', name: 'Instrument then trim async_tcp stack', rank: 2, recovery_bytes_estimate: '0-4096', risk: 'low to measure, moderate to trim', status: 'open'})
CREATE (lev_ble:Lever   {id: 'lever_nimble', name: 'Reduce NimBLE footprint', rank: 3, recovery_bytes_estimate: 'potentially large', risk: 'high', status: 'open', detail: 'Advertising payload is already over-length at boot (Cannot add UUID). Trimming the GATT table or lowering the configured connection count would help.', risk_detail: 'Touches the field-verified BLE contract, which is the reason the rover exists.'})
CREATE (lev_static:Lever {id: 'lever_static_to_psram', name: 'Move remaining static tables to PSRAM', rank: 4, recovery_bytes_estimate: 4000, risk: 'low', status: 'open', detail: 'exportSelectNames, pageNames, snapNames.', risk_detail: 'Adds allocation-failure paths where none exist today.'})
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

CREATE (fm_anchor:FailureMode {id: 'fm_ble_master_anchor_skipping', name: 'BLE master skips connection anchors under its own radio contention', symptom: '75.4% SoloStorm sample loss; data-bearing events every 150-900 ms against a granted 15 ms interval; stalls to 3.3 s; two sessions ended in mid-motion disconnects.', root_cause: 'Anchor starvation is measured; WHICH radio steals the anchors is not. Corrected 08-22 config (per operator): phone = hotspot, tablet = SoloStorm BLE central — the master was NOT the hotspot. Open contenders, all producing this identical fingerprint: (a) tablet-side WiFi activity (joined to the hotspot and/or Android periodic scans) making the master skip anchors; (b) rover-side coexistence denying the slave its anchors (a missed slave anchor also carries no data); (c) near-field receiver desense if the hotspot phone sits beside the tablet or rover; (d) general home-channel 2.4 GHz congestion deferring everything. The firmware amplifies whichever it is: an effective ~2-sample queue plus the 25 ms abandon-on-congestion policy discards everything a gap outlasts.', environment_factor: 'REFUTED as environmental. The 2026-08-23 race day produced both modes on the same device at the same site within the same afternoon (six runs 19.66-19.87 Hz, one run 5.63 Hz), which no site-level RF explanation survives. The earlier 0.86%-vs-75% comparison is additionally unsafe: those baselines predate delivered-truth counters AND come from a period when the GNSS UART harness was intermittently faulty (see fm_gnss_uart_harness_intermittent), so the produced-rate side of the ratio was not sound.', status: 'mechanism measured (anchor starvation); TRIGGER UNDOCUMENTED — endpoint attribution open, environmental attribution refuted; exit is a connection cycle', source: 'measured'})

CREATE (fm_superv:FailureMode {id: 'fm_ble_supervision_400ms', name: 'Peripheral-requested 400 ms supervision timeout', symptom: 'Would convert every sub-second anchor gap into a hard disconnect on any central that honors the request.', root_cause: 'onSubscribe requested conn params with timeout 40 (400 ms). Field data shows the link surviving 3.3 s gaps, so the observed central kept its own multi-second timeout — the hazard is latent, armed against stricter centrals.', status: 'fixed — request is 500 (5 s)', source: 'measured'})

CREATE (fm_gga:FailureMode {id: 'fm_ntrip_gga_blocking_tx', name: 'NTRIP GGA keep-alive as unbounded blocking uplink write', symptom: 'One dropped BLE frame every ~11 s (NCCAR, July 2026): 10 s keep-alive timer + ~1 s blocking TCP send stall, each stall a coex-arbiter BLE anchor denial.', root_cause: 'ntripClient historically had no socket timeout while the scan clients did; print() of the GGA blocked on a congested hotspot uplink and the timestamp advanced only after return, stretching the period to the observed 11 s.', status: 'fixed — bounded by NTRIP_CLIENT_IO_TIMEOUT_MS at connect, blocked-send instrumentation in place, timestamp advances only on success; 2026-08-22 stalls show zero 10-11 s periodicity, consistent with the fix being effective', source: 'measured'})

CREATE (hyp_rto:Hypothesis {id: 'hyp_uplink_rto_bursts', statement: 'The dominant 2026-08-22 anchor-denial driver is lwIP TCP retransmission bursts on the NTRIP uplink (GGA keep-alives / ACKs stuck against a cell-backhauled hotspot): RTO backoff timers fire aperiodically every 0.5-3 s, matching the measured 1-2 s stall recurrence, and no client-side socket timeout bounds radio occupancy from MAC/TCP retries. Explains uniform loss across RF-quiet and RF-dense sections.', status: 'UNCONFIRMED — uplink health is unlogged (only downlink rtcm_bytes exists); needs TX-side instrumentation or a packet capture.', source: 'derived'})

MATCH (a {id: 'fm_ntrip_gga_blocking_tx'}), (b {id: 'mod_ntrip'}) CREATE (a)-[:ARISES_FROM {site: 'GGA keep-alive send path', source: 'code'}]->(b)
MATCH (a {id: 'hyp_uplink_rto_bursts'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:PROPOSED_DRIVER_OF {evidence: 'aperiodic 1-2 s stall recurrence (period folds all below noise floor), RF-environment independence', source: 'measured'}]->(b)

CREATE (hyp_size:Hypothesis {id: 'hyp_sample_size_amplifier', statement: 'Samples grew ~127 B (pre-DBC) to ~277 B (new DBC, 42 values), crossing the 251 B DLE LL-PDU boundary from 1 to 2 fragments per notification — possibly amplifying loss under sparse master grants.', status: 'UNCONFIRMED — CAN was fully populated on every real row of the 08-22 drive, so no within-drive size contrast exists; needs a controlled contrast or the 08-21 export.', source: 'derived'})

CREATE (inst_params:Instrument {id: 'inst_ble_conn_params', name: 'Granted conn-param evidence at subscribe', signals: 'interval (1.25 ms units), latency, supervision timeout, MTU from NimBLEConnInfo', outputs: 'serial line at CCCD subscribe', source: 'code', notes: 'The granted interval was never observable in field logs before; the 08-22 value had to be recovered by lattice-fitting arrival times.'})

CREATE (inst_diffage:Instrument {id: 'inst_diff_age_column', name: 'diff_age GPS-log column', signals: 'GGA field-13 correction age (s), -1 = none in use', outputs: 'gps CSV per-row', source: 'code', notes: 'Direct correction-staleness record; previously parsed and displayed but never logged.'})

MATCH (a {id: 'fm_ble_master_anchor_skipping'}), (b {id: 'mod_ble'})  CREATE (a)-[:AMPLIFIED_BY {mechanism: '25 ms abandon deadline + ~2-sample effective queue convert master scheduling gaps into permanent loss; deadline is DELIBERATE (bounds loop() blocking) — any replacement must preserve that bound', source: 'code'}]->(b)
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

CREATE (buf_tx_slot:Buffer {id: 'buf_tx_slot', name: 'BLE TX message slot', where: 'PSRAM', bytes: 6144, depth_messages: 1, policy: 'newest-wins: an UNSTARTED sample is replaced by the next sample; a started message is never abandoned except on disconnect', rationale: 'Preserves the deliberate no-stale-backlog intent of the 25 ms abandon deadline structurally (at most one sample of latency can exist) while changing its failure mode from discard to supersede. SoloStorm keys rows to the sample own Utc, so late delivery is nearly free and LOSS is what hurts.', source: 'code'})

CREATE (mod_tx_stage:Module {id: 'mod_tx_stage', name: 'txStage()', file: 'ble_racecapture.cpp', role: 'Admission into the slot. critical=true (meta/version/ACK) takes an idle slot or displaces an unstarted sample; a sample takes an idle slot or replaces an unstarted sample; otherwise the sample is dropped and counted.', source: 'code'})

CREATE (mod_tx_pump:Module {id: 'mod_tx_pump', name: 'txPump()', file: 'ble_racecapture.cpp', role: 'Advances the in-flight message by exactly ONE notification per call, no sooner than 4 ms after the previous one; fails fast on congestion and retries next pass; stamps the meta guard at completion.', source: 'code'})

CREATE (rule_tx_wire_envelope:Rule {id: 'rule_tx_wire_envelope', name: 'The BLE wire envelope is client-facing contract', statement: 'Multi-chunk messages MUST go out as one notification at a time with >=4 ms spacing; a meta must be followed by ~15 ms of quiet; no sample may be wedged between a getMeta and its meta, nor around a meta in flight. These were established by trial and error against SoloStorm in June 2026 and are properties of the CLIENT reassembly, not internal timing preferences.', enforcement: 'txPump serialization clock + metaQuietMs + the tick sample barrier', source: 'code+measured'})

CREATE (rule_meta_guard_completion:Rule {id: 'rule_meta_guard_completion', name: 'The getMeta guard measures receipt, not intent', statement: 'bleLastMetaMs is stamped when a meta FINISHES on the wire, never when it is staged. An unfinished meta leaves the guard open so the next getMeta is answered immediately — the bench-proven improving shape. Stamping at staging lets a meta that never reached the client silence the client retry that would have fixed it.', source: 'code'})

CREATE (fm_ble_v3_wire_envelope_removal:FailureMode {id: 'fm_ble_v3_wire_envelope_removal', name: 'SoloStorm handshake spin after wire-envelope removal', symptom: 'MTU 512 negotiated, one clean subscribe, setTelemetry accepted, t=0 sample transmitted — then an endless loop of {"getMeta":null} answered by a complete meta every ~250 ms, forever. Streaming never starts.', root_cause: 'The engine sent up to 3 chunks back-to-back per pass with no inter-chunk spacing and no post-meta quiet, and could wedge a sample between a getMeta and its meta. Byte content and chunk boundaries were identical to the proven sender; only the envelope changed. Every complete meta was rejected by the client while single-chunk samples were accepted.', elimination_chain: '(1) MTU-23/ACK starvation dead — MTU 512. (2) Subscription flap dead — exactly one TX SUBSCRIBED, no re-subscribe. (3) t=0-sample-loss dead — that sample transmitted, and it carries no embedded meta. (4) Staging failure dead — a failed stage keeps pendingMeta set and would reprint at loop rate (~300/s); observed 4/s equals the 250 ms guard exactly, so every stage succeeded, so the previous meta had fully drained. (5) Pump gate trips dead — conn/subscribe/pTxChar all stable. Therefore complete byte-identical metas were handed to the stack every 250 ms and every one was rejected.', library_acquitted: 'NimBLE 2.x NimBLECharacteristic::notify() deep-copies (auto value{m_value}) and sendValue() builds a fresh mbuf via ble_hs_mbuf_from_flat at call time, so back-to-back setValue+notify cannot tear a queued chunk. Verified against upstream source, not memory.', status: 'convicted by exhaustive construction; fix implemented (v3.2)', source: 'code+serial'})

CREATE (inst_ble_delivered_truth:Instrument {id: 'inst_ble_delivered_truth', name: 'delivered-truth BLE counters', surfaces: 'blePacketHz from txDelivered; gps CSV columns ble / ble_dlv / ble_sup after diff_age', answers: 'What the central actually received, and how many samples were superseded or blocked — row-diffs give true wire rate and loss for any window, from the log alone.', source: 'code'})

CREATE (gap_client_reassembly:InstrumentationGap {id: 'gap_client_reassembly', subject: 'SoloStorm message reassembly', missing: 'the client actual framing rule (timeout? notification-boundary? both?)', impact: 'The envelope is known to work empirically but its necessary and sufficient conditions are inferred from June trial-and-error, not read from client source. Any future change to chunking, spacing or message adjacency is therefore an experiment and must be desk-tested against a real SoloStorm handshake before it drives.', priority: 'high'})

MATCH (a {id: 'mod_tx_stage'}), (b {id: 'buf_tx_slot'}) CREATE (a)-[:WRITES_TO {}]->(b)
MATCH (a {id: 'mod_tx_pump'}), (b {id: 'buf_tx_slot'}) CREATE (a)-[:DRAINS {}]->(b)
MATCH (a {id: 'mod_tx_pump'}), (b {id: 'rule_tx_wire_envelope'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'mod_tx_pump'}), (b {id: 'rule_meta_guard_completion'}) CREATE (a)-[:ENFORCES {}]->(b)
MATCH (a {id: 'fm_ble_v3_wire_envelope_removal'}), (b {id: 'rule_tx_wire_envelope'}) CREATE (a)-[:VIOLATED {}]->(b)
MATCH (a {id: 'fm_ble_v3_wire_envelope_removal'}), (b {id: 'mod_ble'}) CREATE (a)-[:ARISES_FROM {site: 'txPump chunk loop', source: 'code'}]->(b)
MATCH (a {id: 'gap_client_reassembly'}), (b {id: 'rule_tx_wire_envelope'}) CREATE (a)-[:GAP_IN {}]->(b)
MATCH (a {id: 'inst_ble_delivered_truth'}), (b {id: 'mod_sdlog'}) CREATE (a)-[:SURFACED_BY {source: 'code'}]->(b)
MATCH (a {id: 'buf_tx_slot'}), (b {id: 'fm_ble_master_anchor_skipping'}) CREATE (a)-[:MITIGATES {how: 'a transient anchor gap now costs latency (supersede) instead of a lost sample (discard)', source: 'derived'}]->(b)

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

CREATE (inst_wire_counters:Instrument {id: 'inst_wire_counters', name: 'per-sentence wire counters', surfaces: 'serial, every 5 s: GGA/RMC/GSA/GSV/oth/ckfail', where: 'gnss.cpp handleRawNmeaLine', answers: 'What the receiver ACTUALLY delivered, counted before TinyGPS++ sees it. At a 20 Hz fix rate a healthy link reads GGA=100 RMC=100 per 5 s and ckfail=0. Any shortfall is loss upstream of the parser; any ckfail is corruption on the wire.', limitation: 'Serial only — not carried in the CSV, so it cannot be recovered from a log after the fact.', source: 'code'})

CREATE (inst_spd_age:Instrument {id: 'inst_spd_age', name: 'spd_age_ms column', surfaces: 'gps CSV', answers: 'Age of the velocity fields at the moment the row was built. A few ms means that epoch was completed by its own RMC. A value at or above the watchdog window means GGA arrived and RMC did not, so the row was completed by the watchdog and its speed/heading are carried over. Distinguishes a PARTIALLY lost epoch (row present, RMC missing) from a WHOLLY lost one (no row at all).', field_result: 'On the 2026-08-26 evening drive: p50 = 3 ms with only 5.6% of rows at or above the watchdog, while 31% of epochs produced no row at all — i.e. the surviving rows were healthy and the losses took GGA and RMC together.', source: 'code+measured'})

CREATE (inst_loop_phase:Instrument {id: 'inst_loop_phase', name: 'loop() phase timing', surfaces: 'serial, every 5 s: max microseconds per phase', where: 'RCX_RTK_Datalogger.ino', answers: 'Which phase of loop() holds the core longest. Exists because the UART ISR that empties the GNSS hardware FIFO runs on this core, so a phase that defers interrupts past the FIFO depth can cost whole sentences.', field_result: 'First measurement (2026-08-27 bench): gnss 7705-11722 us, imu 761-880, temp 283-458, ble 51-56, sdpush 124-150, heapFree 15-26, heapLargest 176-198. gnss_loop is the dominant cost in loop() and everything else is minor.', source: 'code+measured'})

CREATE (fm_gnss_uart_harness_intermittent:FailureMode {id: 'fm_gnss_uart_harness_intermittent', name: 'Intermittent GNSS UART harness fault', symptom: 'Sessions are bimodal, and the mode is fixed from the first second of a session to the last: either essentially perfect (20.00 Hz, ckfail ~0) or degraded (epoch rate anywhere from 19 Hz down to 0.7 Hz, with NMEA checksum failures and raw non-ASCII bytes in the sat log). Measured session damage rates ranged 0.00% to 74.03%, and damage rate and epoch rate track each other closely.', wire_fingerprint: 'Corrupted bytes are NOT random: 89.7% match the bit pattern 1_xxx010 (bit7 set in 100% of cases, bit1 in ~91%, bit0 in ~3%), identical across every degraded session. Corruption CASCADES — after the first bad byte, ~33% of the rest of that line is corrupt against a 0.66% baseline, and only 41% of lines recover before end-of-line. Probability of a byte being corrupt rises with its offset in the sentence (0.03% in the first five bytes, 1.31% by byte 55). Corrupted lines are LONGER than clean ones, so the receiver manufactures bytes rather than merely dropping them.', elimination: 'Temperature REFUTED (coolest session 147 F was 24.6% damaged; hottest 201 F was 6.7%). WiFi and BLE REFUTED (24.6% damage with radios off; 0.00% with WiFi carrying 758 B/s). RTCM volume REFUTED. NMEA volume REFUTED (GSV byte rate flat while epoch rate collapsed). GSV as a cause REFUTED (its block lands at GPS time .989, outside the loss windows; and a 1-in-20-epoch event cannot exceed ~5% loss). Firmware REFUTED by construction: identical build produced 20.00 Hz and 13.76 Hz on the same day. Replacing the LG290P module alone did NOT fix it.', resolution: 'Replacing the UART wiring harness resolved it: the session immediately after showed 20.00 Hz with zero gaps at or above 80 ms, 0.00% NMEA damage and zero non-ASCII bytes over 379 s.', not_understood: 'The exact electrical mechanism that turns a marginal connection into this specific framing-error pattern is NOT established. Baud-ratio mismatch and RC edge degradation were both simulated and neither reproduces the observed byte distribution. Recorded as an unexplained signature that is nonetheless a reliable FINGERPRINT of this fault.', source: 'measured'})

CREATE (rule_ble_epoch_cadence:Rule {id: 'rule_ble_epoch_cadence', name: 'BLE sample cadence is driven by the GNSS epoch', statement: 'ble_racecapture_tick sends one sample per NEW gps.epochSeq, not on a free-running millisecond gate, with a 100 ms fallback that only fires if the GNSS goes silent. A timer gate re-armed from now() keeps its loop-latency residual instead of correcting it, so it drifts against the receiver hard 50 ms epoch grid and periodically skips an epoch entirely. The fallback interval is deliberately far from the 50 ms grid so it cannot beat against real epochs, and a 10 Hz stream of stale-position samples is itself the fault signature.', enforcement: 'ble_racecapture.cpp tick()', source: 'code'})

CREATE (gap_raw_byte_visibility:InstrumentationGap {id: 'gap_raw_byte_visibility', subject: 'raw NMEA bytes on the GNSS UART', missing: 'Raw bytes are logged ONLY for GSV (sat CSV). GGA, RMC, GSA and PQTM are parsed and discarded, so their corruption cannot be measured after the fact.', impact: 'Every corruption rate quoted from field logs is a GSV rate extrapolated to the other sentence types. GSV is roughly a sixth of sentence traffic, so a per-second test for "was this second clean?" is badly underpowered — an apparently clean second can still carry damaged GGA/RMC. Conclusions that rest on corruption NOT correlating with loss are weak for this reason.', would_fix: 'Carrying the existing ckfail counter into the CSV, ideally broken down per sentence type.', priority: 'high'})

CREATE (gap_oth_sentence_rate:InstrumentationGap {id: 'gap_oth_sentence_rate', subject: 'unidentified PQTM traffic', missing: 'The wire counter reports oth=300 per 5 s where the code comment expects ~100 (PQTM EPE at 20 Hz). Roughly 40 sentences per second are arriving that have not been identified.', impact: 'Unaccounted bytes and parse time in gnss_loop, which the loop-phase instrument shows is the dominant cost in loop(). Not known to be harmful; not known to be harmless.', status: 'UNEXPLAINED — observed on a healthy bench session, not yet investigated', priority: 'medium'})

CREATE (gap_imu_low_rate:InstrumentationGap {id: 'gap_imu_low_rate', subject: 'QMI8658 sample rate', missing: 'Cause of the IMU logging at 4.03 Hz instead of ~50 Hz, with 345 gaps of ~1000 ms, on the 2026-08-27 session. imu_readTempC() returned NAN for every row of the preceding session while imu_read() returned valid accelerometer data on the same bus.', impact: 'IMU data is the acceleration and yaw channel; 4 Hz is unusable for motorsport analysis.', established: 'GNSS was simultaneously perfect (20.00 Hz, zero gaps) during the same session, which PROVES the IMU path does not starve the GNSS stream. That directional claim was previously asserted and is now refuted.', status: 'OPEN — cause undocumented; the ~1000 ms period is consistent with an I2C timeout but imu.cpp does not call Wire.setTimeOut(), so the effective timeout value is the core default and has not been verified against this core version', priority: 'high'})

CREATE (rb_gnss_triage:Runbook {id: 'rb_gnss_triage', name: 'Triage: rover logging below 20 Hz', module: 'mod_gnss', step_1: 'Read ckfail and the GGA/RMC counts in the 5 s wire line. ckfail>0 or GGA/RMC below 100 per 5 s means the loss is on the wire, upstream of all firmware — go to fm_gnss_uart_harness_intermittent and suspect the physical link.', step_2: 'If the wire counters are clean but epochs are missing, the loss is downstream: check spd_age_ms to separate watchdog-completed rows from wholly missing ones, and check the loop-phase line for any phase exceeding the FIFO depth.', step_3: 'Check whether the degradation is fixed for the whole session or varies within it. A session-constant mode points at the physical link; a smooth progressive change points at a resource, and a sharp on/off episode with full recovery points at an intermittent connection.', step_4: 'Do NOT read the sat CSV corruption rate as the whole-link corruption rate — see gap_raw_byte_visibility.', anti_patterns: 'Temperature, WiFi/BLE activity, RTCM volume, NMEA volume and satellite count have each been tested against multi-session field data and REFUTED as causes of epoch loss. Do not re-derive them.', source: 'measured'})

CREATE (rb_ble_triage:Runbook {id: 'rb_ble_triage', name: 'Triage: SoloStorm shows missing samples', module: 'mod_ble', step_1: 'Diff ble_dlv and ble_sup across the window. ble_sup climbing means the firmware had a sample and could not place it — the slot was busy, which is the anchor-starvation fingerprint. ble_sup at zero means the firmware placed every sample it built.', step_2: 'Check the ble column for state changes. A run of non-advancing ble_dlv that ENDS in ble going to 0 is a disconnect, not sample loss — the last ~1 s before a drop delivers nothing while the state column still reads subscribed.', step_3: 'If ble_dlv advanced on every subscribed epoch but SoloStorm still shows gaps, the samples reached the controller and died past it — compare against the SoloStorm export directly.', field_result: 'On the first fully clean session (2026-08-27, 379 s): 4475 subscribed epochs, 4433 delivered, ble_sup=0, and every one of the 43 non-advancing epochs was the run-up to one of two disconnects. On healthy hardware the sender loses essentially nothing.', source: 'measured'})

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
