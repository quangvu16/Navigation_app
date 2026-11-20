package com.vacuum.nav.service

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.media.AudioManager
import android.media.ToneGenerator
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import android.net.wifi.WifiManager
import androidx.core.app.NotificationCompat
import com.vacuum.nav.R
import com.vacuum.nav.data.RunStats
import com.vacuum.nav.lidar.LidarBehaviorController
import com.vacuum.nav.lidar.LidarMode
import com.vacuum.nav.data.DockPose
import com.vacuum.nav.data.DockStore
import com.vacuum.nav.data.HistoryStore
import com.vacuum.nav.data.RunRecord
import com.vacuum.nav.data.ZoneStore
import com.vacuum.nav.robot.MapperEngine
import com.vacuum.nav.robot.NavEngine
import com.vacuum.nav.robot.NavState
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.RobotLink
import com.vacuum.nav.robot.SimRobotLink
import com.vacuum.nav.ui.ScanTransform
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.launch

/**
 * Owns the robot brain so it outlives the UI (prompt §3): a foreground
 * service (connectedDevice type) holding a partial WakeLock and a WifiLock
 * (Wi-Fi power-save adds 100-300 ms latency which breaks a control loop).
 *
 * Phase 1 runs the simulator inside this service; Phase 2 swaps in the UDP
 * link + the 20 Hz control loop with no change to the ownership model.
 */
class RobotService : Service() {

    private var wakeLock: PowerManager.WakeLock? = null
    private var wifiLock: WifiManager.WifiLock? = null
    private var controlThread: Thread? = null
    private var alive = true
    private var tone: ToneGenerator? = null
    private var scanJob: kotlinx.coroutines.Job? = null
    private var lastNavMapVersion = -1
    private var telemetryJob: kotlinx.coroutines.Job? = null

    // Diagnostics rates: cumulative counters, zeroed by the1 Hz ticker.
    @Volatile private var scanCount = 0L
    @Volatile private var telemCount = 0L
    @Volatile private var lastScanAt = 0L
    @Volatile private var lastTelemAt = 0L
    private val serviceScope = CoroutineScope(SupervisorJob() + Dispatchers.Default)

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        instance = this
        tone = runCatching {
            ToneGenerator(AudioManager.STREAM_MUSIC, 60)
        }.getOrNull()

        val pm = getSystemService(Context.POWER_SERVICE) as PowerManager
        wakeLock = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "vacuum:brain").apply {
            setReferenceCounted(false)
            acquire()
        }
        // WIFI_MODE_FULL_LOW_LATENCY needs API 29; older phones get HIGH_PERF.
        val wm = applicationContext.getSystemService(Context.WIFI_SERVICE) as WifiManager
        val mode = if (Build.VERSION.SDK_INT >= 29) {
            WifiManager.WIFI_MODE_FULL_LOW_LATENCY
        } else {
            @Suppress("DEPRECATION")
            WifiManager.WIFI_MODE_FULL_HIGH_PERF
        }
        wifiLock = wm.createWifiLock(mode, "vacuum:link").apply {
            setReferenceCounted(false)
            acquire()
        }

        startForeground(NOTIF_ID, buildNotification())
        if (link == null) {
            link = SimRobotLink(RobotConfig())
        }
        mapper = MapperEngine(RobotConfig())   // Phase 3 live mapping
        // Diagnostics: publish link rates once a second (prompt §6).
        serviceScope.launch {
            while (alive) {
                kotlinx.coroutines.delay(1000)
                val now = System.currentTimeMillis()
                val s = scanCount; val t = telemCount
                scanCount = 0; telemCount = 0
                _linkStats.value = LinkStats(
                    transport = link?.description ?: "none",
                    scanHz = s,
                    telemHz = t,
                    lastScanAgeMs = if (lastScanAt == 0L) -1
                        else now - lastScanAt,
                    lastTelemAgeMs = if (lastTelemAt == 0L) -1
                        else now - lastTelemAt,
                )
            }
        }
        // Phase 8: every finished run lands in History (prompt §6).
        RunStats.onEnded = { areaDelta, activeDelta, startedAt ->
            val ctx = applicationContext
            HistoryStore.add(ctx, RunRecord(
                id = System.nanoTime(),
                startedAtMs = startedAt,
                durationSec = activeDelta.toLong(),
                areaM2 = areaDelta,
                coveragePct = 0.0,   // filled when the Phase-7 tracker
                                     // feeds the run loop
                mode = "full",
            ))
        }
        nav = NavEngine(RobotConfig())         // Phase 4 go-to-goal
        watchScans()   // feeds lidarCtl + mapper + nav from the current link

        // 20 Hz control thread. Precedence: E-STOP > lidar behaviour modes
        // (AVOID/FOLLOW) > auto-clean bump-and-go > manual joystick. EVERY
        // path goes through the bumper polygons (collision_monitor
        // reference) and the E-STOP gate (prompt §7).
        val cfg = RobotConfig()
        controlThread = Thread {
            while (alive) {
                val l = link
                if (l != null) {
                    val t = l.telemetry.value
                    RunStats.tickRunning(0.05)
                    RunStats.onPose(t.pose.x.toDouble(),
                        t.pose.y.toDouble(), cfg.brushWidthM)

                    // Phase 6: frontier exploration drives go-to goals in
                    // a loop; finished -> return to the start pose.
                    if (exploreActive && !goToActive && !eStop) {
                        val n = nav
                        if (n == null) {
                            exploreActive = false
                        } else {
                            val g = n.exploreGoal(t.pose.x, t.pose.y)
                            if (g != null) {
                                if (exploreStart == null)
                                    exploreStart =
                                        t.pose.x.toDouble() to
                                            t.pose.y.toDouble()
                                if (!startGoTo(applicationContext, g.first,
                                        g.second)
                                ) {
                                    // Frontier behind a keepout/wall:
                                    // exploration is done for now.
                                    exploreActive = false
                                }
                            } else {
                                // No frontier left: return to start (§5).
                                exploreActive = false
                                val s = exploreStart
                                if (s != null)
                                    startGoTo(applicationContext, s.first,
                                        s.second)
                            }
                        }
                    }

                    // Phase 8: coverage queue - pull the next waypoint
                    // whenever GoTo is idle (and nothing else owns it).
                    if (coverageQueue.isNotEmpty() && !goToActive &&
                        !eStop && !exploreActive &&
                        dockPhase == DockPhase.NONE
                    ) {
                        val wp = coverageQueue.first()
                        if (!startGoTo(applicationContext, wp.first,
                                wp.second, keepRun = true)
                        ) {
                            coverageQueue.removeFirst()   // skip bad waypoint
                        }
                    }

                    val cmd: Pair<Double, Double> = when {
                        eStop -> 0.0 to 0.0
                        // Phase 8: reverse-in final leg of docking.
                        dockPhase == DockPhase.FINAL ->
                            lidarCtl.applyBumper(
                                dockFinalStep(t.pose.x, t.pose.y,
                                    t.pose.theta) ?: (0.0 to 0.0))
                        // Phase 4: go-to-goal owns the wheels until it
                        // finishes or the joystick/lidar mode takes over.
                        goToActive -> {
                            val n = nav
                            if (n == null) {
                                goToActive = false
                                0.0 to 0.0
                            } else {
                                val vw = n.step(t.pose.x, t.pose.y,
                                    t.pose.theta)
                                val st = n.state.value.state
                                if (st == NavState.REACHED ||
                                    st == NavState.STUCK ||
                                    st == NavState.REJECTED
                                ) {
                                    goToActive = false
                                    // Coverage queue: this waypoint is done
                                    // (or unreachable - skip it either way).
                                    if (coverageQueue.isNotEmpty())
                                        coverageQueue.removeFirst()
                                    notifyApproachEnded(
                                        st == NavState.REACHED)
                                    0.0 to 0.0
                                } else {
                                    lidarCtl.applyBumper(vw)
                                }
                            }
                        }
                        lidarCtl.currentMode() != LidarMode.MANUAL ->
                            lidarCtl.compute(System.currentTimeMillis())
                                ?: (0.0 to 0.0)
                        autoClean -> {
                            // Bump-and-go (sim); front sonar blocked -> turn.
                            val raw = if (t.sonarFrontM in 0.02..0.35)
                                (0.0 to 0.9) else (0.16 to 0.0)
                            lidarCtl.applyBumper(raw)
                        }
                        else -> lidarCtl.applyBumper(manualV to manualW)
                    }
                    val (wl, wr) = cfg.velocityToWheelRpm(cmd.first, cmd.second)
                    l.setWheelRpm(wl.toFloat(), wr.toFloat())

                    // Warning: phone beep on the edge + HBC2 buzzer flag
                    // (-> ESP32 GPIO25 on the real link, /Buzzer in the
                    // reference stack).
                    val warnOn = lidarCtl.state.value.warning.buzzer &&
                        lidarCtl.buzzerEnabled
                    l.setBuzzer(warnOn)
                    lidarCtl.buzzerEdge()?.let { on ->
                        if (on) runCatching {
                            tone?.startTone(ToneGenerator.TONE_PROP_BEEP, 150)
                        }
                    }
                }
                try {
                    Thread.sleep(50)
                } catch (_: InterruptedException) {
                    break
                }
            }
        }.apply { isDaemon = true; start() }
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        // Safety (prompt §7): every stop path sends zero RPM through the link.
        if (intent?.action == ACTION_STOP) {
            link?.setWheelRpm(0f, 0f)
        }
        return START_STICKY
    }

    /** Re-point the brain at a new transport (SIM / UDP / BLE), keeping the
     *  control thread and lidar layer alive across the switch. */
    private fun switchLink(newLink: RobotLink) {
        val old = link
        link = newLink
        watchScans()
        old?.setWheelRpm(0f, 0f)
        old?.close()
    }

    private fun watchScans() {
        scanJob?.cancel()
        telemetryJob?.cancel()
        val l = link ?: return
        telemetryJob = serviceScope.launch {
            l.telemetry.collect {
                telemCount++
                lastTelemAt = System.currentTimeMillis()
            }
        }
        scanJob = serviceScope.launch {
            l.scans.collect { scan ->
                scanCount++
                lastScanAt = System.currentTimeMillis()
                lidarCtl.onScan(scan)
                // Phase 3: feed the mapper with the pose guess (sim truth
                // today; EKF output once Phase 2's filter is on-device).
                // ~10 ms of scan matching on this dispatcher, not the UI.
                val guess = l.telemetry.value.pose
                mapper?.feed(scan.ranges, guess)

                // Phase 4: keep the nav costmap fresh - static layer when
                // the mapper publishes a new version, live obstacles from
                // every scan (they decay after 2 s on the C++ side).
                val snap = mapper?.snapshot?.value
                if (snap != null && snap.version != lastNavMapVersion) {
                    lastNavMapVersion = snap.version
                    nav?.setMap(snap)
                }
                val n = nav
                if (n != null) {
                    val cfg = RobotConfig()
                    val pts = ArrayList<Float>(256)
                    var i = 0
                    while (i < scan.ranges.size) {
                        val r = scan.ranges[i].toDouble()
                        if (r > 0.0) {
                            val p = ScanTransform.binToWorldPoint(
                                cfg, guess.x.toDouble(), guess.y.toDouble(),
                                guess.theta.toDouble(), i, r)
                            if (p != null) {
                                pts.add(p.first.toFloat())
                                pts.add(p.second.toFloat())
                            }
                        }
                        i += 2   // every other beam: plenty for the layer
                    }
                    val arr = pts.toFloatArray()
                    n.setObstacles(arr, arr.size / 2)
                }
            }
        }
    }

    override fun onDestroy() {
        alive = false
        controlThread?.join(200)
        serviceScope.cancel()
        link?.setWheelRpm(0f, 0f)  // zero on service destroy (prompt §7)
        link?.setBuzzer(false)
        link?.close()
        link = null
        mapper?.close()
        mapper = null
        nav?.close()
        nav = null
        runCatching { tone?.release() }
        wakeLock?.release()
        wifiLock?.release()
        instance = null
        super.onDestroy()
    }

    private fun buildNotification(): Notification {
        val nm = getSystemService(NotificationManager::class.java)
        if (Build.VERSION.SDK_INT >= 26) {
            nm.createNotificationChannel(
                NotificationChannel(
                    CHANNEL_ID,
                    getString(R.string.notif_channel_robot),
                    NotificationManager.IMPORTANCE_LOW,
                )
            )
        }
        return NotificationCompat.Builder(this, CHANNEL_ID)
            .setSmallIcon(android.R.drawable.ic_menu_compass)
            .setContentTitle(getString(R.string.notif_running))
            .setOngoing(true)
            .build()
    }

    companion object {
        const val ACTION_STOP = "com.vacuum.nav.action.STOP"
        private const val CHANNEL_ID = "robot_brain"
        private const val NOTIF_ID = 1

        // Compose-observable mirror of [link]: the Activity routinely
        // composes BEFORE the service's onCreate assigns the link (async
        // startForegroundService), and a plain @Volatile read captured in
        // a val never recomposes - Home sat on "Starting robot brain..."
        // forever. Every assignment flows through the setter into here.
        private val _linkFlow =
            kotlinx.coroutines.flow.MutableStateFlow<RobotLink?>(null)
        val linkFlow: kotlinx.coroutines.flow.StateFlow<RobotLink?> =
            _linkFlow.asStateFlow()

        /** The brain lives here while the service runs; null otherwise. */
        @Volatile
        var link: RobotLink? = null
            private set(value) {
                field = value
                _linkFlow.value = value
            }

        @Volatile
        private var instance: RobotService? = null

        /** Auto-clean policy engaged (Start button); cleared by Stop or the
         *  joystick taking over (manual override). */
        @Volatile
        var autoClean = false
            private set

        @Volatile
        var eStop = false
            private set

        /** Phase 3 occupancy mapper (grows while scans arrive). */
        @Volatile
        var mapper: MapperEngine? = null
            private set

        /** Diagnostics link statistics (published at1 Hz). */
        data class LinkStats(
            val transport: String = "none",
            val scanHz: Long = 0,
            val telemHz: Long = 0,
            val lastScanAgeMs: Long = -1,
            val lastTelemAgeMs: Long = -1,
        )

        private val _linkStats = kotlinx.coroutines.flow.MutableStateFlow(
            LinkStats())
        val linkStats: kotlinx.coroutines.flow.StateFlow<LinkStats> =
            _linkStats.asStateFlow()

        /** Phase 4 go-to-goal engine (A* + RPP + recovery). */
        @Volatile
        var nav: NavEngine? = null
            private set

        /** True while a goal is being pursued (control thread owns wheels). */
        @Volatile
        var goToActive = false
            private set

        /** Phase 6: frontier exploration loop running. */
        @Volatile
        var exploreActive = false
            private set

        // ---- Phase 8: room-targeted cleaning -----------------------------
        /** Rooms selected on the Rooms tab (stable segmenter ids). */
        @Volatile
        var selectedRooms = setOf<Int>()

        /** Remaining coverage waypoints (flattened pairs), consumed by the
         *  control thread through GoTo, one goal at a time. */
        val coverageQueue = ArrayDeque<Pair<Double, Double>>()

        // ---- Phase 8: docking --------------------------------------------
        /** NONE -> APPROACH (GoTo to0.6 m in front) -> FINAL (reverse in). */
        enum class DockPhase { NONE, APPROACH, FINAL, DOCKED }

        @Volatile
        var dockPhase: DockPhase = DockPhase.NONE
            private set

        @Volatile
        private var dockTarget: DockPose? = null

        @Volatile
        private var finalStartMs = 0L

        /** Save the robot's current pose as the dock ("dock is here"). */
        fun setDock(context: Context) {
            val pose = link?.telemetry?.value?.pose ?: return
            DockStore.save(context,
                DockPose(pose.x.toDouble(), pose.y.toDouble(),
                    pose.theta.toDouble()))
        }

        /**
         * Drive to the dock: GoTo to the approach point0.6 m BEHIND the
         * docked pose (the robot faces outward when docked), then the
         * reverse-in final leg (repo dock_manager behaviour: back in
         * because the front sonar reflex only watches the front -
         * docs/ASSUMPTIONS.md §4.1). Returns false when no dock is saved.
         */
        fun goDock(context: Context): Boolean {
            val dock = DockStore.load(context) ?: return false
            dockTarget = dock
            val ax = dock.x - cos(dock.theta) * 0.6
            val ay = dock.y - sin(dock.theta) * 0.6
            if (!startGoTo(context, ax, ay)) return false
            dockPhase = DockPhase.APPROACH
            return true
        }

        fun cancelDock() {
            dockPhase = DockPhase.NONE
            dockTarget = null
            cancelGoTo()
        }

        /** Final reverse-in leg; returns null when the phase is done. */
        fun dockFinalStep(poseX: Float, poseY: Float, theta: Float): Pair<Double, Double>? {
            val dock = dockTarget
            if (dock == null || dockPhase != DockPhase.FINAL) return null
            val d = hypot(dock.x - poseX, dock.y - poseY)
            // Face the dock heading while backing (small P on heading error).
            var headingErr = dock.theta - theta
            while (headingErr > Math.PI) headingErr -= 2 * Math.PI
            while (headingErr < -Math.PI) headingErr += 2 * Math.PI
            if (d < 0.10) {
                dockPhase = DockPhase.DOCKED
                dockTarget = null
                return 0.0 to 0.0
            }
            if (System.currentTimeMillis() - finalStartMs > 4000) {
                dockPhase = DockPhase.DOCKED   // timed out: call it docked
                dockTarget = null
                return 0.0 to 0.0
            }
            val w = (headingErr * 1.5).coerceIn(-0.5, 0.5)
            return -0.08 to w
        }

        fun notifyApproachEnded(navOk: Boolean) {
            if (dockPhase == DockPhase.APPROACH) {
                if (navOk) {
                    dockPhase = DockPhase.FINAL
                    finalStartMs = System.currentTimeMillis()
                } else {
                    dockPhase = DockPhase.NONE
                    dockTarget = null
                }
            }
        }

        /** Where exploration started (returned to when finished, §5). */
        @Volatile
        private var exploreStart: Pair<Double, Double>? = null

        /** Toggle frontier exploration from the Map rail button. */
        fun toggleExplore() {
            if (exploreActive) {
                exploreActive = false
                cancelGoTo()
            } else {
                // Need a map to explore against (the mapper builds it as
                // the robot drives; before that there is nothing).
                if (nav?.state?.value?.let { true } != true) return
                exploreStart = null   // captured on the first goal below
                autoClean = false
                RunStats.stopRun()
                lidarCtl.setMode(LidarMode.MANUAL)
                exploreActive = true
            }
        }

        /** Lidar behaviours (avoidance / tracking / warning / following),
         *  ported from robot_ws/hoverboard_laser. UI reads [LidarBehaviorController.state]. */
        val lidarCtl = LidarBehaviorController()

        // Manual joystick demand in (v, w); applied through the bumper in
        // the control thread (prompt §7: joystick never bypasses it).
        @Volatile var manualV = 0.0
            private set
        @Volatile var manualW = 0.0
            private set

        fun manualDrive(v: Double, w: Double) {
            manualV = v
            manualW = w
        }

        fun setLidarMode(mode: LidarMode) {
            dockPhase = DockPhase.NONE
            dockTarget = null
            lidarCtl.setMode(mode)
            if (mode != LidarMode.MANUAL) {
                // A lidar behaviour owns the wheels: cleaning and go-to yield.
                autoClean = false
                goToActive = false
                nav?.cancel()
                RunStats.stopRun()
                manualV = 0.0
                manualW = 0.0
            }
        }

        /**
         * Phase 4: send the robot to a tapped world point. Pushes the
         * current no-go zones into the costmap first (a goal behind a
         * virtual wall is rejected with a clear message, prompt §7).
         * Returns false when the goal is inside an obstacle/keepout.
         */
        fun startGoTo(context: Context, x: Double, y: Double,
                      keepRun: Boolean = false): Boolean {
            val n = nav ?: return false
            val zones = ZoneStore.load(context)
            val rects = FloatArray(zones.size * 4)
            zones.forEachIndexed { i, z ->
                rects[i * 4] = z.x.toFloat()
                rects[i * 4 + 1] = z.y.toFloat()
                rects[i * 4 + 2] = z.w.toFloat()
                rects[i * 4 + 3] = z.h.toFloat()
            }
            n.setKeepouts(rects)
            val ok = n.setGoal(x, y)
            if (ok) {
                autoClean = false
                if (!keepRun) RunStats.stopRun()
                manualV = 0.0
                manualW = 0.0
                lidarCtl.setMode(LidarMode.MANUAL)
                goToActive = true
            }
            return ok
        }

        /**
         * Phase 8: clean the selected rooms along their boustrophedon
         * paths (prompt §5/§8): queue every waypoint, the control thread
         * walks it with GoTo one goal at a time. Returns false when no
         * rooms are selected or none yield a path.
         */
        fun startRoomCleaning(context: Context): Boolean {
            val n = nav ?: return false
            coverageQueue.clear()
            for (id in selectedRooms) {
                val path = n.coveragePath(id) ?: continue
                var i = 0
                while (i + 1 < path.size) {
                    coverageQueue.add(
                        path[i].toDouble() to path[i + 1].toDouble())
                    i += 2
                }
            }
            if (coverageQueue.isEmpty()) return false
            eStop = false
            autoClean = false
            lidarCtl.setMode(LidarMode.MANUAL)
            RunStats.startRun()
            return true
        }

        fun cancelGoTo() {
            goToActive = false
            nav?.cancel()
            link?.setWheelRpm(0f, 0f)
        }

        fun start(context: Context) {
            val intent = Intent(context, RobotService::class.java)
            if (Build.VERSION.SDK_INT >= 26) {
                context.startForegroundService(intent)
            } else {
                context.startService(intent)
            }
        }

        // ---- transports (SIM now; UDP and BLE are the real-robot paths) --
        fun connectSim() {
            instance?.switchLink(SimRobotLink(RobotConfig()))
        }

        fun connectUdp(host: String) {
            instance?.switchLink(
                com.vacuum.nav.robot.UdpRobotLink(RobotConfig(), host))
        }

        fun connectBle(address: String) {
            val ctx = instance ?: return
            val ble = com.vacuum.nav.robot.BleRobotLink(
                ctx.applicationContext, RobotConfig())
            instance?.switchLink(ble)
            ble.start(address)
        }

        fun startCleaning() {
            autoClean = true
            eStop = false
            RunStats.startRun()
        }

        fun stopCleaning() {
            autoClean = false
            goToActive = false
            coverageQueue.clear()
            nav?.cancel()
            RunStats.stopRun()
            link?.setWheelRpm(0f, 0f)
        }

        /** Joystick drag: take the wheels back from auto-clean, go-to AND
         *  any lidar behaviour (reference: nodes pause while joystick
         *  active). */
        fun manualOverride() {
            dockPhase = DockPhase.NONE
            dockTarget = null
            if (autoClean) {
                autoClean = false
                RunStats.stopRun()
            }
            if (goToActive) {
                goToActive = false
                nav?.cancel()
            }
            if (lidarCtl.currentMode() != LidarMode.MANUAL)
                lidarCtl.setMode(LidarMode.MANUAL)
        }

        fun setEstop(on: Boolean) {
            dockPhase = DockPhase.NONE
            dockTarget = null
            eStop = on
            if (on) {
                autoClean = false
                goToActive = false
                nav?.cancel()
                RunStats.stopRun()
                link?.setWheelRpm(0f, 0f)
            }
        }
    }
}
