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
import com.vacuum.nav.data.ZoneStore
import com.vacuum.nav.robot.MapperEngine
import com.vacuum.nav.robot.NavEngine
import com.vacuum.nav.robot.NavState
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.RobotLink
import com.vacuum.nav.robot.SimRobotLink
import com.vacuum.nav.ui.ScanTransform
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
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

                    val cmd: Pair<Double, Double> = when {
                        eStop -> 0.0 to 0.0
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
        val l = link ?: return
        scanJob = serviceScope.launch {
            l.scans.collect { scan ->
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

        /** The brain lives here while the service runs; null otherwise. */
        @Volatile
        var link: RobotLink? = null
            private set

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

        /** Phase 4 go-to-goal engine (A* + RPP + recovery). */
        @Volatile
        var nav: NavEngine? = null
            private set

        /** True while a goal is being pursued (control thread owns wheels). */
        @Volatile
        var goToActive = false
            private set

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
        fun startGoTo(context: Context, x: Double, y: Double): Boolean {
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
                RunStats.stopRun()
                manualV = 0.0
                manualW = 0.0
                lidarCtl.setMode(LidarMode.MANUAL)
                goToActive = true
            }
            return ok
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
            nav?.cancel()
            RunStats.stopRun()
            link?.setWheelRpm(0f, 0f)
        }

        /** Joystick drag: take the wheels back from auto-clean, go-to AND
         *  any lidar behaviour (reference: nodes pause while joystick
         *  active). */
        fun manualOverride() {
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
