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
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.RobotLink
import com.vacuum.nav.robot.SimRobotLink
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

        // Feed every scan to the lidar behaviours (avoidance/tracking/
        // warning/following, ported from robot_ws/hoverboard_laser).
        link?.let { l ->
            serviceScope.launch {
                l.scans.collect { lidarCtl.onScan(it) }
            }
        }

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

    override fun onDestroy() {
        alive = false
        controlThread?.join(200)
        serviceScope.cancel()
        link?.setWheelRpm(0f, 0f)  // zero on service destroy (prompt §7)
        link?.setBuzzer(false)
        link?.close()
        link = null
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
                // A lidar behaviour owns the wheels: cleaning yields.
                autoClean = false
                RunStats.stopRun()
                manualV = 0.0
                manualW = 0.0
            }
        }

        fun start(context: Context) {
            val intent = Intent(context, RobotService::class.java)
            if (Build.VERSION.SDK_INT >= 26) {
                context.startForegroundService(intent)
            } else {
                context.startService(intent)
            }
        }

        fun startCleaning() {
            autoClean = true
            eStop = false
            RunStats.startRun()
        }

        fun stopCleaning() {
            autoClean = false
            RunStats.stopRun()
            link?.setWheelRpm(0f, 0f)
        }

        /** Joystick drag: take the wheels back from auto-clean AND any
         *  lidar behaviour (reference: nodes pause while joystick active). */
        fun manualOverride() {
            if (autoClean) {
                autoClean = false
                RunStats.stopRun()
            }
            if (lidarCtl.currentMode() != LidarMode.MANUAL)
                lidarCtl.setMode(LidarMode.MANUAL)
        }

        fun setEstop(on: Boolean) {
            eStop = on
            if (on) {
                autoClean = false
                RunStats.stopRun()
                link?.setWheelRpm(0f, 0f)
            }
        }
    }
}
