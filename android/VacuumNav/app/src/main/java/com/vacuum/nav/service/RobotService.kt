package com.vacuum.nav.service

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.Service
import android.content.Context
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import android.net.wifi.WifiManager
import androidx.core.app.NotificationCompat
import com.vacuum.nav.R
import com.vacuum.nav.data.RunStats
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.robot.RobotLink
import com.vacuum.nav.robot.SimRobotLink

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

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onCreate() {
        super.onCreate()
        instance = this

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

        // 20 Hz control thread: auto-clean ("bump-and-go" in the simulator -
        // front sonar blocked -> turn, else forward) + run-stat integration.
        // The planner replaces the bump-and-go policy in Phase 4; the rate
        // and the safety interlocks (E-stop, deadman) stay here.
        val cfg = RobotConfig()
        controlThread = Thread {
            while (alive) {
                val l = link
                if (l != null) {
                    val t = l.telemetry.value
                    RunStats.tickRunning(0.05)
                    RunStats.onPose(t.pose.x.toDouble(),
                        t.pose.y.toDouble(), cfg.brushWidthM)
                    if (autoClean && !eStop) {
                        val front = t.sonarFrontM
                        if (front in 0.02..0.35) {
                            // Blocked ahead: turn in place away from it.
                            l.setWheelRpm(-14f, 16f)
                        } else {
                            l.setWheelRpm(16f, 16f)
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
        link?.setWheelRpm(0f, 0f)  // zero on service destroy (prompt §7)
        link?.close()
        link = null
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

        /** Joystick drag: take the wheels back from auto-clean. */
        fun manualOverride() {
            if (autoClean) {
                autoClean = false
                RunStats.stopRun()
            }
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
