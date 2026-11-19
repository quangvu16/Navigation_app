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
    }

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        // Safety (prompt §7): every stop path sends zero RPM through the link.
        if (intent?.action == ACTION_STOP) {
            link?.setWheelRpm(0f, 0f)
        }
        return START_STICKY
    }

    override fun onDestroy() {
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

        fun start(context: Context) {
            val intent = Intent(context, RobotService::class.java)
            if (Build.VERSION.SDK_INT >= 26) {
                context.startForegroundService(intent)
            } else {
                context.startService(intent)
            }
        }
    }
}
