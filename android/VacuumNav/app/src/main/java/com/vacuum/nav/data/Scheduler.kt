package com.vacuum.nav.data

import android.app.AlarmManager
import android.app.PendingIntent
import android.content.Context
import android.content.Intent
import android.os.Build
import java.util.Calendar

/**
 * Phase 8: schedule execution (prompt §6: AlarmManager
 * setExactAndAllowWhileIdle + foreground service start). Each enabled
 * Schedule arms one exact alarm at its next occurrence; the receiver
 * starts RobotService and begins a clean.
 *
 * Exact alarms need SCHEDULE_EXACT_ALARM (API 31+); when the user has not
 * granted "Alarms & reminders" we fall back to setAndAllowWhileIdle
 * (inexact but still fires Doze-deferred). Note in the UI: the phone
 * should be charging / on Wi-Fi for reliable firing (prompt §6).
 */
object Scheduler {

    fun scheduleAll(context: Context, schedules: List<Schedule>) {
        val am = context.getSystemService(AlarmManager::class.java) ?: return
        cancelAll(context, schedules)
        for (s in schedules.filter { it.enabled }) {
            val at = nextOccurrenceMillis(s) ?: continue
            val pi = pendingIntent(context, s.id)
            if (Build.VERSION.SDK_INT >= 31 && am.canScheduleExactAlarms()) {
                am.setExactAndAllowWhileIdle(AlarmManager.RTC_WAKEUP, at, pi)
            } else {
                am.setAndAllowWhileIdle(AlarmManager.RTC_WAKEUP, at, pi)
            }
        }
    }

    fun cancelAll(context: Context, schedules: List<Schedule>) {
        val am = context.getSystemService(AlarmManager::class.java) ?: return
        for (s in schedules) am.cancel(pendingIntent(context, s.id))
    }

    private fun pendingIntent(context: Context, id: Long): PendingIntent {
        val intent = Intent(context, AlarmReceiver::class.java)
            .putExtra("id", id)
        return PendingIntent.getBroadcast(
            context, id.toInt(), intent,
            PendingIntent.FLAG_UPDATE_CURRENT or PendingIntent.FLAG_IMMUTABLE,
        )
    }

    /**
     * Next firing time (epoch ms) for a schedule, or null when it can
     * never fire (empty days mask with a past time today is treated as
     * "tomorrow once" - same as the reference UX's one-shot routines).
     */
    fun nextOccurrenceMillis(s: Schedule, now: Calendar = Calendar.getInstance()): Long? {
        val cal = (now.clone() as Calendar)
        cal.set(Calendar.SECOND, 0)
        cal.set(Calendar.MILLISECOND, 0)
        cal.set(Calendar.HOUR_OF_DAY, s.hour)
        cal.set(Calendar.MINUTE, s.minute)

        // Days of week in java.util.Calendar: SUNDAY=1 .. SATURDAY=7.
        // Our bitmask: bit0=Mon .. bit6=Sun.
        fun maskBit(calDay: Int): Int {   // calDay = Calendar.DAY_OF_WEEK
            val idx = (calDay + 5) % 7   // Mon->0 ... Sun->6
            return 1 shl idx
        }

        if (s.daysMask == 0) {
            // One-shot: today if still ahead, else tomorrow.
            if (cal.timeInMillis > now.timeInMillis) return cal.timeInMillis
            cal.add(Calendar.DAY_OF_YEAR, 1)
            return cal.timeInMillis
        }
        // Weekly: search the next `daysMask` day within8 days.
        for (i in 0 until 8) {
            val candidate = cal.clone() as Calendar
            candidate.add(Calendar.DAY_OF_YEAR, i)
            if (s.daysMask and maskBit(candidate.get(Calendar.DAY_OF_WEEK)) != 0 &&
                candidate.timeInMillis > now.timeInMillis
            ) {
                return candidate.timeInMillis
            }
        }
        return null
    }
}
