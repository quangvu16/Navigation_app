package com.vacuum.nav.data

import android.content.BroadcastReceiver
import android.content.Context
import android.content.Intent
import com.vacuum.nav.service.RobotService

/**
 * Fires when a scheduled clean is due: starts the foreground brain and
 * begins cleaning (prompt §6). Re-arms itself for the next occurrence and
 * reschedules everything after a reboot (RECEIVE_BOOT_COMPLETED).
 */
class AlarmReceiver : BroadcastReceiver() {
    override fun onReceive(context: Context, intent: Intent) {
        when (intent.action) {
            Intent.ACTION_BOOT_COMPLETED -> {
                // Alarms do not survive reboots: re-arm from disk.
                val schedules = ScheduleStore.load(context)
                Scheduler.scheduleAll(context, schedules)
            }
            else -> {
                // Scheduled clean: brain first, then start. Room-targeted
                // schedules degrade to a full clean until the Phase-7
                // room planner is wired into the run loop.
                RobotService.start(context)
                RobotService.startCleaning()
                // Re-arm the next occurrence of this schedule.
                val id = intent.getLongExtra("id", -1L)
                val schedules = ScheduleStore.load(context)
                val s = schedules.find { it.id == id }
                if (s != null) Scheduler.scheduleAll(context, schedules)
            }
        }
    }
}
