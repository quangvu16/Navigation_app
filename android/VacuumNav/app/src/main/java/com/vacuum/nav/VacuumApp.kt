package com.vacuum.nav

import android.app.Application
import com.vacuum.nav.service.RobotService

/**
 * The robot brain starts with the app process (prompt §3: UI can close, the
 * service keeps cleaning). Phase 1: the simulator; later: the UDP link and
 * the scheduler all hang off this same service.
 */
class VacuumApp : Application() {
    override fun onCreate() {
        super.onCreate()
        RobotService.start(this)
    }
}
