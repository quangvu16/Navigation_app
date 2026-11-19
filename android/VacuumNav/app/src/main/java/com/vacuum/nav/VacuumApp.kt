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
        instance = this
        RobotService.start(this)
    }

    companion object {
        /** App context for helpers that need it outside a composable (BLE scan). */
        @Volatile
        var instance: VacuumApp? = null
            private set
    }
}
