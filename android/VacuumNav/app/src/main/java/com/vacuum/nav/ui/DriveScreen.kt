package com.vacuum.nav.ui

import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Slider
import androidx.compose.material3.Surface
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.lidar.LidarMode
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.service.RobotService
import com.vacuum.nav.ui.theme.DangerRed
import com.vacuum.nav.ui.theme.RobotOrange
import kotlin.math.abs

/**
 * Drive tab: joystick with deadman (release = stop, prompt §6), speed
 * slider, sonar chips - plus the four lidar behaviours ported from
 * robot_ws/hoverboard_laser:
 *
 *   MANUAL / AVOID / FOLLOW mode chips (joystick always takes over, the
 *   reference nodes pause on /JoyState the same way),
 *   WARNING banner with the nearest in-window distance + beep toggle
 *   (laser_Warning: face + buzzer),
 *   tracked-target readout (laser_Tracker selection = tracking).
 */
@Composable
fun DriveScreen(modifier: Modifier = Modifier) {
    val link = RobotService.link
    val telemetry by (link?.telemetry
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.Telemetry()) }).collectAsState()
    val lidarState by RobotService.lidarCtl.state.collectAsState()
    val config = remember { RobotConfig() }
    var speedFrac by remember { mutableFloatStateOf(0.6f) }
    val eStop = RobotService.eStop
    val mode = lidarState.mode

    Column(
        modifier.fillMaxSize().padding(16.dp),
        horizontalAlignment = Alignment.CenterHorizontally,
    ) {
        Text("Drive", fontSize = 22.sp, fontWeight = FontWeight.Bold)
        Spacer(Modifier.height(4.dp))
        Text(
            "Hold to drive · release to stop",
            fontSize = 13.sp,
            color = MaterialTheme.colorScheme.onSurfaceVariant,
        )

        // ---- lidar behaviour modes (robot_ws/hoverboard_laser) -----------
        Spacer(Modifier.height(10.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(8.dp),
        ) {
            ModeChip("Manual", mode == LidarMode.MANUAL, Modifier.weight(1f)) {
                RobotService.setLidarMode(LidarMode.MANUAL)
            }
            ModeChip("⚠ Avoid", mode == LidarMode.AVOID, Modifier.weight(1f)) {
                RobotService.setLidarMode(LidarMode.AVOID)
            }
            ModeChip("🎯 Follow", mode == LidarMode.FOLLOW, Modifier.weight(1f)) {
                RobotService.setLidarMode(LidarMode.FOLLOW)
            }
        }

        // ---- warning banner (laser_Warning) ------------------------------
        val warn = lidarState.warning
        Spacer(Modifier.height(8.dp))
        Surface(
            Modifier.fillMaxWidth(),
            shape = RoundedCornerShape(14.dp),
            color = when {
                warn.buzzer -> DangerRed
                warn.active -> DangerRed.copy(alpha = 0.25f)
                else -> MaterialTheme.colorScheme.surfaceVariant
            },
        ) {
            Column(Modifier.padding(10.dp), horizontalAlignment = Alignment.CenterHorizontally) {
                Text(
                    when {
                        warn.buzzer ->
                            "⚠ OBSTACLE %.2f m %s".format(
                                warn.nearestDistM,
                                bearing(warn.nearestAngleRad))
                        warn.active ->
                            "Obstacle %.2f m %s".format(
                                warn.nearestDistM,
                                bearing(warn.nearestAngleRad))
                        else -> "Clear ahead"
                    },
                    fontSize = 14.sp, fontWeight = FontWeight.Bold,
                    color = if (warn.buzzer) Color.White
                    else MaterialTheme.colorScheme.onSurface,
                )
                if (mode == LidarMode.FOLLOW) {
                    val t = lidarState.target
                    Text(
                        if (t == null) "Nothing to follow"
                        else "Tracking %.2f m %s".format(
                            t.rangeM, bearing(t.angleRad)),
                        fontSize = 12.sp,
                        color = if (warn.buzzer) Color.White.copy(alpha = 0.9f)
                        else MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
                if (mode == LidarMode.AVOID && lidarState.avoidanceWhy.isNotEmpty()) {
                    Text(
                        lidarState.avoidanceWhy,
                        fontSize = 11.sp,
                        color = if (warn.buzzer) Color.White.copy(alpha = 0.9f)
                        else MaterialTheme.colorScheme.onSurfaceVariant,
                    )
                }
            }
        }

        // Beep toggle (phone speaker; the real link also sets the HBC2
        // buzzer flag -> ESP32 GPIO25, same as the reference /Buzzer).
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
            verticalAlignment = Alignment.CenterVertically,
        ) {
            Text("Buzzer", fontSize = 13.sp)
            Switch(
                checked = RobotService.lidarCtl.buzzerEnabled,
                onCheckedChange = { RobotService.lidarCtl.buzzerEnabled = it },
            )
        }

        Spacer(Modifier.height(8.dp))
        Joystick(
            onDrive = { drive, turn ->
                RobotService.manualOverride()   // joystick takes the wheels
                val scale = (0.15 + speedFrac * 0.85).toDouble()
                RobotService.manualDrive(
                    drive * config.maxLinearMps * scale,
                    turn * config.maxAngularRps * scale,
                )
            },
        )

        Spacer(Modifier.height(16.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Text("Speed", fontSize = 14.sp, fontWeight = FontWeight.Medium)
            Text(
                "${"%.2f".format(config.maxLinearMps * (0.15 + speedFrac * 0.85))} m/s",
                fontSize = 13.sp,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Slider(
            value = speedFrac,
            onValueChange = { speedFrac = it },
        )

        Spacer(Modifier.height(8.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            SonarChip("Front", telemetry.sonarFrontM, Modifier.weight(1f))
            SonarChip("Left", telemetry.sonarLeftM, Modifier.weight(1f))
            SonarChip("Right", telemetry.sonarRightM, Modifier.weight(1f))
        }

        Spacer(Modifier.height(16.dp))
        Button(
            onClick = { RobotService.setEstop(!eStop) },
            Modifier.fillMaxWidth().height(52.dp),
            shape = RoundedCornerShape(50),
            colors = ButtonDefaults.buttonColors(
                containerColor = if (eStop) MaterialTheme.colorScheme.surfaceVariant
                else DangerRed,
            ),
        ) {
            Text(
                if (eStop) "CLEAR E-STOP" else "E-STOP",
                fontWeight = FontWeight.Bold, fontSize = 16.sp,
            )
        }
    }
}

/** "+12°" / "-3°" / "ahead" for a robot-frame bearing. */
private fun bearing(rad: Double): String {
    val deg = Math.toDegrees(rad)
    return when {
        abs(deg) < 3 -> "ahead"
        deg > 0 -> "+%.0f°".format(deg)
        else -> "%.0f°".format(deg)
    }
}

@Composable
private fun ModeChip(label: String, selected: Boolean, modifier: Modifier,
                     onClick: () -> Unit) {
    Surface(
        onClick = onClick,
        modifier = modifier.height(40.dp),
        shape = RoundedCornerShape(50),
        color = if (selected) RobotOrange
        else MaterialTheme.colorScheme.surfaceVariant,
    ) {
        androidx.compose.foundation.layout.Box(
            contentAlignment = Alignment.Center,
        ) {
            Text(
                label, fontSize = 13.sp,
                fontWeight = if (selected) FontWeight.Bold else FontWeight.Normal,
                color = if (selected) Color.White
                else MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
    }
}

@Composable
private fun SonarChip(name: String, meters: Float, modifier: Modifier) {
    val blocked = meters in 0.01f..0.35f
    Surface(
        modifier = modifier,
        shape = RoundedCornerShape(16.dp),
        color = (if (blocked) DangerRed else MaterialTheme.colorScheme
            .surfaceVariant).copy(alpha = 0.85f),
    ) {
        Column(
            Modifier.padding(10.dp),
            horizontalAlignment = Alignment.CenterHorizontally,
        ) {
            Text(name, fontSize = 11.sp,
                color = MaterialTheme.colorScheme.onSurfaceVariant)
            Text(
                if (meters <= 0f) "—" else "%.2f m".format(meters),
                fontSize = 15.sp, fontWeight = FontWeight.Bold,
                color = if (blocked) Color.White
                else MaterialTheme.colorScheme.onSurface,
            )
        }
    }
}
