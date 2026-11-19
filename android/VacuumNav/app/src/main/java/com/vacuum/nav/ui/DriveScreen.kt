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
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.collectAsState
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.vacuum.nav.robot.RobotConfig
import com.vacuum.nav.service.RobotService
import com.vacuum.nav.ui.theme.DangerRed

/**
 * Drive tab: joystick with deadman (release = stop, prompt §6), a speed
 * slider and live sonar readouts. Joystick input always takes the wheels
 * back from auto-clean (manualOverride).
 */
@Composable
fun DriveScreen(modifier: Modifier = Modifier) {
    val link = RobotService.link
    val telemetry by (link?.telemetry
        ?: remember { kotlinx.coroutines.flow.MutableStateFlow(
            com.vacuum.nav.robot.Telemetry()) }).collectAsState()
    val config = remember { RobotConfig() }
    var speedFrac by remember { mutableFloatStateOf(0.6f) }
    val eStop = RobotService.eStop

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
        Spacer(Modifier.height(16.dp))

        Joystick(
            onDrive = { drive, turn ->
                RobotService.manualOverride()
                val scale = (0.15 + speedFrac * 0.85).toFloat()
                val (l, r) = config.velocityToWheelRpm(
                    drive * config.maxLinearMps * scale,
                    turn * config.maxAngularRps * scale,
                )
                if (!eStop) link?.setWheelRpm(l.toFloat(), r.toFloat())
            },
        )

        Spacer(Modifier.height(20.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.SpaceBetween,
        ) {
            Text("Speed", fontSize = 14.sp, fontWeight = FontWeight.Medium)
            Text(
                "${"%.2f".format(config.maxLinearMps * (0.15 + speedFrac * 0.85))} m/s",
                fontSize = 14.sp,
                color = MaterialTheme.colorScheme.onSurfaceVariant,
            )
        }
        Slider(
            value = speedFrac,
            onValueChange = { speedFrac = it },
        )

        Spacer(Modifier.height(12.dp))
        Row(
            Modifier.fillMaxWidth(),
            horizontalArrangement = Arrangement.spacedBy(12.dp),
        ) {
            SonarChip("Front", telemetry.sonarFrontM, Modifier.weight(1f))
            SonarChip("Left", telemetry.sonarLeftM, Modifier.weight(1f))
            SonarChip("Right", telemetry.sonarRightM, Modifier.weight(1f))
        }

        Spacer(Modifier.height(20.dp))
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
                color = if (blocked) androidx.compose.ui.graphics.Color.White
                else MaterialTheme.colorScheme.onSurface,
            )
        }
    }
}
