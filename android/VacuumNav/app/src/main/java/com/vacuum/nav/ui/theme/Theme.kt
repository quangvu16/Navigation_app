package com.vacuum.nav.ui.theme

import androidx.compose.foundation.isSystemInDarkTheme
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.ui.graphics.Color

// Map palette follows the vacuum-app convention: dark unknown, light free
// space, saturated accents for robot/zones (Roborock/Tapo style).
val MapUnknown = Color(0xFF2A2D32)
val MapFree = Color(0xFFE8E6E1)
val MapWall = Color(0xFF4A4D52)
val RobotOrange = Color(0xFFFF6D00)
val LidarCyan = Color(0xFF00B8D4)
val DangerRed = Color(0xFFD32F2F)

private val LightColors = lightColorScheme(
    primary = RobotOrange,
    error = DangerRed,
)

private val DarkColors = darkColorScheme(
    primary = RobotOrange,
    error = DangerRed,
)

@Composable
fun VacuumTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = if (isSystemInDarkTheme()) DarkColors else LightColors,
        content = content,
    )
}
