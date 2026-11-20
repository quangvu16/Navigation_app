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
// Accent colour: Google blue (Material 3 primary). White text stays readable
// on it. The name is kept so existing call sites keep working.
val RobotOrange = Color(0xFF0B57D0)
val LidarCyan = Color(0xFF00B8D4)
val DangerRed = Color(0xFFB3261E)

private val LightColors = lightColorScheme(
    primary = RobotOrange,
    primaryContainer = Color(0xFFD3E3FD),
    onPrimaryContainer = Color(0xFF041E49),
    error = DangerRed,
    background = Color(0xFFF8FAFD),
    surface = Color(0xFFFFFFFF),
    surfaceVariant = Color(0xFFE9EEF6),
    onSurface = Color(0xFF1F1F1F),
    onSurfaceVariant = Color(0xFF444746),
)

private val DarkColors = darkColorScheme(
    primary = Color(0xFFA8C7FA),
    error = Color(0xFFF2B8B5),
)

@Composable
fun VacuumTheme(content: @Composable () -> Unit) {
    MaterialTheme(
        colorScheme = if (isSystemInDarkTheme()) DarkColors else LightColors,
        content = content,
    )
}
