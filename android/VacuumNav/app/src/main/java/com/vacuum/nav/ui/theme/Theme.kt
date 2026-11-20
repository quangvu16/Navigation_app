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

// ---------------------------------------------------------------------------
// Semantic tokens - Phase 2 of the redesign (design-baseline.json
// FINDING-001): single sources for the literals that used to live inline
// in screen files. Map/data colors are data semantics, not brand - they
// keep their values but must be referenced through these names.
val Positive = Color(0xFF7BE0B0)        // status OK on dark surfaces
val PositiveSolid = Color(0xFF1E8E3E)   // status OK on light surfaces
val TargetGreen = Color(0xFF4CAF50)     // tracked-target marker on the map
val HandleBlue = Color(0xFF2196F3)      // zone editor handles
val PauseGray = Color(0xFF9E9E9E)       // paused/inactive controls
val HeroTop = Color(0xFF37474F)         // hero-card gradient, light theme
val HeroBottom = Color(0xFF263238)
val MapGridLine = Color(0xFFE2E6EA)     // editor grid hairlines
val RoomTint = Color(0xFFB3E5FC)        // room backdrop in the zone editor
val MapEditorBg = Color(0xFFF4F6F8)     // zone-editor canvas background
val MapEditorOutline = Color(0x66888888) // room outline in the editor

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
