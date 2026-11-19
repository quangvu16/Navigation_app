// Root build file. Versions pinned to the current stable set (2026-11):
// AGP 9.5.1 / Gradle 9.9.1 (wrapper). AGP 9 has BUILT-IN Kotlin support —
// do not add org.jetbrains.kotlin.android here (AGP refuses it). The Compose
// compiler plugin is still a separate plugin and IS required (Kotlin 2.0+).
plugins {
    id("com.android.application") version "9.5.1" apply false
    id("org.jetbrains.kotlin.plugin.compose") version "2.3.21" apply false
}
