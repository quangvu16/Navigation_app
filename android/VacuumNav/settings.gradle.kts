// VacuumNav — Android robot-brain app. See docs/ARCHITECTURE.md.
pluginManagement {
    repositories {
        google()
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "VacuumNav"
include(":app")

// --- AppleDouble sidecar workaround (see docs/ASSUMPTIONS.md §9) -----------
// This project lives on an exFAT volume. macOS stores xattrs (e.g.
// com.apple.provenance) as "._*" AppleDouble sidecar files on exFAT, and AGP's
// directory-scanning tasks fail on them ("...is not a directory"). When the
// project root is on /Volumes/, every build directory goes to the internal
// disk instead. Force a location with -Pvacuum.buildDir=/somewhere.
val onExternalVolume = rootDir.absolutePath.startsWith("/Volumes/")
val vacuumBuildRoot: String = providers.gradleProperty("vacuum.buildDir").orNull
    ?: if (onExternalVolume) {
        "${System.getProperty("user.home")}/.cache/vacuum-nav/build"
    } else {
        "${rootDir.absolutePath}/build"
    }

gradle.beforeProject {
    val rel = if (path == ":") "root" else path.trimStart(':').replace(':', '/')
    layout.buildDirectory.set(file("$vacuumBuildRoot/$rel"))
}
