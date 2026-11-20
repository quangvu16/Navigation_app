# MOBILE-DESIGN.md — VacuumNav design system (post-redesign)

Written by the `/mobile-redesign-app` close-out (six approved phases,
2026-11-20). Future sessions: read this before touching screens — it is
the inherited system. The regression baseline lives at
[`design-baseline.json`](../design-baseline.json) (Design **A**, Slop **A**,
all 10 findings fixed).

Brief: **"Use google theme with icons"** — Material 3, Google semantic
palette, Material Symbols throughout, zero emoji.

## Nav Read

```
AppNav (Scaffold + bottom NavigationBar, 5 tabs — order/icons frozen)
├── home        HomeScreen        greeting + robot card + Routines rows
│   ├── maps        sub="maps"        MapsScreen      (TopAppBar back)
│   ├── history     sub="history"     HistoryScreen    (TopAppBar back)
│   └── diagnostics sub="diagnostics" DiagnosticsScreen(TopAppBar back)
├── map         MapScreen         2D map canvas, long-press goal, zones/drive entry
├── drive       DriveScreen       joystick + lidar modes + dock + E-STOP
├── zones       ZonesScreen       no-go zones editor
└── schedule    SchedulesScreen   timed cleans
```

- **Back contract (Phase 6):** `BackHandler(enabled = sub.isNotEmpty())`
  in [AppNav.kt:60](../android/VacuumNav/app/src/main/java/com/vacuum/nav/ui/AppNav.kt#L60)
  pops `sub` → Home; hardware/gesture back on a root tab is the OS default
  (leave-app). Tab click also clears `sub`. Never add a 6th tab — Maps and
  History are Home sub-screens on purpose.
- **Never change silently:** tab order (Home/Map/Drive/Zones/Schedule),
  route keys (`home|map|drive|zones|schedule` + `maps|history|diagnostics`),
  the TopAppBar back on each sub-screen (it must agree with BackHandler).
- **Link-error banner** sits above the active screen whenever the link is
  `ERROR_LINK` (FINDING-005) — keep it app-wide, not per-screen.

## Reads & dials (how to judge a screen)

- **Reads:** one glance must answer — robot state (Idle/Cleaning/E-STOP),
  battery, link transport (SIM/Wi-Fi/BT chip), nearest obstacle distance.
  Map screen: pose, coverage, zone overlays readable over the grid.
- **Dials:** press feedback on every large CTA (`Modifier.pressScale()`),
  chips select with color+weight (not color alone), sliders/toggles are
  Material3 defaults. Motion = spring press + fade/expand banner only;
  no decorative animation.

## Token locations

| Token | File | Contents |
|---|---|---|
| `Ts` (type scale) | [Tokens.kt](../android/VacuumNav/app/src/main/java/com/vacuum/nav/ui/theme/Tokens.kt) | Micro 10 → XL 28, seven steps + BodyStrong |
| `Sp` (spacing) | same | Xxs 4 / Xs 8 / Sm 12 / Md 16 / Lg 24 / Xl 32 |
| `Rd` (radius) | same | S 12 / M 16 / L 24 / Pill 50 — containers vs actions |
| Semantic colors | [Theme.kt](../android/VacuumNav/app/src/main/java/com/vacuum/nav/ui/theme/Theme.kt) | `lightColorScheme`/`darkColorScheme` + named accents (`RobotOrange` = Google blue `0xFF0B57D0`, `DangerRed` = `0xFFB3261E`, `Positive*`, map colors) |

Rules inherited from the redesign:

1. **No inline hex colors in screens** — named color vals in Theme.kt only
   (FINDING-001; grep `0xFF` under `ui/` excluding theme/ must stay 0 —
   the one exception is the byte-mask `and 0xFF` in MapScreen.kt, which is
   not a color).
2. **No raw sp/dp in screens** — `Ts.*`, `Sp.*`, `Rd.*` only
   (FINDING-006/007).
3. **No emoji** — Material `Icons.Filled.*` everywhere (FINDING-008).
4. **States on every data screen** — loading (real spinner + copy,
   FINDING-005), empty, error (banner or inline), disabled
   (FINDING-003/004).
5. **Signatures preserved** (from baseline): deadman joystick, always-on
   E-STOP, long-press goal, Manual/Avoid/Follow chips, tab order,
   warning-banner semantics.
6. **Destructive actions confirm** — History Clear, Maps Delete, Zones
   Clear all (and anything new) raise an AlertDialog naming the
   consequence first (run-2 FINDING-014).
7. **Every data-heavy screen scrolls** — Home included; no fixed Columns
   with `weight()` spacers (run-2 FINDING-013; weights are illegal in
   scrollable columns).

## Regression greps (run after any screen change)

```bash
# inline hex colors outside theme (only hit allowed: MapScreen and 0xFF mask):
grep -rn "0xFF" android/VacuumNav/app/src/main/java/com/vacuum/nav/ui \
  --include=*.kt | grep -v /theme/ | grep -v "and 0xFF"
# emoji / manual back / raw font sizes:
grep -rnP "[\x{1F300}-\x{1FAFF}\x{2600}-\x{27BF}]" android/VacuumNav/app/src/main/java
grep -rn "BackHandler\|fontSize = [0-9]" android/VacuumNav/app/src/main/java/com/vacuum/nav/ui
```

All must be empty (BackHandler in AppNav.kt is the sanctioned one).

## Unverified (needs a device)

- Live dark-mode walkthrough, 1.3× font scale, IME behavior.
- Cold Trunk Test of interior screens (reconstructed, not dropped fresh).
- See `design-baseline.json → unverified` for the full list.
