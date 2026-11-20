package com.vacuum.nav.ui.theme

import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp

/**
 * Design tokens - Phase 1 of the redesign (design-baseline.json
 * FINDING-006/007). Every screen consumes these; no raw fontSize /
 * padding / corner values in screen code from this phase on.
 *
 * Type: a TIGHT seven-step scale (was14 scattered sp values: 10..32).
 * Off-scale values from the old UI were snapped onto the scale - that is
 * a deliberate, small visual change (overhaul mode).
 */
object Ts {
    val Micro = 10.sp      // footnotes, badges
    val Caption = 12.sp    // secondary lines
    val Body = 14.sp       // default copy
    val BodyStrong = 14.sp // same size, FontWeight.SemiBold at call site
    val Title = 16.sp      // row/card titles
    val Headline = 20.sp   // screen section heads
    val Display = 24.sp    // screen titles, hero numbers
    val XL = 28.sp         // hero values (battery)
}

/** Spacing rhythm: the only dp values screens may use for padding/gaps. */
object Sp {
    val Xxs = 4.dp
    val Xs = 8.dp
    val Sm = 12.dp
    val Md = 16.dp
    val Lg = 24.dp
    val Xl = 32.dp
}

/**
 * Radius lock (shape discipline): containers use one of these, actions use
 * Pill. No other corner values in screen code.
 */
object Rd {
    val S = 12.dp      // chips, small tiles
    val M = 16.dp      // cards, sheets
    val L = 24.dp      // hero cards
    val Pill = 50      // buttons (round)
}
