// Test LaunchToOrbit script.

runpath ("LaunchToOrbit V03.ks",
  80,              // Orbital altitude.
  0,                // Orbital inclination.
  "NORTH",          // Launch direction.
  130,             // Turn start altitude.
  10,               // Turn pitchover.
  30,               // Steering duration.
  "NOWARP",         // Warp type.
  10,               // Launch countdown duration.
  "NOSYNC").        // Launch sync period.
