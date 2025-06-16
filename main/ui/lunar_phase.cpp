#include "lunar_phase.hpp"

#include <cmath>

// Determine the Moon Phase and orbital positions for the specified time.
LunarPhase LunarPhaseCalculator::calculate(time_t t)
{
  const double jDate = julianDate(t);

  // Calculate illumination (synodic) phase.
  // From number of days since new moon on Julian date MOON_SYNODIC_OFFSET
  // (1815UTC January 6, 2000), determine remainder of incomplete cycle.
  double phase = (jDate - MOON_SYNODIC_OFFSET) / MOON_SYNODIC_PERIOD;
  phase -= floor(phase);

  // Calculate age and illumination fraction.
  return static_cast<LunarPhase>(static_cast<int>(phase * 8 + 0.5) % 8);
}

// Determine Julian date from Unix time.
// Provides marginally accurate results with older Arduino 4-byte double.
double LunarPhaseCalculator::julianDate(time_t t)
{
  return (t / 86400.0L + 2440587.5);
}
