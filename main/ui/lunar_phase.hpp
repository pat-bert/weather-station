#ifndef LUNAR_PHASE_HPP_INCLUDED
#define LUNAR_PHASE_HPP_INCLUDED

#include <ctime>

#define MOON_SYNODIC_PERIOD 29.530588853 // Period of moon cycle in days.
#define MOON_SYNODIC_OFFSET 2451550.26   // Reference cycle offset in days.

enum class LunarPhase : uint8_t
{
  NewMoon,
  WaxingCrescent,
  FirstQuarter,
  WaxingGibbous,
  FullMoon,
  WaningGibbous,
  LastQuarter,
  WaningCrescent
};

class LunarPhaseCalculator
{
public:
  static LunarPhase calculate(time_t t);

protected:
  static double julianDate(time_t t);
};

#endif
