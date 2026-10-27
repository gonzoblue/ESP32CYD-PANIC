#include "backlight_policy.h"

static_assert(!BacklightPolicy::shouldBlank(true, 89, 59999), "Stay lit before 60s");
static_assert(BacklightPolicy::shouldBlank(true, 89, 60000), "Blank at 60s below 90%");
static_assert(!BacklightPolicy::shouldBlank(true, 90, 60000), "Stay lit at 90%");
static_assert(!BacklightPolicy::shouldBlank(true, 100, 120000), "Stay lit at full charge");
static_assert(!BacklightPolicy::shouldBlank(false, 0, 120000), "Stay lit without a valid reading");
static_assert(!BacklightPolicy::shouldBlank(true, 89, 0), "Wake resets inactivity");
static_assert(!BacklightPolicy::refreshDue(4999, 0), "No early dark redraw");
static_assert(BacklightPolicy::refreshDue(5000, 0), "Redraw every 5s");
static_assert(!BacklightPolicy::refreshDue(3998, UINT32_MAX - 1000), "Handle timer rollover before deadline");
static_assert(BacklightPolicy::refreshDue(3999, UINT32_MAX - 1000), "Handle timer rollover at deadline");
