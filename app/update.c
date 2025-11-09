#include "update.h"

static bool do_reset = false;

void schedule_reset(void)
{
  do_reset = true;
}

bool should_reset(void)
{
  return do_reset;
}
