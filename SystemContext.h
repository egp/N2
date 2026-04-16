// SystemContext.h v1
#ifndef SYSTEM_CONTEXT_H
#define SYSTEM_CONTEXT_H

#include "SystemConfig.h"
#include "SystemRuntime.h"
#include "InputSnapshot.h"
#include "SystemSnapshot.h"

struct SystemContext {
  SystemConfig config;
  SystemRuntime runtime;
  InputSnapshot input;
  SystemSnapshot snapshot;
};

#endif

// SystemContext.h v1