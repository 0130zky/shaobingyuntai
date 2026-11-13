#ifndef SENTRY_STORAGE_H
#define SENTRY_STORAGE_H
#include "motion.h"
int Sentry_Load(SentryCalibration *c);
int Sentry_Save(const SentryCalibration *c);
#endif
