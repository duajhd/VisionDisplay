#pragma once

#include <QtCore/qglobal.h>

#if defined(VISIONTOOLS_LIBRARY)
#  define VISIONTOOLS_API Q_DECL_EXPORT
#else
#  define VISIONTOOLS_API Q_DECL_IMPORT
#endif
