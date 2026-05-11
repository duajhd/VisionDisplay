#pragma once

#include <QtCore/qglobal.h>

#if defined(VISIONDISPLAY_LIBRARY)
#  define VISIONDISPLAY_API Q_DECL_EXPORT
#else
#  define VISIONDISPLAY_API Q_DECL_IMPORT
#endif
