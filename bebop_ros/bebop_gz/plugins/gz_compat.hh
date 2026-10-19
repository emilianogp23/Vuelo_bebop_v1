// Compatibilidad entre Gazebo Harmonic (gz-sim8, ROS 2 Jazzy) y
// Gazebo Fortress (ignition-gazebo6, ROS 2 Humble).
//
// Fortress 6.x ya trae cabeceras <gz/...> y el alias de espacio de nombres
// gz::sim -> ignition::gazebo, pero NO las macros con prefijo GZ_/gz.
// Este archivo las define a partir de sus equivalentes ign/IGNITION.
// En Harmonic no hace nada.
#ifndef BEBOP_GZ_COMPAT_HH_
#define BEBOP_GZ_COMPAT_HH_

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>

#ifdef BEBOP_GZ_FORTRESS

#ifndef gzerr
#define gzerr ignerr
#endif
#ifndef gzwarn
#define gzwarn ignwarn
#endif
#ifndef gzmsg
#define gzmsg ignmsg
#endif
#ifndef gzdbg
#define gzdbg igndbg
#endif

#ifndef GZ_ADD_PLUGIN
#define GZ_ADD_PLUGIN IGNITION_ADD_PLUGIN
#endif
#ifndef GZ_ADD_PLUGIN_ALIAS
#define GZ_ADD_PLUGIN_ALIAS IGNITION_ADD_PLUGIN_ALIAS
#endif

#ifndef GZ_PROFILE
#define GZ_PROFILE IGN_PROFILE
#endif

#endif  // BEBOP_GZ_FORTRESS

#endif  // BEBOP_GZ_COMPAT_HH_
