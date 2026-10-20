# colcon (per-package install prefixes, the default `colcon build` layout)
# only guarantees $COLCON_CURRENT_PREFIX is correct when sourcing a custom
# hook like this one; $AMENT_CURRENT_PREFIX can be left over/stale from a
# previously-sourced package. Prefer COLCON_CURRENT_PREFIX when present.
_bebop_gz_prefix="${COLCON_CURRENT_PREFIX:-$AMENT_CURRENT_PREFIX}"

# `gz sim <world>` and `model://` URI resolution only look directly
# inside each GZ_SIM_RESOURCE_PATH entry, not in its subdirectories, so
# worlds/ and models/ must be added separately (not their share/bebop_gz
# parent) or bare world filenames (e.g. "bebop1.sdf") and model URIs
# (e.g. "model://parrot_bebop_2") fail to resolve.
# Las variables IGN_GAZEBO_* son las de Gazebo Fortress (ROS 2 Humble).
for _bebop_gz_var in GZ_SIM_RESOURCE_PATH IGN_GAZEBO_RESOURCE_PATH; do
  ament_prepend_unique_value "$_bebop_gz_var" "$_bebop_gz_prefix/share/bebop_gz/worlds"
  ament_prepend_unique_value "$_bebop_gz_var" "$_bebop_gz_prefix/share/bebop_gz/models"
done
for _bebop_gz_var in GZ_SIM_SYSTEM_PLUGIN_PATH IGN_GAZEBO_SYSTEM_PLUGIN_PATH; do
  ament_prepend_unique_value "$_bebop_gz_var" "$_bebop_gz_prefix/lib/bebop_gz_plugins"
done

unset _bebop_gz_var
unset _bebop_gz_prefix

# Versión de Gazebo con la que se compilaron los plugins (harmonic o fortress).
# Solo se fija si el usuario no eligió otra.
if [ -z "${GZ_VERSION:-}" ]; then
  export GZ_VERSION=harmonic
fi
