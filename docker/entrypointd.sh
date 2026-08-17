#!/bin/sh

# https://github.com/sudo-bmitch/docker-base/blob/main/bin/entrypointd.sh
# Copyright: Brandon Mitchell
# License: MIT

set -e
# Handle a kill signal before the final "exec" command runs
trap "{ exit 0; }" TERM INT

# strip off "/bin/sh -c" args from a string CMD
if [ $# -gt 1 ] && [ "$1" = "/bin/sh" ] && [ "$2" = "-c" ]; then
  shift 2
  eval "set -- $1"
fi

if [ -f /.volume-cache/volume-list.already-run ]; then
  rm /.volume-cache/volume-list.already-run
fi

for ep in /etc/entrypoint.d/*; do
  ext="${ep##*.}"
  if [ "${ext}" = "env" ] && [ -f "${ep}" ]; then
    # source files ending in ".env"
    echo "Sourcing: ${ep} $@"
    set -a && . "${ep}" "$@" && set +a
  elif [ "${ext}" = "sh" ] && [ -x "${ep}" ]; then
    # run scripts ending in ".sh"
    echo "Running: ${ep} $@"
    "${ep}" "$@"
  fi
done

# inject certificates
if [ -d /etc/certs.d ]; then
  add-certs
fi

# load any cached volumes
if [ -f /.volume-cache/volume-list -a ! -f /.volume-cache/volume-list.already-run ]; then
  load-volume -a
fi

# Default to the prior entrypoint if defined
if [ -n "$ORIG_ENTRYPOINT" ]; then
  set -- "$ORIG_ENTRYPOINT" "$@"
fi

# run a shell if there is no command passed
if [ $# = 0 ]; then
  if [ -x /bin/bash ]; then
    set -- /bin/bash
  else
    set -- /bin/sh
  fi
fi

# include tini if requested
if [ -n "${USE_INIT}" ]; then
  set -- tini -- "$@"
fi

# ------------------------------------------------------------------------------
USER=fsm_lidar_odometry

# fix stdout/stderr permissions to allow non-root user
chown --dereference $USER "/proc/$$/fd/1" "/proc/$$/fd/2" || :

# A host uid that is not the container user's own owns whatever it bind mounts
# here, and this package's own compose file mounts nothing at all. Another
# compose file may. Transferring ownership to the container's user would only
# shuffle the problem, so others are granted rwX instead.
chmod -R o+rwX /home/$USER/ros2_ws/src

# Rights, not ownership. Both directories are already owned by $USER inside the
# image, so the chown that once stood here was redundant there; where they are
# bind mounted it took ownership of the host's own directories away from whoever
# checked them out, and it does not need to. o+rwX is enough for the container's
# user to read and write in them whatever uid the host gave them.
chmod -R o+rwX /home/$USER/shared-input
chmod -R o+rwX /home/$USER/shared-output

# Drop from root to fsm_lidar_odometry
set -- gosu $USER "$@"
# ------------------------------------------------------------------------------

exec "$@"
