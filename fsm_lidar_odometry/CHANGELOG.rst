^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package fsm_lidar_odometry
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Declares ``libfftw3-dev`` as a build dependency and ``libfftw3`` as a run
  dependency. ``ros/rosdistro`` #54634 split the ``libfftw3`` rule: that key
  now installs the run time libraries only, and the new key ``libfftw3-dev``
  installs the headers. Thus ``rosdep`` installs each dependency, and the
  install steps and ``docker/Dockerfile`` no longer install ``libfftw3-dev``
  by name or skip a key.
* Unpacks the spectrum of an odd count of rays correctly. It unpacked each
  spectrum as if its length was even. Thus at an odd length it removed the
  imaginary part of the highest coefficient and gave one coefficient too few.
  This changed each rotation that the matcher found for a scan of an odd count
  of rays. The 1073 rays of the rb1 recording are such a count at the shipped
  ``size_scan`` of 0. The same defect was in ``fsm_localisation``. At an odd
  length the arithmetic no longer agrees with the ROS 1 package.
* Ports the package from ROS 1 Kinetic to ROS 2. The ROS 1 version was only
  ever available from source, so there is no earlier changelog to continue.
* The package is renamed from ``fsm_lo`` to ``fsm_lidar_odometry``. The node,
  the launch file, the default node name, the include directory, the C++
  namespace and the four service names all follow it. Topic and parameter
  names are unchanged.
* The four services take and return ``std_srvs/srv/Trigger`` instead of taking
  and returning nothing, so callers now get a success flag and a message.
* Output is stamped from the incoming scan rather than from the clock at the
  moment of processing. The reported twist follows from the interval between
  scan stamps, so it no longer varies with machine load. A replay of a
  recording gives the same stamps every time. A match that needs the recovery
  search can still give a different pose on each replay, as ``rng_seed`` below
  says.
* The scan subscription's reliability and depth are exposed as the
  ``scan_qos_reliability`` and ``scan_qos_depth`` parameters. The defaults
  mirror ROS 1, so a ``best_effort`` sensor driver needs
  ``scan_qos_reliability`` set to match or no scans arrive.
* ``size_scan`` accepts ``0``, now the default, which matches every ray the
  scan carries instead of requiring a fixed ray count. A scan carrying fewer
  ranges than a non-zero ``size_scan`` is refused with a warning instead of
  being read past its end.
* Adds the ``ray_search`` parameter, selecting between the new ``angular``
  search, now the default, and the ``windowed`` search the algorithm shipped
  with. Over 13908 matches at ``size_scan: 360``, ``angular`` completes a match
  in 17.9 ms against ``windowed``'s 25.4 ms, median.
* Adds the ``rng_seed`` parameter. ``0`` draws the recovery search from
  hardware entropy. Any other value seeds the generator of each executor thread
  once, at the first draw on that thread. Any executor thread may run a match,
  so which draw a recovery gets depends on the thread schedule. Two runs over
  the same input can still differ where a recovery was needed: on a recording
  from a real robot, two runs at ``rng_seed`` 1 over 2000 scans differed from
  step 230.
* Parameters outside their permitted range are refused at start-up with an
  explanation instead of tripping an assertion that release builds compile
  away.
* The matching core reports through an installable diagnostic sink rather than
  writing to a terminal. The node forwards it to the ROS log. Stage timings are
  compiled out unless the core is built with ``FSM_LO_TRACE``.
* The node runs on a multi-threaded executor, so a service call is not held up
  behind a scan being matched.
* Frame id defaults no longer carry a leading slash, which tf2 rejects.
* ``std_msgs/Header`` has no sequence number in ROS 2, so published messages no
  longer carry one.
* Corrects eight defects present in the ROS 1 version:

  * The rotation matrix accumulating the trajectory mixed single and double
    precision, so it was not a rotation. Its determinant was 1.7e-8 from unity,
    and since it is composed once per scan the error grew along the whole path.
  * The initial pose supplied through ``set_initial_pose`` was built in single
    precision, degrading it to about seven significant figures.
  * The convergence criterion of the translation stage took a single precision
    square root of a double.
  * Gap filling read one element before the start of an empty list whenever a
    scan contained no invalid returns at all, which crashed the node.
  * With ``pose_estimate_topic`` unset, the fallback was written to the wrong
    field, so the node threw during construction and could not start at all
    without that parameter.
  * Frame id fallbacks carried a leading slash, which tf2 rejects.
  * Two assertions checked that an unsigned value was at least zero.
  * Output was stamped from the wall clock rather than from the scan.

* Contributors: Alexandros Filotheou
