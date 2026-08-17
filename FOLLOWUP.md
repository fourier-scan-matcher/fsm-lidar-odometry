# Known gaps and deferred work

Recorded during the ROS 2 port. None of these blocks the package; all were left
alone deliberately, and the reason is given so that the decision can be
revisited rather than rediscovered.

## An unmeasurable range is treated as no reading, but nothing is clamped

Infinity, not-a-number, zero and any negative range are all taken to mean the
sensor could not measure that ray, and are filled in from the rays either side.
A scan with nothing valid in it is refused.

What is still open is whether a ray that reports `range_max` exactly, which
some drivers use to mean "nothing within range" rather than "a surface at
exactly this distance", should be treated the same way. It currently is not.
Deciding that needs a survey of what drivers actually do.

## Twenty six core functions are shipped and exercised by nothing

The core header declares seventy three functions. The runtime path reaches
forty five. Of the twenty eight it does not, one is unreachable on purpose and
one has a test; the remaining twenty six are compiled into the package and run
by nothing at all.

They are listed and grouped, with the method used to arrive at the list, in the
header comment of `fsm_lidar_odometry/test/test_core_golden.cpp`. `tests/reachability/`
holds the probe, so the list can be recomputed whenever a call is added or
removed.

They are listed rather than deleted because deleting them is a decision about
how much of the original algorithm this package is obliged to carry, and that
is not a decision to take while porting.

## The intersection of an oblique ray and a wall is good to about 1e-9

A ray is represented by a point a hundred million metres along it, and where
that ray meets a wall perpendicular to the x-axis the height is worked out by
subtracting two quantities of that size. Almost all of it cancels, and about
eight of the sixteen digits go with it.

The remaining accuracy is a few parts in a thousand million, which is the same
order as the tolerance the cross version comparison is held to. It does not
threaten that comparison, both versions computing it the same way, but it does
bound how accurately any single scan can be matched, and it is not obvious from
reading the code.

The fix is to compute the height from the ray's own gradient rather than from
the far point, which is one line and changes the numbers.

## Gap filling never returns for a scan in which nothing was measured

Given a scan whose every reading is missing, the gap filling appends a list of
indices to itself while walking that same list, and allocates until the process
is killed.

Nothing reaches it: the matcher refuses a scan with nothing valid in it before
gap filling is called, and `test_scan_handling.cpp` holds that guard in place.
The function on its own is still unsafe for any other caller.

## The core header is duplicated in the `fsm` repository

`include/fsm_lidar_odometry/fsm_core.hpp` and `include/fsm.h` in
[`fsm`](https://github.com/li9i/fsm) are two copies of the same algorithm that
have drifted apart. Differences found while porting:

- `fsm` carries measurement noise parameters, a terminal constraint and an
  early gear-up feature that this copy does not.
- This copy carries the transform accumulation, scan subsampling and gap
  filling that the ROS wrapper needs, which `fsm` does not.
- One index variable that selects the best candidate has a different type in
  each copy, which can select a different candidate.

Only this copy received the eight corrections listed in the readme, and only
this copy has the angular ray search. The `fsm` copy still carries all eight
defects and the windowed search alone.

Reconciling the two is worth doing and is a job in itself. It was explicitly
out of scope for the port.

## The two formatting linters are switched off

`ament_cmake_uncrustify` and `ament_cmake_cpplint` impose a brace and wrapping
style this package has never used. Satisfying them means reformatting 3800
lines of inherited numerical code, which is the riskiest change available for
no behavioural gain. Every other check `ament_lint_common` provides is on.

## The recovery path is untested across versions

The matcher falls back on a randomly seeded search when a match goes badly.
`rng_seed` pins it so a run can be reproduced within one build, and there is a
unit test for that, but the cross version comparison deliberately never
exercises it: the synthetic motion is small enough that recovery never fires.

The reason is that the standard library does not specify how a distribution
turns random bits into a number, so two implementations may differ even from
the same seed. Comparing the recovery path across versions would first need
that conversion written out explicitly rather than taken from the library.

## The comparison harness does not assert that recovery stayed quiet

Because recovery cannot reproduce across builds, a comparison run in which it
fired would be meaningless, and the harness is supposed to fail rather than
report a difference it cannot explain. It does not check.

The node now reports the count truthfully and logs a warning whenever a match
needed recovery, so the evidence exists in the run output on the ROS 2 side; no
warning appears in any of the six scenarios. What is missing is the harness
reading it and failing on it, on both sides.

The reason it was left is cost against benefit. Closing it properly means
adding the same reporting to the ROS 1 reference build, which means rebuilding
the pinned container and regenerating every reference recording, disturbing a
proof that currently passes. The six scenarios agree to better than 4e-15,
which is itself strong evidence that the random path never fired on either
side, since it could not have agreed to that precision if it had.

## The recovery seed exists on the port branch only

`rng_seed` was meant to be threaded through on the correction branch as well,
so that both sides of the comparison could pin the recovery search. It reached
only the port branch. The correction branch still seeds from hardware entropy
with no way to pin it.

This changes nothing about the reference recordings, which were captured from a
build where recovery never fired, so the seed would have had nothing to
influence. Adding it now would invalidate a working reference for no gain. It
matters only if the recovery path is ever compared across versions, which the
entry above already says needs other work first.

## A consumer's own arithmetic settings govern the matcher it links

The package now exports targets a downstream package can link, and nearly all of
the matcher is in the header, so a consumer compiles those functions itself, at
its own flags, and its own copies are the ones its calls reach. The figures this
package publishes were measured at the settings its own `CMakeLists.txt` sets and
nothing characterises what tolerance holds at any other.

The alternative is to export the arithmetic settings as a usage requirement of
the exported targets, which imposes them on a consumer that may have its own
reasons for the settings it chose. Left to the readme to warn about instead.

## The container's shell trim has not been through a build

Three faults were corrected in this container. The shell profile looked for the
workspace under a path that resolved to `/home/`, so no shell in the shipped
image had the workspace on it. The entrypoint took ownership of the two shared
directories, which matters wherever a compose file bind mounts host directories
over them. The build deleted the rosdep sources list and fetched an identical
copy over a link with no retry.

All three are now in a built image and verified in it: a login shell lists this
package's executable and `colcon_cd` is a function, the entrypoint carries no
`chown` of the shared directories, and the image holds the sources list the base
image ships, byte for byte the file the deleted step used to fetch.

What that build predates is the general half of the shell profile, cut from
eleven hundred and fifty six lines of somebody's personal configuration to fifty
six this package owns, and the entrypoint's comments about an X authority file
and a workspace mount that no compose file declares, removed with it. Both were
proved by mounting the new file into the built image over its own copy, and
neither has been through an image build.

Two network fetches remain in the build, `rosdep update` and the lookup that
finds the current ROS apt source release. Both genuinely need the network and
neither retries.

## The container cannot be told to run the node

Its main process is a shell. The entrypoint drops from root to the container's
user and execs whatever it was given without sourcing the ROS environment first,
so giving the compose file a `command` that runs the node would meet `ros2:
command not found`. The environment comes from the user's shell profile and from
nowhere else.

So `docker compose up` starts a container sitting in a shell rather than a
running node, and the node has to be launched into it afterwards. The readme
says so and gives the two arguments an `exec` needs. Making the container run
the node on its own means sourcing the environment in the entrypoint, which is a
small change to a file inherited verbatim from an upstream project and is left
for a decision about what the shipped container is for.
