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

## The container has not been rebuilt since three faults were corrected in it

The shell profile looked for the workspace under a path that resolved to
`/home/`, so no shell in the shipped container had the workspace on it. The
entrypoint took ownership of the two shared directories, which matters wherever
a compose file bind mounts host directories over them. The build deleted the
rosdep sources list and fetched an identical copy over a link with no retry.

All three are corrected. The shell one was proved by mounting the file into a
built image of the package descended from this one, whose entrypoint drops from
root the same way. The other two need a build to exercise, and no build has run:
`raw.githubusercontent.com` is rate limiting this machine, which is what exposed
the rosdep step in the first place.

Two network fetches remain in the build, `rosdep update` and the lookup that
finds the current ROS apt source release. Both genuinely need the network and
neither retries.
