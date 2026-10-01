# PathPlannerLib (temporary copy for WPILib 2027 alpha 7)

This is the Java source of PathPlannerLib, copied into the robot project because
there is no official PathPlannerLib release for WPILib 2027 alpha 7 yet.

- **Upstream:** https://github.com/mjansen4857/pathplanner, branch `2027`,
  commit `61f2fad9d3dd800d99577bdd52afded0404abcff` (2026-07-02), which targets
  WPILib 2027 alpha 6.
- **License:** MIT, see [LICENSE](LICENSE). The bundled `org/json/simple` is
  json-simple (Apache 2.0), which upstream ships the same way.
- **Our changes:** [alpha7-port.patch](alpha7-port.patch) (16 files), all to
  follow alpha 6 → alpha 7 API changes:
  - Constants renamed to ALL_CAPS (`Rotation2d.kZero` → `ZERO`,
    `k180deg`/`kPi` → `PI`, `kCW_90deg` → `CW_90DEG`, `Translation2d.kZero` →
    `ZERO`, `Pose2d.kZero` → `ZERO`)
  - `Alert` moved to `org.wpilib.util`, and its 3-argument constructor changed
    from `(group, text, level)` to `(id, text, level)`, so the alerts now use
    the 4-argument `(group, id, text, level)` form
  - `Pair` moved to `org.wpilib.util`
  - `SendableChooser` → `Selectable` (`setDefaultOption`/`addOption` →
    `addDefault`/`add`), so `AutoBuilder.buildAutoChooser()` now returns a
    `Selectable<Command>`
  - `Translation2d.getAngle()` now returns `Optional<Rotation2d>`; we use
    `.orElse(Rotation2d.ZERO)`, which matches the old behavior for zero-length
    vectors
  - `SwerveSetpointGenerator`: `SwerveModuleVelocity.optimize()` and
    `SwerveDriveKinematics.desaturateWheelVelocities()` return new values in
    2027 instead of modifying their arguments, and upstream still called them
    the 2026 way (the results were silently thrown away). WPILib's `@NoDiscard`
    compile check caught this; we now use the returned values. This is an
    upstream bug, so check whether the official release fixes it too.

It is compiled as part of the robot code via the `sourceSets` entry in
`build.gradle`.

## Removing it

Once PathPlanner publishes a release for the WPILib version this project uses:

1. Delete this `vendor/pathplannerlib` folder.
2. Remove the `vendor/pathplannerlib` `sourceSets` line from `build.gradle`.
3. Install PathPlannerLib through WPILib's vendor dependency manager.

The robot code imports `com.pathplanner.lib.*` exactly as it would with the
official library, so no robot code changes should be needed beyond any API
changes in that release.
