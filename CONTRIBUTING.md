# Contributing

How we work with branches, commits and pull requests. How to build, run and test the software is in the
[README](README.md).

## Workflow

We use [GitHub Flow](https://docs.github.com/en/get-started/using-github/github-flow): `main` always works,
every change goes through a short-lived branch and a pull request.

1. **Issue.** Every change starts with an issue: what and why. Take an existing one or create a new one.
2. **Branch** from `main`, named `<issue number>-<short-description>`, e.g. `57-imu-calibration`.
   The *Create a branch* button in the issue (*Development* section) makes exactly this name.
3. **Commits** in the branch: small, each one builds. Write the messages in the same
   [Conventional Commits](#pull-request-title) format as the PR title, e.g. `fix(hw): reconnect after a UART timeout`.
   This is a recommendation, CI does not check it: the branch commits do not get into `main` (squash), but the
   history of the PR is easier to review.
4. **Pull request** into `main` as soon as there is something to show, mark it as *Draft* while it is not ready.
   The PR title follows [Conventional Commits](#pull-request-title), the description
   [links the issue](#links-to-issues) (`Closes #57`).
5. **Review and CI.** At least one approval, all checks green. Test on the robot before merging if the change
   affects it: [Testing your changes on the robot](README.md#testing-your-changes-on-the-robot).
6. **Squash and merge.** The whole PR becomes one commit in `main`, its message is the PR title. The branch is
   deleted after merging.

Keep the branch up to date with `main` (`git merge origin/main` or *Update branch* in the PR). One PR is one
topic: a fix found on the way goes to a separate PR.

## Pull request title

The PR title becomes the commit message in `main`, so it follows
[Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/). CI checks it. Use the same format for
the commit messages in the branch (recommended, not checked).

```
<type>(<scope>): <description>
```

- **type** — what kind of change:

  | Type | When |
  |---|---|
  | `feat` | new functionality |
  | `fix` | bug fix |
  | `docs` | documentation only |
  | `refactor` | code change that neither fixes a bug nor adds a feature |
  | `perf` | performance improvement |
  | `test` | tests only |
  | `build` | build, dependencies, Docker images, `deps.repos` |
  | `ci` | GitHub Actions |
  | `chore` | anything else that does not change the robot software (scripts, configs of tools) |
  | `revert` | revert of a previous change |

- **scope** (optional) — the part of the project, lower case: a package without the `kpi_rover_` prefix
  (`hw`, `description`, `bringup`, `localization`, `slam`, `worlds`), `nav`, `sim`, `docker`, `rover` (rover.sh),
  `rpi-setup`, or the name of an application in `src/apps/`.
- **description** — in English, imperative mood, lower case first letter, no period at the end:
  "add", not "added" or "adds".
- **Breaking change** (topics, parameters or launch arguments that applications use change incompatibly):
  `!` after the type or scope, and a `BREAKING CHANGE:` paragraph in the PR description.

Examples:
```
feat(hw): add IMU calibration on startup
fix(slam): use base_footprint as the base frame
docs: describe the network setup
build(docker): update ROS base image
feat(bringup)!: rename the lidar_model launch argument to lidar
feat(patrol): follow waypoints from a YAML file
```

GitHub adds the PR number to the commit message when squashing: `feat(hw): add IMU calibration (#58)`.

## Links to issues

Every PR is linked to an issue, CI checks it. Write in the PR description:

- `Closes #57` (also `Fixes`, `Resolves`) — the issue is closed automatically when the PR is merged;
- `Refs #57` — only linked, the issue stays open (the PR is one part of a larger task).

Several issues: one line per issue. An issue of another repository: `Closes KPI-Rover/other_repo#12`.

The link is shown in the issue and in the PR, and it gets into the commit in `main`: the squash commit
contains the PR title and description. A branch created with the *Create a branch* button is linked to the
issue as well. In commits inside the branch you can also mention the issue (`#57`), GitHub shows them in the
issue.

## Where code goes

- `src/platform/` — the robot platform, reviewed by the maintainers. Changes here affect every robot and every
  application.
- `src/apps/<app_name>/` — applications, see [Platform and applications](README.md#platform-and-applications).
  An application does not change the platform: if something is missing, open an issue for the platform.
- `src/external/` — not edited here: change the upstream repository, then the commit in `deps.repos`.
