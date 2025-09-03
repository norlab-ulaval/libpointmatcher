# Contributing to _libpointmatcher_

## Bug Reporting

Please use our [github's issue tracker](http://github.com/norlab-ulaval/libpointmatcher/issues) to
report bugs. If you are running the library on Ubuntu, copy-paste the output of the
script [listVersionsUbuntu.sh](https://github.com/norlab-ulaval/libpointmatcher/blob/master/utest/listVersionsUbuntu.sh)
to simplify the search of an answer.

## Code Contributions

Libpointmatcher codebase now
integrate [norlab-build-system (NBS)](https://github.com/norlab-ulaval/norlab-build-system)
and [norlab-shell-script-tools (N2ST)](https://github.com/norlab-ulaval/norlab-shell-script-tools).
`NBS` is a build-infrastructure-agnostic build system custom-made to meet our needs in robotic
software engineering at NorLab and `N2ST` is a library of shell script functions as well as a shell
testing tools leveraging _**bats-core**_ and _**docker**_ .
`N2ST` purpose is to speed up shell script development and improve reliability.

`NBS` is deployed on our [TeamCity](https://www.jetbrains.com/teamcity/) continuous
integration/deployment server and oversees protected branches of
the [libpointmatcher](https://github.com/norlab-ulaval/libpointmatcher) GitHub repository:

- The `develop` branch can only be merged through a pull-request from any `<feature>` branches. Any
  contributor can submit a pull request to the `develop` branch;
- the `release` branch is a revision and preparation branch where we can freeze the codebase in a
  given state without stalling to `develop` branch progression;
- The `master` branch can only be merged through a pull-request from the `release` branch. Only
  repository admin can submit a PR to the `master` branch.

In any cases, submitting a pull request to `develop` or `master` will trigger a build/test
configuration on our build system and the pull request will be granted if the build/test run
succeed.

**Current build matrix:**
`[latest] x [x86, arm64] x [ubuntu] x [bionic, focal, jammy] x [Release, RelWithDebInfo, MinSizeRel]`

### Development Workflow

To speed up the development process, you can run the build system localy on your workstation and
have access to stacktrace and build log.
It support multi-OS and multi-architecture through docker container.

#### Install _libpointmatcher-build-system_ Dependencies

```shell
cd <path/to/libpointmatcher>

# If libpointmatcher is already cloned, fetch the NBS and N2ST submodule 
git submodule update --remote --recursive --init

cd ./build_system/lpm_utility_script

# Execute docker tools install script i.e. docker daemon, docker compose, docker buildx 
bash lpm_install_docker_tools.bash

# Configure a multi-architecture docker builder
bash lpm_create_multiarch_docker_builder.bash
```

#### _libpointmatcher_ Development › To Execute Build/Test Step Locally

```shell
cd <path/to/libpointmatcher>/build_system

# Run the build matrix as specified in ".env.build_matrix.libpointmatcher" 
#   on native architecture using "ci_PR" service 
bash lpm_crawl_libpointmatcher_build_matrix.bash --fail-fast -- build ci_PR

# Run a specific case using build flags with multi-architecture 
# virtualization using "ci_PR_amd64" and "ci_PR_arm64" services 
bash lpm_crawl_libpointmatcher_build_matrix.bash \
            --repository-version-build-matrix-override latest \
            --os-name-build-matrix-override ubuntu \
            --cmake-build-type-build-matrix-override RelWithDebInfo \
            --ubuntu-version-build-matrix-override jammy \
            --fail-fast \
            -- build ci_PR_amd64 ci_PR_arm64

# Read the help for details
bash lpm_crawl_libpointmatcher_build_matrix.bash --help
```

Note: To assess the state of the codebase, even for cases that are known the break the build,
execute `lpm_crawl_libpointmatcher_build_matrix.bleeding.bash` with build
matrix `.env.build_matrix.libpointmatcher.bleeding`.
The stable build matrix used for release is `.env.build_matrix.libpointmatcher`.

#### Build System Development

```shell
cd <path/to/libpointmatcher>/build_system/tests/
 
# To execute docker dryrun and configuration tests
bash run_all_docker_dryrun_and_config_tests.bash

# To execute shell script tests
bash run_bats_core_test_in_n2st.bash

# To spin a container in interactive mode with the codebase cloned but not compiled  
cd ./tests_docker_interactive/
bash build_and_run_IamBuildSystemTester.bash bash
```

#### Build System Notes

- `lpm_crawl_dependencies_build_matrix.bash` execute the build matrix for the libpointmatcher
  dependencies.
  It's not required to build them locally as they are pre-build by our TeamCity server periodically
  push to dockerhub.
  When executing `lpm_crawl_libpointmatcher_build_matrix.bash`, the `libpointmatcher-dependencies`
  docker images are pull and used as base image for the `libpointmatcher-[ci_PR_test|release]`
  images.
- About `libpointmatcher/.github/workflow/` vs `libpointmatcher/build_system/` logic: Those are
  separate build logic.
  `.github/workflow/` was community contributed and as the responsibilities of building
  python-binding and pushing packages.
  For this reason, it run a one-dimension build matrix: multiple python version, single OS version,
  single arch (x86) and
  single compile flag which GitHub action computing resources can handle just fine.

## Commit Messages
This is optional for now but will eventually move our release workflow to semantic-versioning.
See [Commit Message References](commit_msg_reference.md) for details. 

## Note For Repository Admins

### About Release Branch And Pull Request To Master Branch

- Only repository admins have the privilege to `push/merge` on the default branch (ie: `master`)
  and the `release` branch.
- Keep PR in `draft` mode until all the release reviewers are ready to push the release.
- Once a PR from `release` -> `master` branch is created (not in draft mode),
  - it triggers the _build-system_ test
  - (in-progress) and it triggers the _semantic release automation_

## Writing Documentation
The documentation is located in the /doc folder and uses the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/) Markdown framework. 
