# Generating a new release of PX4 Drone Autopilot
## Dependencies
```
$ apt install catkin python-bloom
```

## TL;DR
As a one-time task the release repository must be created, after verifying the defaults in the
`Makefile`.
```shell
$ make init-release RELEASE_ARGS="-F private=false" RELEASE_ORG="PX4-release" RELEASE_REPO="Firmware"
$ make release
```

## Release
```shell
$ make -n release VERSION="1.11.0"
make tag-release CATKIN_ARGS="--tag-prefix v --non-interactive"
make[1]: Entering directory '~/src/upstream/PX4/Firmware'
catkin_prepare_release --version 1.11.0 --tag-prefix v --non-interactive
make[1]: Leaving directory '~/src/upstream/PX4/Firmware'
make bloom-release
make[1]: Entering directory '~/src/upstream/PX4/Firmware'
git clone --depth 1 git@github.com:PX4-release/Firmware.git /tmp/tmpgUakMS
mkdir -p /tmp/tmpgUakMS/melodic/src/lib/version
~/src/upstream/PX4/Firmware/src/lib/version/px_update_git_header.py /tmp/tmpgUakMS/melodic/src/lib/version/build_git_version.h
git -C /tmp/tmpgUakMS add melodic/src/lib/version/build_git_version.h
git -C /tmp/tmpgUakMS commit -m "Generated git metadata patch for melodic release"
git -C /tmp/tmpgUakMS push
rm -Ir /tmp/tmpgUakMS
ROSDISTRO_INDEX_URL=https://raw.githubusercontent.com/ros/rosdistro/master/index-v4.yaml bloom-release \
	--override-release-repository-url https://github.com/PX4-release/Firmware.git \
	--override-release-repository-push-url git@github.com:PX4-release/Firmware.git \
	--rosdistro melodic --track melodic px4
make[1]: Leaving directory '/home/test/src/upstream/PX4/Firmware'
```

## Internal ROS Buildfarm
This tooling can be used to build packages with an internal buildfarm.

The release process can be customized by using [`direnv`](http://direnv.net/) and a `.envrc` file with local changes.
```shell
export RELEASE_REPO=MyPX4-Firmware-fork
export RELEASE_ORG=MyOrg-release
export RELEASE_ARGS="-F private=true"
export RELEASE_PUSH_URL="git@github.com:${RELEASE_ORG}/${RELEASE_REPO}.git"
export RELEASE_REPO_URL="https://github.com/${RELEASE_ORG}/${RELEASE_REPO}.git"
export ROS_DISTRO=melodic
export ROSDISTRO_INDEX_URL="http://repo.example.com/myorg/rosdistro/index.yaml"
export ROS_PACKAGE=px4
```

## Tag release
The `tag-release` makefile target uses `catkin_prepare_release` to update `package.xml` and tag a
new release. It can bump the version number by major, minor or by default the patch level.
`make tag-release BUMP=minor` will bump the version from 1.11.0 -> 1.12.0

```shell
$ make tag-release VERSION="1.11.0"
Prepare the source repository for a release.
Repository type: git
Found packages: px4
Warning: the following packages do not have a changelog file or entry for version '1.11.0': px4
Checking if working copy is clean (no staged changes, no modified files, no untracked files)...
Rename the forthcoming section of the following packages to version '1.11.0': 
Bump version of all packages from '1.0.0' to '1.11.0'
Committing the package.xml files...
[master 465d287f9b] v1.11.0
 1 file changed, 1 insertion(+), 1 deletion(-)
Creating tag 'v1.11.0'...
You can use the following commands to manually push the changes to the remote repository:
  /usr/bin/git push origin master
  /usr/bin/git push origin v1.11.0
The release of the source repository has been prepared successfully but the changes have not been pushed yet. After pushing the changes manually the next step will be 'bloom-release'.
```

After tagging the release, manually push the release to the remote repository or remove `--no-push`
from `CATKIN_ARGS` in the `Makefile`

```shell
$ /usr/bin/git push origin master
$ /usr/bin/git push origin v1.11.0
```

## Bloom Release
This makefile target generates a patched header file `src/lib/version/build_git_version.h` that
contains git metadata and pushes it to the release repository. It then runs `bloom-release` to
make the release available to the buildfarm.

```shell
$ make bloom-release
Cloning into '/tmp/tmpgvqbuI'...
remote: Enumerating objects: 7, done.
remote: Counting objects: 100% (7/7), done.
remote: Compressing objects: 100% (2/2), done.
remote: Total 7 (delta 0), reused 6 (delta 0), pack-reused 0
Receiving objects: 100% (7/7), done.
Updating header /tmp/tmpgvqbuI/melodic/src/lib/version/build_git_version.h
[master d9a1a1c] Generated git metadata patch for melodic release
 1 file changed, 3 insertions(+), 3 deletions(-)
Counting objects: 7, done.
Delta compression using up to 4 threads.
Compressing objects: 100% (2/2), done.
Writing objects: 100% (7/7), 532 bytes | 266.00 KiB/s, done.
Total 7 (delta 1), reused 0 (delta 0)
remote: Resolving deltas: 100% (1/1), completed with 1 local object.
To github.com:PX4-release/Firmware.git
   a9875a7..d9a1a1c  master -> master
==> Fetching 'px4' repository from 'git@github.com:PX4-release/Firmware.git'
Cloning into '/tmp/tmpsNh7YM'...
remote: Enumerating objects: 21, done.
remote: Counting objects: 100% (21/21), done.
remote: Compressing objects: 100% (4/4), done.
remote: Total 21 (delta 2), reused 21 (delta 2), pack-reused 0
Receiving objects: 100% (21/21), done.
Resolving deltas: 100% (2/2), done.
Given track 'melodic' does not exist in release repository.
Available tracks: []
Create a new track called 'melodic' now [Y/n]? 
Creating track 'melodic'...
Repository Name:
  upstream
    Default value, leave this as upstream if you are unsure
  <name>
    Name of the repository (used in the archive name)
  ['upstream']: 
Upstream Repository URI:
  <uri>
    Any valid URI. This variable can be templated, for example an svn url
    can be templated as such: "https://svn.foo.com/foo/tags/foo-:{version}"
    where the :{version} token will be replaced with the version for this release.
  [None]: https://github.com/PX4/Firmware.git
Upstream VCS Type:
  svn
    Upstream URI is a svn repository
  git
    Upstream URI is a git repository
  hg
    Upstream URI is a hg repository
  tar
    Upstream URI is a tarball
  ['git']: 
Version:
  :{ask}
    This means that the user will be prompted for the version each release.
    This also means that the upstream devel will be ignored.
  :{auto}
    This means the version will be guessed from the devel branch.
    This means that the devel branch must be set, the devel branch must exist,
    and there must be a valid package.xml in the upstream devel branch.
  <version>
    This will be the version used.
    It must be updated for each new upstream version.
  [':{auto}']: 
Release Tag:
  :{none}
    For svn and tar only you can set the release tag to :{none}, so that
    it is ignored.  For svn this means no revision number is used.
  :{ask}
    This means the user will be prompted for the release tag on each release.
  :{version}
    This means that the release tag will match the :{version} tag.
    This can be further templated, for example: "foo-:{version}" or "v:{version}"
    
    This can describe any vcs reference. For git that means {tag, branch, hash},
    for hg that means {tag, branch, hash}, for svn that means a revision number.
    For tar this value doubles as the sub directory (if the repository is
    in foo/ of the tar ball, putting foo here will cause the contents of
    foo/ to be imported to upstream instead of foo itself).
  [':{version}']: v:{version}
Upstream Devel Branch:
  <vcs reference>
    Branch in upstream repository on which to search for the version.
    This is used only when version is set to ':{auto}'.
  [None]: master
ROS Distro:
  <ROS distro>
    This can be any valid ROS distro, e.g. indigo, kinetic, lunar, melodic
  ['melodic']: 
Patches Directory:
  :{none}
    Use this if you want to disable overlaying of files.
  <path in bloom branch>
    This can be any valid relative path in the bloom branch. The contents
    of this folder will be overlaid onto the upstream branch after each
    import-upstream.  Additionally, any package.xml files found in the
    overlay will have the :{version} string replaced with the current
    version being released.
  [None]: melodic
Release Repository Push URL:
  :{none}
    This indicates that the default release url should be used.
  <url>
    (optional) Used when pushing to remote release repositories. This is only
    needed when the release uri which is in the rosdistro file is not writable.
    This is useful, for example, when a releaser would like to use a ssh url
    to push rather than a https:// url.
  [None]: git@github.com:PX4-release/Firmware.git
Created 'melodic' track.
==> Setting release repository remote url to 'git@github.com:PX4-release/Firmware.git'
==> git remote set-url origin git@github.com:PX4-release/Firmware.git
==> Testing for push permission on release repository
==> git remote -v
origin	git@github.com:PX4-release/Firmware.git (fetch)
origin	git@github.com:PX4-release/Firmware.git (push)
==> git push --dry-run
To github.com:PX4-release/Firmware.git
   d9a1a1c..9b6af68  master -> master
==> Releasing 'px4' using release track 'melodic'
==> git-bloom-release melodic
Processing release track settings for 'melodic'
Checking upstream devel branch 'master' for package.xml(s)
Cloning into '/tmp/tmpsUYvv6/upstream'...
remote: Enumerating objects: 47, done.
remote: Counting objects: 100% (47/47), done.
remote: Compressing objects: 100% (35/35), done.
remote: Total 270753 (delta 17), reused 27 (delta 11), pack-reused 270706
Receiving objects: 100% (270753/270753), 95.66 MiB | 1.06 MiB/s, done.
Resolving deltas: 100% (204710/204710), done.
Submodule path 'Tools/jMAVSim': checked out 'dc10a13c78afeb97f7b570f8049ec8a912e22d81'
Submodule path 'Tools/jMAVSim/jMAVlib': checked out 'b8d4e8e7acfd2e47f4b51f0c2577fba6ef5ba735'
Submodule path 'Tools/sitl_gazebo': checked out '3062d287c322fabf1b41b8e33518eb449d4ac6ed'
Submodule path 'Tools/sitl_gazebo/external/OpticalFlow': checked out '54471159d4202d305e5643bbad3f2307f5dd1e37'
Submodule path 'Tools/sitl_gazebo/external/OpticalFlow/external/klt_feature_tracker': checked out 'a0d242294ef638d8fa422ab43f0d476ba37a15a6'
Submodule path 'boards/atlflight/cmake_hexagon': checked out '08fd0a73045346448adf6969660196228b23e1fa'
Submodule path 'cmake/configs/uavcan_board_ident': checked out '2e5f9d6768b1dbffc006dc2ceeb2bfe120f22163'
Submodule path 'mavlink/include/mavlink/v2.0': checked out '57000134021fb8d5c2c6281e3783ce29bb35bc17'
Submodule path 'msg/tools/gencpp': checked out '7e446a9976916a7b6fc7266098c67fc6f73a76e0'
Submodule path 'msg/tools/genmsg': checked out '5736b1f7ad037fb5811a3100ba9da2db0ec1f20a'
Submodule path 'platforms/nuttx/NuttX/apps': checked out '7db2a352fb19438bb966e419c61e55092056d385'
Submodule path 'platforms/nuttx/NuttX/nuttx': checked out '423371c7d4012e725ac4ca51323a18df64e581b3'
Submodule path 'src/drivers/gps/devices': checked out 'a4999f111d13bcb209754823a3c503fa659a0d15'
Submodule path 'src/drivers/uavcan/libuavcan': checked out '6174b8c10a2dbf47076ca7a7b5820a4c36c8a988'
Submodule path 'src/drivers/uavcan/libuavcan/dsdl': checked out '192295c4f9b67f4a20b0eabf74757b6597415f2b'
Submodule path 'src/drivers/uavcan/libuavcan/libuavcan/dsdl_compiler/pyuavcan': checked out 'c58477a644d20ccf95a20c151f3a0402f271c3b8'
Submodule path 'src/drivers/uavcan/libuavcan/libuavcan/dsdl_compiler/pyuavcan/dsdl': checked out 'fd12483ddd4e58242d61d74a163e7aeaa1e0f466'
Submodule path 'src/drivers/uavcan/libuavcan/libuavcan_drivers/kinetis': checked out '0c774a5a99bbd91d1f4832290fbed9168b2f65e5'
Submodule path 'src/lib/DriverFramework': checked out '06277ef49fb8c9fad18d56ef40e8bc9fe1655a65'
Submodule path 'src/lib/DriverFramework/dspal': checked out '9b46b4a57f230672ee0806a523963af70bc44f1c'
Submodule path 'src/lib/DriverFramework/dspal/cmake_hexagon': checked out '07168bd5715818802b78f674816ec851307998a7'
Submodule path 'src/lib/ecl': checked out 'a27a43eafa8f4dd514e89984f5394260a36ea4f6'
Submodule path 'src/lib/matrix': checked out '56b069956da141da244926ed7000e89b2ba6c731'
Submodule path 'src/modules/micrortps_bridge/micro-CDR': checked out '62d95c870eafb0cccc7bf70bb8a0cbb86f125a0e'
Looking for packages in 'master' branch... found 'px4'.
Detected version '1.11.0' from package(s): ['px4']

Executing release track 'melodic'
==> bloom-export-upstream /tmp/tmpsUYvv6/upstream git --tag v1.11.0 --display-uri https://github.com/PX4/Firmware.git --name upstream --output-dir /tmp/tmp189G__
Checking out repository at 'https://github.com/PX4/Firmware.git' to reference 'v1.11.0'.
Exporting to archive: '/tmp/tmp189G__/upstream-v1.11.0.tar.gz'
md5: f72ab8beac2f61828357609a3e343a16

==> git-bloom-import-upstream /tmp/tmp189G__/upstream-v1.11.0.tar.gz melodic --release-version 1.11.0 --replace
Creating upstream branch.
Importing archive into upstream branch...
Overlaying files from patched folder 'melodic' on the 'master' branch into the 'upstream' branch...
  Overlaying 'src/lib/version/build_git_version.h' into upstream branch...
Creating tag: 'upstream/1.11.0'
I'm happy.  You should be too.

==> git-bloom-generate -y rosrelease melodic --source upstream -i 1
Releasing package: ['px4']
Releasing package 'px4' for 'melodic' to: 'release/melodic/px4'

==> git-bloom-generate -y rosdebian --prefix release/melodic melodic -i 1 --os-name ubuntu
Generating source debs for the packages: ['px4']
Debian Incremental Version: 1
Debian Distributions: ['bionic']
Releasing for rosdistro: melodic

Pre-verifying Debian dependency keys...
Running 'rosdep update'...
All keys are OK

Placing debian template files into 'debian/melodic/px4' branch.
Not overwriting debian directory.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'bionic' debian for package 'px4' at version '1.11.0-1'
####
Generating debian for bionic...
No historical releaser history, using current maintainer name and email for each versioned changelog entry.

A CHANGELOG.rst was found, but no changelog for this version was found.
You REALLY should have a entry (even a blank one) for each version of your package.

Package 'px4' has dependencies:
Run Dependencies:
  rosdep key           => bionic key
  message_runtime      => ['ros-melodic-message-runtime']
  roscpp               => ['ros-melodic-roscpp']
  rospy                => ['ros-melodic-rospy']
  std_msgs             => ['ros-melodic-std-msgs']
  eigen                => ['libeigen3-dev']
  libmavconn           => ['ros-melodic-libmavconn']
  tf                   => ['ros-melodic-tf']
  mav_msgs             => ['ros-melodic-mav-msgs']
Build and Build Tool Dependencies:
  rosdep key           => bionic key
  message_generation   => ['ros-melodic-message-generation']
  roscpp               => ['ros-melodic-roscpp']
  rospy                => ['ros-melodic-rospy']
  std_msgs             => ['ros-melodic-std-msgs']
  eigen                => ['libeigen3-dev']
  libmavconn           => ['ros-melodic-libmavconn']
  tf                   => ['ros-melodic-tf']
  rostest              => ['ros-melodic-rostest']
  mav_msgs             => ['ros-melodic-mav-msgs']
  catkin               => ['ros-melodic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/control.em' -> 'debian/control'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Creating tag: debian/ros-melodic-px4_1.11.0-1_bionic
####
#### Successfully generated 'bionic' debian for package 'px4' at version '1.11.0-1'
####


==> git-bloom-generate -y rosdebian --prefix release/melodic melodic -i 1 --os-name debian --os-not-required
Generating source debs for the packages: ['px4']
Debian Incremental Version: 1
Debian Distributions: ['stretch']
Releasing for rosdistro: melodic

Pre-verifying Debian dependency keys...
Running 'rosdep update'...
All keys are OK

Placing debian template files into 'debian/melodic/px4' branch.
Not overwriting debian directory.
==> Placing templates files in the 'debian' folder.

####
#### Generating 'stretch' debian for package 'px4' at version '1.11.0-1'
####
Generating debian for stretch...
No historical releaser history, using current maintainer name and email for each versioned changelog entry.

A CHANGELOG.rst was found, but no changelog for this version was found.
You REALLY should have a entry (even a blank one) for each version of your package.

Package 'px4' has dependencies:
Run Dependencies:
  rosdep key           => stretch key
  message_runtime      => ['ros-melodic-message-runtime']
  roscpp               => ['ros-melodic-roscpp']
  rospy                => ['ros-melodic-rospy']
  std_msgs             => ['ros-melodic-std-msgs']
  eigen                => ['libeigen3-dev']
  libmavconn           => ['ros-melodic-libmavconn']
  tf                   => ['ros-melodic-tf']
  mav_msgs             => ['ros-melodic-mav-msgs']
Build and Build Tool Dependencies:
  rosdep key           => stretch key
  message_generation   => ['ros-melodic-message-generation']
  roscpp               => ['ros-melodic-roscpp']
  rospy                => ['ros-melodic-rospy']
  std_msgs             => ['ros-melodic-std-msgs']
  eigen                => ['libeigen3-dev']
  libmavconn           => ['ros-melodic-libmavconn']
  tf                   => ['ros-melodic-tf']
  rostest              => ['ros-melodic-rostest']
  mav_msgs             => ['ros-melodic-mav-msgs']
  catkin               => ['ros-melodic-catkin']
==> In place processing templates in 'debian' folder.
Expanding 'debian/copyright.em' -> 'debian/copyright'
Expanding 'debian/control.em' -> 'debian/control'
Expanding 'debian/gbp.conf.em' -> 'debian/gbp.conf'
Expanding 'debian/rules.em' -> 'debian/rules'
Expanding 'debian/source/options.em' -> 'debian/source/options'
Expanding 'debian/source/format.em' -> 'debian/source/format'
Expanding 'debian/changelog.em' -> 'debian/changelog'
Expanding 'debian/compat.em' -> 'debian/compat'
Creating tag: debian/ros-melodic-px4_1.11.0-1_stretch
####
#### Successfully generated 'stretch' debian for package 'px4' at version '1.11.0-1'
####


==> git-bloom-generate -y rosrpm --prefix release/melodic melodic -i 1
No platforms defined for os 'fedora' in release file for the 'melodic' distro.
Not performing RPM generation.




Tip: Check to ensure that the debian tags created have the same version as the upstream version you are releasing.
Everything went as expected, you should check that the new tags match your expectations, and then push to the release repo with:
  git push --all && git push --tags  # You might have to add --force to the second command if you are over-writing existing tags
<== Released 'px4' using release track 'melodic' successfully
==> git remote -v
origin	git@github.com:PX4-release/Firmware.git (fetch)
origin	git@github.com:PX4-release/Firmware.git (push)
Releasing complete, push to release repository?
Continue [Y/n]? 
==> Pushing changes to release repository for 'px4'
==> git push --all
Counting objects: 20713, done.
Delta compression using up to 4 threads.
Compressing objects: 100% (9653/9653), done.
Writing objects: 100% (20713/20713), 133.99 MiB | 592.00 KiB/s, done.
Total 20713 (delta 10188), reused 20602 (delta 10162)
remote: Resolving deltas: 100% (10188/10188), done.
To github.com:PX4-release/Firmware.git
   d9a1a1cb..5ddacac4  master -> master
 * [new branch]        debian/melodic/bionic/px4 -> debian/melodic/bionic/px4
 * [new branch]        debian/melodic/px4 -> debian/melodic/px4
 * [new branch]        debian/melodic/stretch/px4 -> debian/melodic/stretch/px4
 * [new branch]        patches/debian/melodic/bionic/px4 -> patches/debian/melodic/bionic/px4
 * [new branch]        patches/debian/melodic/px4 -> patches/debian/melodic/px4
 * [new branch]        patches/debian/melodic/stretch/px4 -> patches/debian/melodic/stretch/px4
 * [new branch]        patches/release/melodic/px4 -> patches/release/melodic/px4
 * [new branch]        release/melodic/px4 -> release/melodic/px4
 * [new branch]        upstream -> upstream
<== Pushed changes successfully
==> Pushing tags to release repository for 'px4'
==> git push --tags
Total 0 (delta 0), reused 0 (delta 0)
To github.com:PX4-release/Firmware.git
 * [new tag]           debian/ros-melodic-px4_1.11.0-1_bionic -> debian/ros-melodic-px4_1.11.0-1_bionic
 * [new tag]           debian/ros-melodic-px4_1.11.0-1_stretch -> debian/ros-melodic-px4_1.11.0-1_stretch
 * [new tag]           release/melodic/px4/1.11.0-1 -> release/melodic/px4/1.11.0-1
 * [new tag]           upstream/1.11.0 -> upstream/1.11.0
<== Pushed tags successfully
...
```


## Reset release
A release that has been tagged but bloom-release has not yet been run can be rolled back
```shell
git log | head
git tag -d v1.11.0 \
&& git reset HEAD~1 \
&& git checkout package.xml
```

## Other Notes
Snippet for `~/.gitconfig`
```
[url "git@github.com:PX4-release"]
	insteadOf = https://github.com/PX4-release
```
