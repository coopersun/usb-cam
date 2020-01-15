#!/bin/bash

set -e

root=$(pwd)

# Jenkins sets both of these but for non-jenkins builds try to do
# the right thing...
if [ -z "$WORKSPACE" ]; then
    WORKSPACE=$root
fi

build_num="$BUILD_NUMBER"
if [ -z "$build_num" ]; then
    build_num="0"
fi

# What the heck ros...
cat > ${WORKSPACE}/ffmpeg.yaml <<EOF
ffmpeg:
  ubuntu: [ffmpeg]
EOF

cat > ${WORKSPACE}/ffmpeg.list <<EOF
yaml file:////${WORKSPACE}/ffmpeg.yaml
EOF

# Put this first so our packages get found first.
sudo cp ffmpeg.list /etc/ros/rosdep/sources.list.d/00-ffmpeg.list

rosdep update

cd code

rm -rf obj-x86_64-linux-gnu debian build

# We started our version at 0.4.x
# Seeing as the driver is orphaned, this should be ok...
old_ver="0.3.6"
new_ver="0.4.$build_num"
sed -i s/$old_ver/$new_ver/ package.xml
bloom-generate rosdebian --os-name ubuntu \
                         --os-version xenial \
                         --ros-distro kinetic .

fakeroot make -d -f debian/rules binary
