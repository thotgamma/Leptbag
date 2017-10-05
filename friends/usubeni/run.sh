#!/bin/bash
TARGET="usubeni"

cp $TARGET".friends" "../../source/friends/"
cd ../../source
./japari_park
cd "../friends/"$TARGET
rm "../../source/friends/"$TARGET".friends"
