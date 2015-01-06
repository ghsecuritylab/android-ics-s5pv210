#!/bin/bash
rm out/target/product/smdkv210/root/ -rf
. build/envsetup.sh
lunch 12
make -j4

