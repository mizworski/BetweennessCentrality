#!/usr/bin/env bash

rm times_o2

for i in {1..12}
do
    echo "Testing O2 for ${i} threads."
    echo "Performance for ${i} threads." >>times_o2
    time (./../build/brandes ${i} ../input/wiki-vote-sort.txt out/out_o3_${i} 1>/dev/null) &>>times_o2
done
