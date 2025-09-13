#!/bin/bash

execute_names=("passable_node")

for execute_name in ${execute_names[*]}
do
ps aux | grep -e ${execute_name} | grep -v grep | awk '{print $2}' | xargs -i kill -2 {}
done