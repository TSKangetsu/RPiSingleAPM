#!/bin/bash
sleep 1

while ! [ -f $3/.vscode/connect-ok ]; do
    sleep 0.2
done

sleep 2

$2 "$@"
