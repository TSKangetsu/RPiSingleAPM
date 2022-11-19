ssh $1@$2 "killall gdbserver"
echo "[SingleFlight-DEBUG] start scp EXE to Target ..."
scp build/SingleFlight $1@$2:/usr/bin
scp APMconfig.json $1@$2:/etc
echo "[SingleFlight-DEBUG] lanuch SingleFlight Directly ..."
ssh $1@$2 "/usr/bin/gdbserver :9590 /usr/bin/SingleFlight $3"