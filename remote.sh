rm .vscode/connect-ok

$5 ssh $1@$2 "killall gdbserver"
echo "[$3-DEBUG] start scp EXE to Target ..."
$5 scp build/$3 $1@$2:$7
$5 $6
echo "[$3-DEBUG] lanuch $3 Directly ..."
touch .vscode/connect-ok
if [ "$9" == "1" ]; then
    $5 ssh $1@$2 "$8 /usr/bin/gdbserver :9590 $7$3 $4"
elif [  "$9" == "2" ]; then
    $5 ssh $1@$2 "$8 $7$3 $4"
fi
