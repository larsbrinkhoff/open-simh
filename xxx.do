!rm -f XXX*.LOG
!../lap6-tools/lap6 mk xxx.linc
!../lap6-tools/lap6 rb fresh-lap6.linc 300 40 | ../lap6-tools/lap6 wb xxx.linc 300
!../lap6-tools/lap6 mx xxx.linc
!../lap6-tools/lap6 sm xxx.linc XXX < XXX

set debug stdout
set dpy text
attach tape0 xxx.linc
expect "1"
boot tape0

send "\e"
expect "-/"
continue

send "AM XXX,0\r"
expect "LOADED"
set tape debug=dbg;seek;read;write
continue

send "\020"
expect " 50 "
examine @xxx1.log 0-3777
continue

send "\016"
expect " 51 "
examine @xxx2.log 0-3777
continue

examine @xxx3.log 0-3777
quit
