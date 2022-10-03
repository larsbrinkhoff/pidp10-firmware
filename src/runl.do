set throttle 1m

; ; connect to real panel
;set realcons host=localhost
;set realcons host=blinkenbone
;set realcons connected
; We're running on a desktop PC with connection to localhost: max speed!
;set realcons interval=1
;set realcons debug
;show realcons



;				        title	Lights
;					; This show a moving light pattern on the
;					; "memory indicator" LEDs on a KI10 console panel.
;
;	001000				loc	1000
;	001000			start:
;	001000	201000	000777		movei	0,777	; acc0 is shifted. pattern = 12 bits
;	001001			nxtbit:
;	001001	241000	000001		rot    	0,1		; acc0 <<= 1
;	001002	202000	001007		movem	0,buff		; buff := acc0
;	001003	700540	001007		datao	pi,buff		; show acc0 on memory indicator
;
;					; wait loop
;	001004	201100	060650		movei	2,^D25000  	; acc2 = 25000 = good @1MHz
;	001005	366100	001005	wait:   sojn	2,wait		;  while(acc2--) ;
;
;	001006	324000	001001		jumpa	nxtbit
;
;
;	001007			buff:
;	001007	000000	000000		 0	; temporary word buffer


d  777 200000700000
d 1000 201000000777
d 1001 241000000001
d 1002 202000001007
d 1003 700540001007
d 1004 201100060650
d 1005 366100001005
d 1006 324000001001
d 1007 000000000000
d 1005 jfcl

set cpu 16k
at ptr /home/pi/imgs/dskdmp.rim
;go 1000
