
° subsystem organisation


-------     ---------	   -----------------     -----------     ----------      -----------------       ---------------
|     |     |modem  |	   |constellation  |     |servers  |imap |servers | imap |               | 	 |             |
| AP  |<->  |iridium|  <-> |iridium        | <-> |iridium  | <-> |mail    |  <-> |iridiumAgent.pl|  <->  |groundStation|
|_____|uart |_______|wawe  |_______________|wave |_________|smtp |________| smtp |_______________| ivy	 |_____________|


° iridiumAgent.pl usage

iridiumAgent.pl is a two way link between groundstation (ivy) and mail server (imap+smtp).
for now, google mail server (gmail) is hardcoded in the script, but it is possible to displace
that as parameter.

Mandatory arguments are :

 * aircraft id :	--acid 1 
 * gmail user :		--gmuser mu.gmail.account@gmail.com 
 * gmail password :	--gmpasswd paparazzi_118 
 * modem imei           --imei 123456789012345 

optional argument is :

 * --lan if connected on lan behind a firewall (should connect without auth on local smtp server)
         if no option is given, it will behave as in the field situation where it connects to gmail smtm server
         with auth.


iridiumAgent.pl --acid 1 --gmuser satcom.drones.enac@gmail.com --gmpasswd paparazzi_118 --imei 300434060002380
