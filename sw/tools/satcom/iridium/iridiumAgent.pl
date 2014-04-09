#!/usr/bin/perl -w


# alexandre bustico
# alexandre.bustico@enac.fr
#
# this ivy agent fetch mail from long range uav which transmit data
# over an iridium modem.
# aircraft id is not yet in the binary message, so it must be given on command line
# with --acid
# If several iridium modem could send message, a filtering is possible specifing
# a --imei option. ALL 15 digits of imei should be given as is is used for uplink communication

# TODO :
# * utiliser l'adresse mail d'emission reelle
# * testIridium sur maquette stm32 :
#   ° reception => depaqueter les 3 types de messages et afficher les messages dans ttyConsole
#   ° envoi => envoyer le message descendant de 23 octets
# * faire la même chose dans pprz dans la section TEST


use strict;
use warnings;
use feature ':5.12';

use Mail::IMAPClient;
use IO::Socket::SSL;
use MIME::Parser;
use Net::SMTP;
use MIME::Lite;
use File::Path; 
use Getopt::Long;
use Ivy;


use constant M_PI => 3.141592654 ;
use constant ALIVE_TIMEOUT_PERIOD => 5000;
use constant FETCHMAIL_PERIOD => 15000; # 15 seconds
use constant DATAGRAM_LENGTH => 23;

my @md5 = (0207 ,0151 ,0313 ,0256 ,0355 ,0252 ,0016 ,0273 ,0072 ,0126 ,0273 ,0222 ,0017 ,0372 ,0320 ,0200);


sub periodicMailfetch ();
sub imapConnectGmail ($$);
sub imapDisconnectGmail ();
sub getNextUnseenMessage(); # return undef when no more unseen message
sub getLastReceivedMessage(); 
sub getAttachment ($);
sub decodeBinaryAttachment ($);
sub smtpSend ($);
sub RadOfDeg($);
sub DegOfRad($) ;
sub statusFunc ($$);
sub defaultOption ($$);
sub usage (;$);
sub receiveMoveWPCb(@);
sub receiveSettingCb(@);
sub receiveBlockCb(@);




my $gmailImap;
my $gmailSmtp;
my $mimeParser = MIME::Parser->new;
my %options;

# TODO
# renseigner neededApp

#                             _
#                            (_)
#         _ __ ___     __ _   _    _ __
#        | '_ ` _ \   / _` | | |  | '_ \
#        | | | | | | | (_| | | |  | | | |
#        |_| |_| |_|  \__,_| |_|  |_| |_|

END {
    Ivy::stop ();
    imapDisconnectGmail ();
} 


#OPTIONS
GetOptions (\%options, "bus=s", "imei=s", "acid=i", "gmuser=s", "gmpasswd=s", "lan");
defaultOption ("bus", $ENV{IVYBUS});
usage ("aircraft id is mandatory") unless exists $options{'acid'};
usage ("gmail user is mandatory") unless exists $options{'gmuser'};
usage ("gmail passwd is mandatory") unless exists $options{'gmpasswd'};
usage ("imei (15 digits) is mandatory\n") unless 
    exists $options{'imei'};
usage ("imei should be 15 digits long\n") unless  $options{'imei'} =~ /^\d{15}$/;
imapConnectGmail ($options{'gmuser'}, $options{'gmpasswd'});

if (exists($options{'lan'})) {
    warn "use local, *NO* authenticated, smtp server\n";
} else {
    warn "use gmail, authenticated, smtp server\n";
}

# TEST SEND
#smtpSend ("un buffer de test");
#exit (0);
# END TEST SEND

$mimeParser->output_to_core(1);
$mimeParser->tmp_dir("/tmp");

Ivy->init (-loopMode => 'LOCAL',
           -appName =>  'IvySatCom',
           -ivyBus => $options{bus},
           -filterRegexp => [$options{'acid'}]
          ) ;

my $bus = Ivy->new (-statusFunc => \&statusFunc,
#                    -neededApp => 
    );

$bus->bindRegexp ('^(\S+)\s+MOVE_WP\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)\s+(\S+)', [\&receiveMoveWPCb]);
$bus->bindRegexp ('^(\S+)\s+SETTING\s+(\S+)\s+(\S+)\s+(\S+)', [\&receiveSettingCb]);
$bus->bindRegexp ('^(\S+)\s+BLOCK\s+(\S+)\s+(\S+)', [\&receiveBlockCb]);

$bus->after (1, [\&periodicMailfetch]);
$bus->repeat (FETCHMAIL_PERIOD, [\&periodicMailfetch]);
$bus->repeat (ALIVE_TIMEOUT_PERIOD, [\&alive]);

$bus->start ();
Ivy::mainLoop ();

#                        _
#                       | |
#         ___    _   _  | |__
#        / __|  | | | | | '_ \
#        \__ \  | |_| | | |_) |
#        |___/   \__,_| |_.__/
#
sub periodicMailfetch ()
{
    while (my @unseenMails = getLastReceivedMessage()) {
	foreach my $unseen  (@unseenMails) {
	    my $msgBody = $gmailImap->message_string($unseen);
	    if (defined $msgBody) {
		my $entity =  $mimeParser->parse_data($msgBody);
		getAttachment ($entity);
	    } else {
		say "WTF \$msgBody *NOT* defined";
	    }
	
	    # debug, on le garde non lu
            #$gmailImap->unset_flag("Seen", $unseen);
	}
    }
 }



sub alive ()
{
    $bus->sendMsgs (sprintf  "%d ALIVE %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,",
		    $options{'acid'},
		    @md5);
}


sub getNextUnseenMessage() # return undef when no more unseen message
{
    return $gmailImap->unseen();
}

sub getLastReceivedMessage() # return undef when no more unseen message
{
    state %allSeen;
    my $mn;

    unless ($gmailImap->IsConnected) {
	say "Warn: reconnect after beeing deconnected";
	imapConnectGmail ($options{'gmuser'}, $options{'gmpasswd'});
    }
    
    $gmailImap->select("INBOX");
    my @mailno = $gmailImap->search("ALL");
    push  @mailno, $gmailImap->unseen();

    @mailno = grep (!exists $allSeen{$_}, $gmailImap->search("ALL"));
    foreach $mn (@mailno) {
	$allSeen{$mn} =1;
    }

    return (@mailno);
}



sub imapConnectGmail ($$) 
{
    my ($u,$p) =@_;
# Connect to the IMAP server via SSL
    my $socket = IO::Socket::SSL->new(
	PeerAddr => 'imap.gmail.com',
	PeerPort => 993,
	)
	or die "socket(): $@";
    
# Build up a gmail attached to the SSL socket.
# Login is automatic as usual when we provide User and Password
    $gmailImap = Mail::IMAPClient->new(
	Socket   => $socket,
	User     => $u,
	Password => $p,
	)
	or die "new(): $@";
    $gmailImap->select("INBOX");
}




sub imapDisconnectGmail ()
{
# Say bye
    $gmailImap->logout() if defined $gmailImap;
}



sub getAttachment ($) {
    my $ent = shift;
    my @parts = $ent->parts;
    my $fileName;
    my $io;

    if (@parts) {        # multipart...
	map { getAttachment($_) } @parts;
    } elsif (($fileName= $ent->head->recommended_filename) && ($fileName =~ /.sbd$/)) {
	my ($modemImei) = $fileName =~ /^(\d+)/;
	say "Filename : $fileName ;;  modem imei : $modemImei";
	if (exists $options{'imei'}) {
	    unless ($modemImei =~ /$options{'imei'}/o) {
		warn "message imei $modemImei does not match imei filtering pattern $options{'imei'}\n";
		return;
	    }
	}
	if ($io = $ent->open("r")) {
	    #my $pbc = <$io>;
	    my $pbc;
	    read ($io, $pbc, 100);
	    if (length $pbc == DATAGRAM_LENGTH) {
		decodeBinaryAttachment ($pbc) ;
	    } else {
		warn sprintf ("Taille de pièce jointe (%d) incorrecte, ".
			      "attendu : %d\n", length $pbc, DATAGRAM_LENGTH);
	    }
	    $io->close();
	} else {
	    warn "*** io open error ***\n";
	}
    } 
}


# C  An unsigned char (octet) value.
# s  A signed short (16-bit) value.
# S  An unsigned short value.
# i  A signed integer value.
sub decodeBinaryAttachment ($)
{
    my $binaryData = shift;

    my ($raw_gps_lat, $raw_gps_lon, $raw_gps_alt, $raw_gps_gspeed, $raw_gps_course, 
	$raw_estimator_airspeed, $raw_electrical_vsupply, $raw_energy, $raw_throttle, 
	$raw_pprz_mode, $raw_nav_block, $raw_estimator_flight_time) = 
	    unpack ("iiSSs" . "sCCC" . "CCS", $binaryData);

    

    say "raw_gps_lat=$raw_gps_lat, raw_gps_lon=$raw_gps_lon, raw_gps_alt=$raw_gps_alt\n" .
	"raw_gps_gspeed=$raw_gps_gspeed, raw_gps_course=$raw_gps_course\n" .
	"raw_estimator_airspeed=$raw_estimator_airspeed, raw_electrical_vsupply=$raw_electrical_vsupply\n" .
	"raw_energy=$raw_energy, raw_throttle=$raw_throttle\n" .
	"raw_pprz_mode=$raw_pprz_mode, raw_nav_block=$raw_nav_block, " .
	"raw_estimator_flight_time=$raw_estimator_flight_time\n" ;
    

    $bus->sendMsgs (sprintf "%d GENERIC_COM %d %d %d %d %d %d %d %d %d %d %d %d",
		    $options{'acid'},
		    $raw_gps_lat,
		    $raw_gps_lon,
		    $raw_gps_alt,
		    $raw_gps_gspeed,
		    $raw_gps_course,
		    $raw_estimator_airspeed,
		    $raw_electrical_vsupply,
		    $raw_energy,
		    $raw_throttle,
		    $raw_pprz_mode,
		    $raw_nav_block,
		    $raw_estimator_flight_time);
}



sub statusFunc ($$)
{
  my ($ready, $notReady) = @_;

  if (@{$notReady})  {
    printf "appli manquantes : %s\n", join (' ', @{$notReady});
  } else {
    printf ("Toutes applis OK !!\n");
  }
}

sub defaultOption ($$)
{
  my ($option, $default) = @_;
  unless  (defined $options{$option}) {
    warn "option $option not given : using of default value $default\n";
    $options{$option} = $default;
  }
}

 # <message name="MOVE_WP" id="2" link="forwarded">
 #  <field name="wp_id" type="uint8"/>
 #  <field name="ac_id" type="uint8"/>
 #  <field name="lat" type="int32" unit="e-7deg"/>
 #  <field name="lon" type="int32" unit="e-7deg"/>
 #  <field name="alt" type="int32" unit="cm"/>
 # </message>
sub receiveMoveWPCb(@)
{
    my ($app, $from, $wp_id, $ac_id, $lat, $lon, $alt) = @_;
    my $msgId = 2;

    say "DBG> receiveMoveWPCb => wp_id=$wp_id, ac_id=$ac_id, lat=$lat, long=$lon, alt=$alt";
    return unless $ac_id == $options{'acid'};
    smtpSend (pack ("CCClll", $msgId, $wp_id, $ac_id, $lat, $lon, $alt));
}

 # <message name="SETTING" id="4" link="forwarded">
 #  <field name="index" type="uint8"/>
 #  <field name="ac_id" type="uint8"/>
 #  <field name="value" type="float"/>
 # </message>

sub receiveSettingCb(@)
{
    my ($app, $from, $index, $ac_id, $value) = @_;
    my $msgId = 4;
  
    say "receiveSettingCb index=$index, ac_id=$ac_id, value=$value";
    return unless $ac_id == $options{'acid'};
     smtpSend (pack ("CCCf", $msgId, $index, $ac_id, $value));
}


 # <message name="BLOCK" id="5" link="forwarded">
 #  <field name="block_id" type="uint8"/>
 #  <field name="ac_id" type="uint8"/>
 # </message>
sub receiveBlockCb(@)
{
    my ($app, $from, $block_id, $ac_id) = @_;
    my $msgId = 5;

    say "receiveBlockCb  block_id=$block_id, ac_id=$ac_id";
    return unless $ac_id == $options{'acid'};
    smtpSend (pack ("CCC", $msgId, $block_id, $ac_id));
}



sub smtpSend ($)
{
    my $binaryBuffer = shift;
    
    my $msg = MIME::Lite->new(
        From     => $options{'gmuser'},
#        To       => 'alexandre.bustico@free.fr',
	To       => 'Data@SBD.Iridium.com',
        Subject  => $options{'imei'},
        Type     => 'application/octet-stream',
        Encoding => 'base64',
        Filename => 'msg.sbd',
        Data     => $binaryBuffer
    );

    if (exists($options{'lan'})) {
	$msg->send (smtp => 'smtp');
    } else {
	$msg->send (smtp => 'smtp.gmail.com:587',
		    AuthUser =>  $options{'gmuser'},
		    AuthPass => $options{'gmpasswd'});
    }
}

sub usage (;$)
{
    my $msg = shift;
    say "Error: $msg" if defined $msg;
    say "USAGE: $0 ".
	"\n[(optional) --bus 'ivybus'] \n".
	"(mandatory) --acid 'aircraft_id'\n".
	"(mandatory) --gmuser 'gmail user'\n".
	"(mandatory) --gmpasswd 'gmail password'\n".
	"[(optional) --imei 'filtering pattern on imei']";
    exit;
}

sub RadOfDeg($) 
{
    my $x = shift;
    return ($x * (M_PI/180.));
}

sub DegOfRad($) 
{ 
    my $x = shift;
    return ($x * (180. / M_PI));
}
