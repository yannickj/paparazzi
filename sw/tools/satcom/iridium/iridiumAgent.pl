#!/usr/bin/perl -w


# alexandre bustico
# alexandre.bustico@enac.fr
#
# this ivy agent fetch mail from long range uav which transmit data
# over an iridium modem.
# aircraft id is not yet in the binary message, so it must be given on command line
# with --acid
# If several iridium modem could send message, a filtering is possible specifing
# a --imei option. Not all 15 digits of imei should be given, but only 
# any discriminent part of it.



use strict;
use warnings;
use feature ':5.12';

use Mail::IMAPClient;
use IO::Socket::SSL;
use MIME::Parser;
use File::Path; 
use Getopt::Long;
use Ivy;


use constant M_PI => 3.141592654 ;
use constant ALIVE_TIMEOUT_PERIOD => 5000;
use constant FETCHMAIL_PERIOD => 15000; # 15 seconds
use constant DATAGRAM_LENGTH => 23;

my @md5 = (0207 ,0151 ,0313 ,0256 ,0355 ,0252 ,0016 ,0273 ,0072 ,0126 ,0273 ,0222 ,0017 ,0372 ,0320 ,0200);


sub periodicMailfetch ();
sub imapConnectGmail ();
sub imapDisconnectGmail ();
sub getNextUnseenMessage(); # return undef when no more unseen message
sub getLastReceivedMessage(); 
sub getAttachment ($);
sub decodeBinaryAttachment ($);
sub RadOfDeg($);
sub DegOfRad($) ;
sub statusFunc ($$);
sub defaultOption ($$);
sub usage (;$);



my $gmail;
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
GetOptions (\%options, "bus=s", "imei=i", "acid=i");
defaultOption ("bus", $ENV{IVYBUS});
usage ("aircraft id is mandatory") unless exists $options{'acid'};
warn "imei not given, no filtering on imei will be done\n" unless 
    exists $options{'imei'};
imapConnectGmail ();


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
sub periodicMailfetch ()
{
    while (my @unseenMails = getLastReceivedMessage()) {
	foreach my $unseen  (@unseenMails) {
	    my $msgBody = $gmail->message_string($unseen);
	    if (defined $msgBody) {
		my $entity =  $mimeParser->parse_data($msgBody);
		getAttachment ($entity);
	    } else {
		say "WTF \$msgBody *NOT* defined";
	    }
	
	    # debug, on le garde non lu
            #$gmail->unset_flag("Seen", $unseen);
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
    return $gmail->unseen();
}

sub getLastReceivedMessage() # return undef when no more unseen message
{
    state %allSeen;
    my $mn;

    unless ($gmail->IsConnected) {
	say "Warn: reconnect after beeing deconnected";
	imapConnectGmail ();
    }
    
    $gmail->select("INBOX");
    my @mailno = $gmail->search("ALL");
    push  @mailno, $gmail->unseen();

    @mailno = grep (!exists $allSeen{$_}, $gmail->search("ALL"));
    foreach $mn (@mailno) {
	$allSeen{$mn} =1;
    }

    return (@mailno);
}



sub imapConnectGmail () 
{
# Connect to the IMAP server via SSL
    my $socket = IO::Socket::SSL->new(
	PeerAddr => 'imap.gmail.com',
	PeerPort => 993,
	)
	or die "socket(): $@";
    
# Build up a gmail attached to the SSL socket.
# Login is automatic as usual when we provide User and Password
    $gmail = Mail::IMAPClient->new(
	Socket   => $socket,
	User     => 'test.corsica',
	Password => 'spokecorsica',
	)
	or die "new(): $@";
    $gmail->select("INBOX");
}




sub imapDisconnectGmail ()
{
# Say bye
    $gmail->logout() if defined $gmail;
}



sub getAttachment ($) {
    my $ent = shift;
    my @parts = $ent->parts;
    my $fileName;
    my $io;

    if (@parts) {        # multipart...
	map { getAttachment($_) } @parts;
    }
    elsif (($fileName= $ent->head->recommended_filename) && ($fileName =~ /.sbd$/)) {
	my ($modemImei) = $fileName =~ /^(\d+)/;
	say "Filename : $fileName ;;  modem imei : $modemImei";
	if (exists $options{'imei'}) {
	    unless ($modemImei =~ /$options{'imei'}/o) {
		warn "message imei $modemImei does not match imei filtering pattern $options{'imei'}\n";
		return;
	    }
	}
	if ($io = $ent->open("r")) {
	    my $pbc = <$io>;
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



sub usage (;$)
{
    my $msg = shift;
    say "Error: $msg" if defined $msg;
    say "USAGE: $0 \n[(optional) --bus 'ivybus'] \n(mandatory) --acid 'aircraft_id'\n".
	"[(optional) --imei 'filtering pattern on imei']";
    exit;
}
