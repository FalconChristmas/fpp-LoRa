<?
$LoRaSpeeds = Array();
$LoRaSpeeds['1200'] = '1200';
$LoRaSpeeds['2400'] = '2400';
$LoRaSpeeds['4800'] = '2400';
$LoRaSpeeds['9600'] = '9600';
$LoRaSpeeds['19200'] = '19200';
$LoRaSpeeds['38400'] = '38400';
$LoRaSpeeds['57600'] = '57600';
$LoRaSpeeds['115200'] = '115200';

$LoRaAirSpeeds = Array();
$LoRaAirSpeeds['300'] = '300';
$LoRaAirSpeeds['1200'] = '1200';
$LoRaAirSpeeds['2400'] = '2400';
$LoRaAirSpeeds['4800'] = '2400';
$LoRaAirSpeeds['9600'] = '9600';
$LoRaAirSpeeds['19200'] = '19200';

$LoRaPorts = Array();
foreach(scandir("/dev/") as $fileName) {
    if ((preg_match("/^ttyS[0-9]+/", $fileName)) ||
        (preg_match("/^ttyACM[0-9]+/", $fileName)) ||
        (preg_match("/^ttyO[0-9]/", $fileName)) ||
        (preg_match("/^ttyS[0-9]/", $fileName)) ||
        (preg_match("/^ttyAMA[0-9]+/", $fileName)) ||
        (preg_match("/^ttyUSB[0-9]+/", $fileName))) {
        $LoRaPorts[$fileName] = $fileName;
    }
}
?>


<div id="global" class="settings">
<fieldset>
<legend>LoRa MultiSync Configuration</legend>

Enable LoRa Plugin:  <? PrintSettingCheckbox("Enable LoRa", "LoRaEnable", 1, 0, "1", "0", "fpp-LoRa"); ?>
<p>

The FPP LoRa plugin supports using the UART based LoRa modules to send/receive MultiSync packets via LoRa wireless.   It currently
supports the EBYTE "E32-*" modules including the E32-915T30D, E32-433T20D, etc...  They can be wired directly to the UART pins on
the Pi/BBB or by pluging into the USB port using a E15-USB-T2 adapter (recommended).   Make sure the M1 and M2 pins are pulled
low (M1 and M2) jumpers are in place on the E15-USB-T2).
<p>
If FPP is in Master mode, it will send MultiSync packets out.   If in Remote mode, it will listen for packets.
<p>
These setting much match what the module has been configured to use.<br>
Port:  <? PrintSettingSelect("Port", "LoRaDevicePort", 1, 0, 'ttyUSB0', $LoRaPorts, "fpp-LoRa") ?>
&nbsp; Baud Rate: <? PrintSettingSelect("Speed", "LoRaDeviceSpeed", 1, 0, '9600', $LoRaSpeeds, "fpp-LoRa") ?>
<p>
Send media sync packets: <? PrintSettingCheckbox("Enable LoRa Media", "LoRaMediaEnable", 1, 0, "1", "0", "fpp-LoRa", "", "1"); ?>
<br>
Bridge to local network: <? PrintSettingCheckbox("Enable LoRa Bridge", "LoRaBridgeEnable", 1, 0, "1", "0", "fpp-LoRa"); ?>
<p>
The LoRa protocol is very slow, it defaults to 2400 baud over the air.   At the start of each sequence, we have to send the
filenames for the sequence and the media which can take time.  If you know the remotes will not need the media filenames, turn
off sending the media sync packets which will help the remotes start quicker.
<p>
Bridging to the local network will re-send the sync packets out on the local network so other FPP devices on can
be synced.   This remote must have it's MultiSync targets set while in Master mode once before being placed back
into Remote mode.

</fieldset>
<br>

<fieldset>
<legend>LoRa Device Configuration</legend>
In order to configure the LoRa device, FPPD must be running with the LoRa plugin loaded.
The module must be put into Sleep mode by pulling M1 and M2 high.  If using the E15-USB-T2 adapter, just remove
the M1 and M2 jumpers.   On submit, the settings below will be saved onto the device.
<p>

<form  id="LoRaconfigForm" action="api/plugin-apis/LoRa" method="post">
Module Address (0-65535): <input name="MA" type="number" required min="0" max="65535" value="0">
<br>
UART Baud Rate:
<select name="UBR">
<option value="1200">1200</option>
<option value="2400">2400</option>
<option value="4800">4800</option>
<option value="9600" selected>9600</option>
<option value="19200">19200</option>
<option value="38400">38400</option>
<option value="57600">57600</option>
<option value="115200">115200</option>
</select>
<br>
Air Data Rate: <select name="ADR">
<option value="300">300</option>
<option value="1200">1200</option>
<option value="2400" selected>2400</option>
<option value="4800">4800</option>
<option value="9600">9600</option>
<option value="19200">19200</option>
</select>
<br>
FEC (Forward Error Correction): <input name="FEC" type="checkbox" value="1" checked/>
<br>
Transmit Power: <select name="TXP">
<option value="4" selected>High</option>
<option value="3">Medium</option>
<option value="2">Low</option>
<option value="1">Very Low</option>
</select>
<br>
Channel/Frequency (0-255): <input name="CH" type="number" required min="0" max="255" value="23">
<br>
The Channel/Frequency setting depends on the Module.  For the 433Mhz modules, the frequency will be 410 plus
the setting.   Thus, a setting of 23 will result in the default frequency of 433Mhz.    For the 868Mhz modules,
it is 862 + the setting so the default is 6.   For the 915Mhz, it is 900 plus the setting, so the default is 15.
<br>
<input type="submit" value="Save">

</form>


<script type='text/javascript'>
$("#LoRaconfigForm").submit(function(event) {
     event.preventDefault();
     var $form = $( this ),
     url = $form.attr( 'action' );
     
     /* Send the data using post with element id name and name2*/
     var posting = $.post( url, $form.serialize());
     
     /* Alerts the results */
     posting.done(function( data ) {
                  //alert('success');
                  });
     });
</script>
</fieldset>
