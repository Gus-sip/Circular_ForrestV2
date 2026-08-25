#pragma once

#include <Arduino.h>

// Ultra-simple plain-text dashboard: no CSS, no layout, just a <pre> block of
// label/value lines. Polls /data every 3s via fetch() - same JSON endpoint as
// before, just rendered as text instead of a tile grid.
const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CHIP FOREST</title>
</head>
<body>
<pre id="out">connecting...</pre>
<script>
function fmt(v) { return (typeof v === 'number') ? v.toFixed(1) : v; }

function render(d) {
  var out = document.getElementById('out');

  if (d.status === 'no_data') {
    out.textContent = 'NO DATA YET - waiting for first packet...';
    return;
  }
  if (d.status === 'stale') {
    out.textContent = 'SIGNAL LOST - last packet ' + d.secondsAgo + 's ago';
    return;
  }

  var n = d.nbiot || {};
  out.textContent =
    'Last packet: ' + d.secondsAgo + 's ago (RSSI ' + d.rssi + ' dBm, SNR ' + d.snr + ')\n\n' +
    'Temp:        ' + fmt(d.temp) + ' C\n' +
    'Humidity:    ' + fmt(d.hum) + ' %\n' +
    'Pressure:    ' + fmt(d.pres) + ' hPa\n' +
    'Gas:         ' + fmt(d.gas) + ' ohm\n' +
    'PM1:         ' + fmt(d.pm1) + '\n' +
    'PM2.5:       ' + fmt(d.pm25) + '\n' +
    'PM10:        ' + fmt(d.pm10) + '\n' +
    'CO2:         ' + fmt(d.co2) + ' ppm\n' +
    'CO:          ' + fmt(d.co) + ' ppm\n' +
    'CO Temp:     ' + fmt(d.coTemp) + ' C\n' +
    'Wind Angle:  ' + fmt(d.windAngle) + ' deg\n' +
    'Wind Speed:  ' + fmt(d.windSpeed) + '\n\n' +
    '--- NB-IoT/MQTT uplink (Module B -> ThingsBoard) ---\n' +
    'State:       ' + n.state + (n.lastError && n.lastError !== 'none' ? ' (' + n.lastError + ')' : '') + '\n' +
    'Attached:    ' + (n.attached ? ('yes, ' + n.attachedSec + 's') : 'no') + '\n' +
    'MQTT:        ' + (n.mqttConnected ? 'connected' : 'not connected') + ' (' + n.mqttReconnects + ' reconnects)\n' +
    'RSSI:        ' + n.rssiDbm + ' dBm\n' +
    'Sent/Failed/Dropped: ' + n.sent + ' / ' + n.failed + ' / ' + n.dropped + '\n' +
    'Batch:       ' + n.ringCount + ' pending, target ' + n.batchReadings + ' readings / ' + n.batchSeconds + 's';
}

function poll() {
  fetch('/data').then(function (r) { return r.json(); }).then(render).catch(function () {
    document.getElementById('out').textContent = 'connection error';
  });
}
poll();
setInterval(poll, 3000);
</script>
</body>
</html>
)HTML";
