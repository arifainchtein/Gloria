var currentData;
var REFRESH_SECONDS = 5;
var OPERATING_MODES = {1: 'Sleep', 2: 'No LED', 3: 'Full Mode', 4: 'Cloudy', 5: 'Comma'};
var BATTERY_CHEMISTRIES = {0: 'Lead-acid', 1: 'LiFePO4'};

$(document).ready(function() {
    refreshData();
    refreshScreen();
    // Live values: only redraw while the main cards are showing, so a config form being filled
    // in is never wiped out from under the user.
    loadPumpData(showPumpCard);
    setInterval(function() {
        if ($('#wifi-configure').hasClass('d-none') && $('#system-info').hasClass('d-none') && $('#power-info').hasClass('d-none') && $('#relay-info').hasClass('d-none') && $('#pump-info').hasClass('d-none')) {
            refreshData(true);
            loadPumpData(showPumpCard);
        } else if (!$('#relay-info').hasClass('d-none')) {
            refreshData(false, true);  // the relay panel's counters keep moving
        } else if (!$('#pump-info').hasClass('d-none')) {
            loadPumpData(pumpEditorRefresh);  // status and values move; the rule being edited is kept
        }
    }, REFRESH_SECONDS * 1000);
});

function ajaxError(data) {
    console.log("solo:" + JSON.stringify(data));
    alert("There was an error processing your request:" + JSON.stringify(data));
    return false;
}

//
// wifi functions
//
$(document).on('click', '#wifi-switch', function() {
    if ($("#wifi-switch").is(':checked')) {
        setStationMode();
    } else {
        setAccessPointMode();
    }
});

function setAccessPointMode() {
    $('#wifi-switch').prop('checked', false);
    $('.ap-mode').removeClass('d-none').addClass('d-block');
    $('.station-mode').removeClass('d-block').addClass('d-none');
    $('#ap-address').val(currentData.soft_ap_ssid);
    $('#ap-password').val('');
    $('#ap-hostname').val(currentData.hostname);
}

function setStationMode() {
    var ssids = [];
    try {
        ssids = JSON.parse(currentData.ssids);
    } catch (e) {
        ssids = [];
    }
    $('.station-mode').removeClass('d-none').addClass('d-block');
    $('.ap-mode').removeClass('d-block').addClass('d-none');
    $('#ssid').find('option').remove().end();
    $('#wifi-switch').prop('checked', true);
    $.each(ssids, function(i, ss) {
        var optionText = ss.ssid + '   ' + ss.rssi + '  ' + (ss.enc ? 'pwd' : '');
        $('#ssid').append($("<option>", {
            value: ss.ssid,
            text: optionText
        }));
    });
    if (currentData.ssid != '') {
        $('#ssid').val(currentData.ssid);
    }
    $('#wifi-password').val('');
    $('#hostname').val(currentData.hostname);
}

$(document).on('click', '#wifi-config-icon', function() {
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#wifi-configure').removeClass('d-none').addClass('d-block');
    if (currentData.stationmode) {
        setStationMode();
    } else {
        setAccessPointMode();
    }
});

$(document).on('click', '#wifi-configure-cancel', function() {
    refreshScreen();
});

$(document).on('click', '#wifi-configure-submit', function() {
    if ($("#wifi-switch").is(':checked')) {
        var ssid = $('#ssid').val();
        var pass = $('#wifi-password').val();
        var host = $('#hostname').val();
        if (ssid == undefined || ssid == '') {
            alert("Please input SSID");
            return false;
        }
        if (host == undefined || host == '') {
            alert("Please input Host");
            return false;
        }
        $.ajax({
            type: "POST",
            url: "/LangleyServlet",
            data: {formName: 'ConfigSTA', ssid: ssid, pass: pass, host: host},
            success: function(result) {
                currentData = JSON.parse(result);
                refreshScreen();
            },
            error: ajaxError
        });
    } else {
        var apaddress = $('#ap-address').val();
        var apPass = $('#ap-password').val();
        var apHost = $('#ap-hostname').val();
        if (apaddress == undefined || apaddress == '') {
            alert("Please input AP Address");
            return false;
        }
        if (apHost == undefined || apHost == '') {
            alert("Please input Host");
            return false;
        }
        $.ajax({
            type: "POST",
            url: "/LangleyServlet",
            data: {formName: 'ConfigAP', apaddress: apaddress, pass: apPass, host: apHost},
            success: function(result) {
                currentData = JSON.parse(result);
                refreshScreen();
            },
            error: ajaxError
        });
    }
    alert("Please wait 5 seconds and refresh");
});

//
// system info, time and GPS
//
$(document).on('click', '#system-info-icon', function() {
    $('.display-module').removeClass('d-block').addClass('d-none');
    if ($('#system-info').hasClass('d-block')) {
        $('#system-info').removeClass('d-block').addClass('d-none');
    } else {
        $('#system-info').removeClass('d-none').addClass('d-block');
    }
    if (currentData.internetAvailable) {
        $('#internet-time-configure-button').removeClass('d-none').addClass('d-block');
        $('#internet-ping-time').html(currentData.internetPingTime);
    } else {
        $('#internet-time-configure-button').removeClass('d-block').addClass('d-none');
        $('#internet-ping-time').html('');
    }
    $('#latitude').val(currentData.latitude == null ? '' : currentData.latitude);
    $('#longitude').val(currentData.longitude == null ? '' : currentData.longitude);
});

$(document).on('click', '#system-info-cancel', function() {
    refreshScreen();
});

$(document).on('click', '#manual-time-button', function() {
    var date = new Date();
    var day = ("0" + date.getDate()).slice(-2);
    var month = ("0" + (date.getMonth() + 1)).slice(-2);
    var today = date.getFullYear() + "-" + month + "-" + day + 'T' + ("0" + date.getHours()).slice(-2) + ":" + ("0" + date.getMinutes()).slice(-2) + ":" + ("0" + date.getSeconds()).slice(-2);
    $('#date-input').val(today);
    $('#manual-time-modal').modal('show');
});

$(document).on('click', '#manual-time-submit', function() {
    var date = new Date($('#date-input').val());
    if (date.getTime() < 10000) {
        alert('Please input valid time');
        return false;
    }
    var year = date.getFullYear() - 2000;
    var time = "SetTime#" + date.getDate() + "#" + (date.getMonth() + 1) + "#" + year + "#" + date.getDay() + "#" + date.getHours() + "#" + date.getMinutes() + "#" + date.getSeconds();
    $.ajax({
        type: "POST",
        url: "/LangleyServlet",
        data: {formName: "ManualSetTime", time: time},
        success: function(result) {
            $('#manual-time-modal').modal('hide');
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: ajaxError
    });
});

$(document).on('click', '#internet-time-configure-button', function() {
    $.ajax({
        type: "POST",
        url: "/LangleyServlet",
        data: {formName: "SetTimeViaInternet"},
        success: function(result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: ajaxError
    });
});

//
// GPS: the Latitude/Longitude fields take decimal degrees (-33.859722) or degrees minutes
// seconds as the iPhone Compass shows them (33°51′35″ S), or typed with spaces (33 51 35 S).
// S and W (or a minus sign) make it negative. Converted to decimal before it is saved.
//
function parseCoordinate(text, positiveLetter, negativeLetter, max) {
    var s = String(text).trim().toUpperCase();
    if (s == '') return null;
    var sign = 1;
    var m = s.match(/^([NSEW])\s*(.*)$/) || s.match(/^(.*?)\s*([NSEW])$/);
    if (m) {
        var letter = /^[NSEW]$/.test(m[1]) ? m[1] : m[2];
        s = /^[NSEW]$/.test(m[1]) ? m[2] : m[1];
        if (letter == negativeLetter) sign = -1;
        else if (letter != positiveLetter) return null;
    }
    if (s.charAt(0) == '-') {
        sign = -1;
        s = s.substring(1);
    }
    var parts = s.split(/[\s°º˚′'’″"]+/).filter(function(p) { return p != ''; });
    if (parts.length < 1 || parts.length > 3) return null;
    for (var i = 0; i < parts.length; i++) {
        if (!/^\d+(\.\d+)?$/.test(parts[i])) return null;
    }
    var deg = parseFloat(parts[0]);
    var min = parts.length > 1 ? parseFloat(parts[1]) : 0;
    var sec = parts.length > 2 ? parseFloat(parts[2]) : 0;
    if (min >= 60 || sec >= 60) return null;
    var value = deg + min / 60 + sec / 3600;
    if (value > max) return null;
    return Math.round(sign * value * 1000000) / 1000000;
}

function showCoordinatePreview(field, positiveLetter, negativeLetter, max) {
    var text = $('#' + field).val();
    var value = parseCoordinate(text, positiveLetter, negativeLetter, max);
    var preview = $('#' + field + '-preview');
    if (String(text).trim() == '') preview.css('color', '').text('');
    else if (value == null) preview.css('color', '#c0392b').text('Not understood');
    else preview.css('color', '#198754').text('= ' + value);
}

$(document).on('input', '#latitude', function() { showCoordinatePreview('latitude', 'N', 'S', 90); });
$(document).on('input', '#longitude', function() { showCoordinatePreview('longitude', 'E', 'W', 180); });

$(document).on('click', '#save-gps-button', function() {
    var latText = $('#latitude').val();
    var longText = $('#longitude').val();
    if (latText.trim() == '') {
        alert("Please input the latitude");
        return false;
    }
    if (longText.trim() == '') {
        alert("Please input the longitude");
        return false;
    }
    var lat = parseCoordinate(latText, 'N', 'S', 90);
    var long = parseCoordinate(longText, 'E', 'W', 180);
    if (lat == null) {
        alert("The latitude is not understood. Use decimal (-33.859722) or degrees minutes seconds (33 51 35 S).");
        return false;
    }
    if (long == null) {
        alert("The longitude is not understood. Use decimal (151.211111) or degrees minutes seconds (151 12 40 E).");
        return false;
    }
    $.ajax({
        type: "POST",
        url: "/LangleyServlet",
        data: {formName: "SetGPS", lat: lat, long: long},
        success: function(result) {
            currentData = JSON.parse(result);
            refreshScreen();
        },
        error: ajaxError
    });
});

//
// raw data (tap the device name)
//
$(document).on('click', '.show-raw-data', function() {
    var tabledata = '<table class="table table-striped">';
    for (const key in currentData) {
        if (key != 'ssids') {
            tabledata += '<tr><td>' + key + '</td><td>' + currentData[key] + '</td></tr>';
        } else {
            var ssids = [];
            try {
                ssids = JSON.parse(currentData[key]);
            } catch (e) {
                ssids = [];
            }
            tabledata += '<tr><td>' + key + '</td><td>';
            for (let i = 0; i < ssids.length; i++) {
                let s = ssids[i];
                var icon = s.secure == 0 ? '' : '<i class="fa fa-lock"></i>';
                tabledata += s.ssid + " rssi=" + s.rssi + " ch=" + s.channel + " " + icon + '<br>';
            }
            tabledata += '</td></tr>';
        }
    }
    tabledata += '</table>';
    $('#raw-data-table').empty().append(tabledata);
    $('#raw-data-title').html(currentData.devicename);
    $('#raw-data-json').empty().append('<code>' + JSON.stringify(currentData) + '</code>');
    $('#show-raw-data-modal').modal('show');
});

//
// data
//
function refreshData(redraw, relayOnly) {
    $.ajax({
        type: "GET",
        async: !!redraw || !!relayOnly,
        url: "/LangleyServlet",
        data: {formName: "GetWebData"},
        success: function(result) {
            currentData = JSON.parse(result);
            if (redraw) refreshScreen();
            showRelayIcon();
            if (relayOnly) showRelayInfo();
        },
        error: function(data) {
            // A live refresh failing (device asleep, WiFi dropped) is expected - only the first,
            // page-load request is worth an alert.
            console.log("solo:" + JSON.stringify(data));
            if (!redraw && !relayOnly) alert("There was an error processing your request:" + JSON.stringify(data));
        }
    });
}

function show(id) {
    $('#' + id).removeClass('d-none').addClass('d-block');
}

function fixed(value, digits) {
    var n = Number(value);
    return isNaN(n) ? '-' : n.toFixed(digits);
}

function showRuntime(hours) {
    if (!hours || hours <= 0) return 'runtime: n/a';
    if (hours >= 48) return 'runtime: ' + (hours / 24).toFixed(1) + ' days';
    return 'runtime: ' + hours.toFixed(1) + ' h';
}

function showBattery() {
    $('#langley-battery-value').html(fixed(currentData.batteryVoltage, 2));
    var mA = Number(currentData.batteryCurrentMa);
    var flow = mA >= 0 ? 'charging ' + mA.toFixed(0) + ' mA' : 'discharging ' + (-mA).toFixed(0) + ' mA';
    var chemistry = BATTERY_CHEMISTRIES[currentData.batteryChemistry] || '';
    $('#langley-battery-detail').html(flow + '<br>' + chemistry + ', ' + showRuntime(currentData.estimatedRuntime));
    show('langley-battery');
}

function showSolar() {
    $('#langley-solar-value').html(fixed(currentData.solarVoltage, 1));
    $('#langley-solar-detail').html(fixed(currentData.solarCurrentMa, 0) + ' mA');
    show('langley-solar');
}

function showTemperature() {
    $('#langley-temperature-value').html(fixed(currentData.temperature, 1));
    show('langley-temperature');
}

function showMode() {
    $('#langley-mode-value').html(OPERATING_MODES[currentData.operatingStatus] || '-');
    $('#langley-mode-detail').html('overnight use: ' + fixed(currentData.overnightMah, 0) + ' mAh');
    show('langley-mode');
}

function showLora() {
    $('#langley-lora-value').html(fixed(currentData.rssi, 0));
    $('#langley-lora-detail').html('SNR ' + fixed(currentData.snr, 1) + ' dB');
    show('langley-lora');
}

function refreshScreen() {
    $('.display-module').removeClass('d-block').addClass('d-none');

    $('#host-name').html(currentData.devicename);
    $('#ssid-name').html(currentData.ssid);
    var d = new Date(currentData.secondsTime * 1000);
    var minutes = String(d.getMinutes()).padStart(2, '0');
    var seconds = String(d.getSeconds()).padStart(2, '0');
    $('#time').html(d.getHours() + ":" + minutes + ":" + seconds + " " + d.getDate() + "/" + (d.getMonth() + 1) + "/" + d.getFullYear());

    $('#lora-logo').css('color', currentData.lora ? 'blue' : 'red');

    $('#serial-number').html(currentData.serialnumber);
    $('#temperature').html(currentData.temperature + '&#8451;');
    $("#rtc-bat-volt").html(fixed(currentData.rtcBatVolt, 2));
    if (currentData.rtcBatVolt >= 2.7) {
        $('#rtc-battery-icon').css("color", "green");
    } else if (currentData.rtcBatVolt >= 2.5) {
        $('#rtc-battery-icon').css("color", "yellow");
    } else {
        $('#rtc-battery-icon').css("color", "red");
    }

    show('wifi-info');
    showPumpCard();
    showBattery();
    showSolar();
    showTemperature();
    showMode();
    if (currentData.lora) showLora();
}

// ---- Power panel (the bolt under the cog) - same layout as Daffodil's, with the Valentino Build 3
// charger, thermistor and coin cell. Data from /LangleyServlet?formName=GetPowerData every 5 s.
var powerTimer = null;
var RESET_REASONS = {0: 'Unknown', 1: 'Power on', 2: 'External pin', 3: 'Software', 4: 'Crash', 5: 'Interrupt watchdog',
    6: 'Task watchdog', 7: 'Watchdog', 8: 'Deep sleep', 9: 'Brownout', 10: 'SDIO'};
var CHARGER_STATES = {0: 'unknown', 1: 'no input power', 2: 'charging', 3: 'charge complete', 4: 'suspended / fault / no battery', 5: 'invalid STAT levels'};
// Charge stage from the firmware (battery voltage and current while charging). Lead-acid: bulk ->
// absorption -> float. LiFePO4: constant current -> constant voltage -> complete.
var LEAD_ACID_STAGES = {1: 'bulk', 2: 'absorption', 3: 'float (full)', 4: 'complete'};
var LIFEPO4_STAGES = {1: 'constant current', 2: 'constant voltage', 3: 'constant voltage', 4: 'complete'};

function chargerText(d, live){
    var state = CHARGER_STATES[live.chargerState] || '-';
    var stages = d.chemistry == 1 ? LIFEPO4_STAGES : LEAD_ACID_STAGES;
    if(live.chargerState == 2 && stages[live.chargeStage]) return 'charging: ' + stages[live.chargeStage];
    return state;
}

function pwRow(label, value){
    return '<div class="pw-row"><span>' + label + '</span><span>' + value + '</span></div>';
}

function pwDuration(sec){
    sec = Math.round(sec);
    if(sec < 60) return sec + ' s';
    if(sec < 3600) return Math.floor(sec / 60) + ' min';
    if(sec < 86400) return Math.floor(sec / 3600) + ' h ' + Math.floor((sec % 3600) / 60) + ' min';
    return Math.floor(sec / 86400) + ' d ' + Math.floor((sec % 86400) / 3600) + ' h';
}

// Langley's battery current is positive while charging
function pwCurrent(ma){
    var a = Math.abs(ma).toFixed(0) + ' mA';
    if(Math.abs(ma) < 5) return a;
    return a + (ma > 0 ? ' charging' : ' out');
}

function showPowerInfo(d){
    var live = d.live || {};
    var batteryV = live.batteryV || 0;
    var soc = Math.round(live.socPercent >= 0 ? live.socPercent : 0);
    var color = soc >= 50 ? '#2e7d32' : soc >= 20 ? '#f9c74f' : '#c62828';
    $('#pw-battery-fill').css({width: '100%', background: color});
    $('#pw-battery-text').text(batteryV > 1 ? soc + ' %' : 'no battery').css('color', soc >= 20 && soc < 50 ? '#000' : '#fff');
    // While charging the charger lifts the terminal voltage, so the state of charge reads high
    var floating = live.chargeStage == 3;
    var charging = live.batteryMa > 5 && !floating;
    $('#pw-battery-fill').css('opacity', charging ? 0.45 : 1);
    if(charging) $('#pw-battery-text').text(soc + ' %*').css('color', '#000');
    // Float: the charge current has tapered to a trickle at the set voltage, so the battery is full
    if(floating) $('#pw-battery-fill').css('background', '#2e7d32'), $('#pw-battery-text').text('Full').css('color', '#fff');
    $('#pw-soc-note').html(batteryV <= 1 ? '' : floating
        ? 'Float: held at ' + (live.chargeVolts || 0).toFixed(1) + ' V, ' + Math.max(0, live.batteryMa).toFixed(0) + ' mA trickle'
        : charging
        ? '<span style="color:#ef6c00">* Charging (' + ((d.chemistry == 1 ? LIFEPO4_STAGES : LEAD_ACID_STAGES)[live.chargeStage] || 'charging') + ') - the voltage reads high, so this is too high. It settles 10-30 min after charging stops.</span>'
        : 'about ' + Math.round(soc / 100 * d.capacityMah) + ' of ' + d.capacityMah + ' mAh &middot; ' + (BATTERY_CHEMISTRIES[d.chemistry] || '') + ' &middot; estimated from ' + batteryV.toFixed(2) + ' V');

    var html = '';
    html += pwRow('Battery', batteryV.toFixed(2) + ' V');
    html += pwRow('Battery current', pwCurrent(live.batteryMa || 0));
    html += pwRow('Solar panel', (live.panelV || 0).toFixed(2) + ' V');
    html += pwRow('Solar panel current', Math.max(0, live.panelMa || 0).toFixed(0) + ' mA');
    html += pwRow('Charger', chargerText(d, live));
    html += pwRow('Charge voltage (JP7/JP8)', (live.chargeVolts || 0).toFixed(1) + ' V (set in firmware)');
    html += pwRow('Battery temperature', live.batteryTempC !== undefined ? live.batteryTempC.toFixed(1) + ' &deg;C' : 'no thermistor');
    html += pwRow('Mode', OPERATING_MODES[live.operatingStatus] || ('Mode ' + live.operatingStatus));
    html += pwRow('Pump relay output', live.pumpRelay ? 'on' : 'off');
    html += pwRow('Emergency switch SW1', live.emergencyClosed ? 'closed' : 'open');
    html += pwRow('Awake for', pwDuration(d.uptimeSec));
    $('#pw-sensors').html(html);

    html = '';
    html += pwRow('Chemistry (JP3)', BATTERY_CHEMISTRIES[d.chemistry] || '-');
    html += pwRow('STAT1 / STAT2', (live.stat1V || 0).toFixed(2) + ' V / ' + (live.stat2V || 0).toFixed(2) + ' V');
    html += pwRow('Board temperature', (live.boardTempC || 0).toFixed(1) + ' &deg;C');
    html += pwRow('RTC coin cell', live.rtcBatV > 0 ? live.rtcBatV.toFixed(2) + ' V' : 'not read');
    $('#pw-board').html(html);

    var v = d.vital;
    if(!v){
        $('#pw-vital').html('<div style="font-size:14px;color:#6c757d">Not built yet this boot - the first report goes out with the next LoRa send.</div>');
        return;
    }
    html = '';
    html += pwRow('Report built', pwDuration(v.ageSec) + ' ago (#' + v.seq + ')');
    html += pwRow('Resets', v.resetCount + ' - last ' + (RESET_REASONS[v.lastResetReason] || v.lastResetReason)
        + (v.lastResetAgoSec ? ', ' + pwDuration(v.lastResetAgoSec) + ' ago' : ''));
    html += pwRow('Wakes', v.wakeCount + (v.earlyWakeCount ? ' (' + v.earlyWakeCount + ' early)' : '') + (v.commaWakeCount ? ', ' + v.commaWakeCount + ' comma' : ''));
    html += pwRow('Awake / asleep', pwDuration(v.awakeSec) + ' / ' + pwDuration(v.sleptSec));
    html += pwRow('Wake voltage', v.wakeV ? v.wakeV.toFixed(2) + ' V' : '-');
    html += pwRow('Lowest voltage', v.minV ? v.minV.toFixed(2) + ' V' : '-');
    html += pwRow('Charge in', v.chargeInMah.toFixed(1) + ' mAh');
    html += pwRow('Charge out', v.dischargeOutMah.toFixed(1) + ' mAh');
    html += pwRow('Panel energy', v.panelInWh.toFixed(2) + ' Wh');
    html += pwRow('Last LoRa send', v.txMs + ' ms, peak ' + v.txPeakMa + ' mA' + (v.txMinV ? ', dip ' + v.txMinV.toFixed(2) + ' V' : ''));
    html += pwRow('LoRa failures', v.loraTxFails);
    html += pwRow('Firmware', v.firmwareBuild);
    $('#pw-vital').html(html);
}

function loadPowerInfo(){
    $.ajax({
        type: "GET",
        url: "/LangleyServlet",
        data: {formName: "GetPowerData"},
        success: function(result){ showPowerInfo(JSON.parse(result)); },
        error: function(){ $('#pw-sensors').html('<div style="color:#c62828">Cannot reach the Langley</div>'); }
    });
}

function stopPowerInfo(){
    if(powerTimer){ clearInterval(powerTimer); powerTimer = null; }
}

$(document).on('click', '#power-info-icon', function(){
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#power-info').removeClass('d-none').addClass('d-block');
    loadPowerInfo();
    stopPowerInfo();
    powerTimer = setInterval(loadPowerInfo, 5000);
});

$(document).on('click', '#power-info-close', function(){
    stopPowerInfo();
    refreshScreen();
});

$(document).on('click', '#system-info-icon', function(){
    stopPowerInfo();
});

// ---- Relay panel (2026-11-11) - the R under the bolt. currentData.relay comes with GetWebData:
// {on, strongDbm, heard, sent, strong, dup, dropped, last, lastAction, lastRssi, lastAgeSec}.
// The R is green while relaying, grey when off.
function showRelayIcon() {
    var r = currentData && currentData.relay;
    $('#relay-icon-letter').css('color', r && r.on ? '#2e7d32' : '#6c757d');
}

function showRelayInfo() {
    var r = currentData && currentData.relay;
    if (!r) { $('#relay-status').html('This firmware has no relay'); return; }
    $('#relay-switch').prop('checked', !!r.on);
    if (!$('#relay-strong').data('touched')) $('#relay-strong').val(String(r.strongDbm));
    if (!$('#relay-minv').is(':focus') && r.minVolts !== undefined) $('#relay-minv').val(Number(r.minVolts).toFixed(1));
    if (r.minVolts !== undefined) {
        $('#relay-awake').html(!r.on ? ''
            : r.keepingAwake ? '&#9790; Staying awake only to relay - battery ' + fixed(r.battery, 2) + ' V (LEDs and displays off)'
            : r.battery >= r.minVolts ? 'Battery ' + fixed(r.battery, 2) + ' V - above ' + Number(r.minVolts).toFixed(1) + ' V, will stay awake to relay'
            : '<span style="color:#c62828">Battery ' + fixed(r.battery, 2) + ' V - below ' + Number(r.minVolts).toFixed(1) + ' V: sleeps as usual, relays only while awake</span>');
    }
    $('#relay-status').html(r.on
        ? '<b style="color:#2e7d32">ON</b> &middot; ' + r.sent + ' relayed, ' + r.strong + ' not relayed (strong), ' + r.dup + ' already relayed'
            + (r.dropped ? ', ' + r.dropped + ' dropped' : '')
        : '<b style="color:#6c757d">OFF</b>');
    var t = '';
    if (r.on && r.last !== undefined && r.lastAgeSec < 60) {
        var ago = r.lastAgeSec < 2 ? 'just now' : r.lastAgeSec + ' s ago';
        t = r.lastAction === 'strong'
            ? '&#8856; <b>' + r.last + '</b> received but not relayed - strong signal (' + r.lastRssi + ' dBm) &middot; ' + ago
            : '&#8594; <b>' + r.last + '</b> relayed ' + ago;
    }
    $('#relay-ticker').html(t);
}

function setRelay() {
    $.ajax({
        type: "POST",
        url: "/LangleyServlet",
        data: {formName: "SetRelay", on: $('#relay-switch').is(':checked') ? 1 : 0, dbm: $('#relay-strong').val(), minv: $('#relay-minv').val()},
        success: function(result) {
            $('#relay-strong').data('touched', false);
            // applied on Langley's next loop pass - read it back shortly
            setTimeout(function() { refreshData(false, true); }, 1500);
        },
        error: ajaxError
    });
}

$(document).on('click', '#relay-info-icon', function() {
    if (typeof stopPowerInfo === 'function') stopPowerInfo();
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#relay-info').removeClass('d-none').addClass('d-block');
    showRelayInfo();
    refreshData(false, true);
});
$(document).on('change', '#relay-switch', setRelay);
$(document).on('click', '#relay-minv-save', function() {
    var v = parseFloat($('#relay-minv').val());
    if (isNaN(v) || v < 9 || v > 15) { alert('Enter a voltage between 9 and 15'); return; }
    setRelay();
});
$(document).on('change', '#relay-strong', function() { $(this).data('touched', true); setRelay(); });
$(document).on('click', '#relay-info-close', function() { refreshScreen(); });

// ---- Pump (2026-11-18) - the Pump and Flow cards, and the rule editor (the P under the R). Data from
// /GloriaServlet?formName=GetPumpData: {status, rule, types:[{id,name,fields:[{id,label,units}]}],
// devices:[{t,name,age,rssi,stale,values:{field: value|null}}]}. The rows look like the colour rules of the
// devices.digitalstables.com tab editor: device, variable, comparison, value. Device names arrive over the
// radio, so everything here is set as text, never as HTML.
var pumpData = null;
var pumpEdit = null;  // the rule being edited - a copy, kept while the status refreshes
var PUMP_OPS = [['<', 'below'], ['<=', 'at or below'], ['>', 'above'], ['>=', 'at or above']];
var PUMP_OP_SYMBOL = {'<': '<', '<=': '≤', '>': '>', '>=': '≥'};

function loadPumpData(then) {
    $.ajax({
        type: 'GET', url: '/GloriaServlet', data: {formName: 'GetPumpData'}, dataType: 'json',
        success: function(d) { pumpData = d; showPumpIcon(); if (then) then(); },
        error: function(e) { console.log('pump: ' + JSON.stringify(e)); }
    });
}

function pumpType(id) { return (pumpData && pumpData.types || []).find(function(t) { return t.id === id; }); }
function pumpField(typeId, fieldId) { var t = pumpType(typeId); return t ? t.fields.find(function(f) { return f.id === fieldId; }) : null; }
function pumpDevice(typeId, name) { return (pumpData && pumpData.devices || []).find(function(d) { return d.t === typeId && d.name === name; }); }
function pumpAge(s) { return s < 90 ? s + ' s' : s < 5400 ? Math.round(s / 60) + ' min' : (s / 3600).toFixed(1) + ' h'; }
function pumpFieldText(f) { return f ? f.label + ' (' + f.units + ')' : '?'; }

// The condition's value now, and whether it holds: {text, holds (true/false/null = no data)}
function pumpConditionNow(c) {
    var d = pumpDevice(c.t, c.d);
    if (!d) return {text: 'not heard', holds: null};
    if (d.stale) return {text: 'stale - last heard ' + pumpAge(d.age) + ' ago', holds: null};
    var v = d.values[c.f];
    if (v === null || v === undefined) return {text: 'no reading', holds: null};
    var f = pumpField(c.t, c.f);
    var holds = c.op === '<' ? v < c.v : c.op === '<=' ? v <= c.v : c.op === '>' ? v > c.v : v >= c.v;
    return {text: 'now ' + Number(v).toFixed(1) + ' ' + (f ? f.units : ''), holds: holds};
}

function showPumpIcon() {
    var st = pumpData && pumpData.status;
    var color = !st ? '#6c757d' : st.fault ? '#c62828' : st.stale ? '#d4a000' : st.pump ? '#2e7d32' : '#6c757d';
    $('#pump-icon-letter').css('color', color);
}

function pumpStatusText(st) {
    var t = (st.auto ? 'AUTO' : 'MANUAL') + ' · pump ' + (st.pump ? 'ON' : 'off') + ' (' + st.reason + ')';
    if (st.pump) t += ' for ' + pumpAge(st.onSec);
    return t;
}

function showPumpCard() {
    if (!pumpData) return;
    var st = pumpData.status;
    $('#gloria-pump-value').text(st.pump ? 'ON' : 'off').css('color', st.fault ? '#c62828' : st.pump ? '#2e7d32' : '');
    $('#gloria-pump-mode').text(st.auto ? 'AUTO' : 'MANUAL');
    var detail = st.fault ? 'stopped: no flow - press START or clear it in the rule (P)'
        : st.stale ? 'data stale - the rule cannot be worked out' : st.reason;
    if (st.sw1 === 'open') detail += ' · emergency switch OPEN';
    $('#gloria-pump-detail').text(detail);
    show('gloria-pump');
    $('#gloria-flow-value').text(fixed(st.flow, 1));
    $('#gloria-flow-detail').text(fixed(st.total, 0) + ' L since start-up');
    show('gloria-flow');
}

// ---- the editor
function pumpOpenEditor() {
    if (typeof stopPowerInfo === 'function') stopPowerInfo();
    $('.display-module').removeClass('d-block').addClass('d-none');
    $('#pump-info').removeClass('d-none').addClass('d-block');
    $('#pump-message').text('');
    loadPumpData(function() {
        pumpEdit = JSON.parse(JSON.stringify(pumpData.rule));
        $('#pump-stale').val(Math.round(pumpEdit.stale / 60));
        $('#pump-dryrun').val(pumpEdit.dryRunSec);
        $('#pump-manualmin').val(pumpEdit.manualMin);
        $('#pump-manualminv').val(Number(pumpEdit.manualMinV).toFixed(1));
        $('#pump-qfactor').val(pumpEdit.qfactor);
        $('#pump-autowhenlow').val(pumpEdit.autoWhenLow ? '1' : '0');
        pumpDrawRows('start');
        pumpDrawRows('stop');
        pumpEditorRefresh();
    });
}

// Status, the heard devices and each row's "now" line - not the rows themselves, so nothing being typed is lost
function pumpEditorRefresh() {
    if (!pumpData || !pumpEdit) return;
    var st = pumpData.status;
    var status = $('#pump-status').empty();
    status.append($('<b>').text(pumpStatusText(st)));
    if (st.stale) status.append($('<div>').css('color', '#9a7400').text('Data stale: a device the rule uses is not heard - pump off'));
    if (st.fault) status.append($('<div>').css('color', '#c62828').text('Stopped: the pump ran with no flow. Clear the fault here or press START.'));
    if (st.sw1 === 'open') status.append($('<div>').css('color', '#c62828').text('Emergency switch SW1 is open - the relay cannot close'));
    status.append($('<div>').text('Flow ' + fixed(st.flow, 1) + ' L/min · ' + fixed(st.total, 0) + ' L since start-up'));
    $('#pump-clear-fault').toggleClass('d-none', !st.fault);
    ['start', 'stop'].forEach(function(list) {
        pumpEdit[list].forEach(function(c, i) { pumpShowNow(list, i); });
    });
    pumpShowDevices();
}

function pumpShowDevices() {
    var box = $('#pump-devices').empty();
    var devices = pumpData.devices || [];
    if (!devices.length) { box.text('None yet. Devices appear here once Gloria has received their data.'); return; }
    devices.forEach(function(d) {
        var t = pumpType(d.t);
        var line = $('<div>').css('margin-bottom', '4px');
        line.append($('<b>').text(d.name));
        line.append(document.createTextNode(' (' + (t ? t.name : d.t) + ') heard ' + pumpAge(d.age) + ' ago, ' + d.rssi + ' dBm' + (d.stale ? ' - STALE' : '')));
        var vals = [];
        (t ? t.fields : []).forEach(function(f) {
            var v = d.values[f.id];
            if (v !== null && v !== undefined) vals.push(f.label + ' ' + Number(v).toFixed(1) + ' ' + f.units);
        });
        if (vals.length) line.append($('<div>').css({'color': '#6c757d', 'font-size': '12px'}).text(vals.join(' · ')));
        box.append(line);
    });
}

function pumpShowNow(list, i) {
    var c = pumpEdit[list][i];
    var now = pumpConditionNow(c);
    var el = $('#pump-' + list + '-now-' + i);
    var holdsText = now.holds === null ? '' : now.holds ? ' — holds' : ' — does not hold';
    el.text(now.text + holdsText).css('color', now.holds === null ? '#9a7400' : now.holds ? '#2e7d32' : '#6c757d');
}

function pumpDrawRows(list) {
    var box = $('#pump-' + list + '-rows').empty();
    var conds = pumpEdit[list];
    if (!conds.length) box.append($('<div>').css({'color': '#6c757d', 'font-size': '13px', 'margin': '4px 0'}).text('None'));
    conds.forEach(function(c, i) {
        var row = $('<div>').css({'display': 'flex', 'flex-wrap': 'wrap', 'gap': '6px', 'align-items': 'center', 'padding': '6px 0', 'border-top': i ? '1px solid #e3e7e4' : '0'});
        row.append($('<span>').css('font-size', '13px').text(i === 0 ? 'When' : list === 'start' ? 'and' : 'or'));
        // device: the ones heard, plus this condition's own if it is not heard now
        var dev = $('<select class="form-select form-select-sm">').css('width', 'auto').attr('aria-label', 'Device');
        var seen = false;
        (pumpData.devices || []).forEach(function(d) {
            var t = pumpType(d.t);
            if (!t) return;
            var o = $('<option>').val(d.t + '|' + d.name).text(d.name);
            if (d.t === c.t && d.name === c.d) { o.prop('selected', true); seen = true; }
            dev.append(o);
        });
        if (!seen) dev.prepend($('<option>').val(c.t + '|' + c.d).text(c.d + ' (not heard)').prop('selected', true));
        dev.on('change', function() {
            var parts = dev.val().split('|');
            var newType = parts.shift();
            c.d = parts.join('|');
            if (newType !== c.t) { c.t = newType; var t = pumpType(newType); c.f = t && t.fields.length ? t.fields[0].id : ''; }
            pumpDrawRows(list);
        });
        row.append(dev);
        // variable of that device type
        var field = $('<select class="form-select form-select-sm">').css('width', 'auto').attr('aria-label', 'Variable');
        var t = pumpType(c.t);
        (t ? t.fields : []).forEach(function(f) { field.append($('<option>').val(f.id).text(pumpFieldText(f))); });
        field.val(c.f);
        field.on('change', function() { c.f = field.val(); pumpShowNow(list, i); });
        row.append(field);
        // comparison
        var op = $('<select class="form-select form-select-sm">').css('width', 'auto').attr('aria-label', 'Comparison');
        PUMP_OPS.forEach(function(o) { op.append($('<option>').val(o[0]).text(PUMP_OP_SYMBOL[o[0]] + ' (' + o[1] + ')')); });
        op.val(c.op);
        op.on('change', function() { c.op = op.val(); pumpShowNow(list, i); });
        row.append(op);
        // value
        var val = $('<input class="form-control form-control-sm" type="number" step="any">').css('width', '90px').attr('aria-label', 'Value').val(c.v);
        val.on('input', function() { c.v = parseFloat(val.val()); pumpShowNow(list, i); });
        row.append(val);
        var del = $('<button type="button" class="btn btn-outline-danger btn-sm" title="Remove">×</button>');
        del.on('click', function() { conds.splice(i, 1); pumpDrawRows(list); pumpEditorRefresh(); });
        row.append(del);
        row.append($('<div>').attr('id', 'pump-' + list + '-now-' + i).css({'width': '100%', 'font-size': '12px'}));
        box.append(row);
    });
    conds.forEach(function(c, i) { pumpShowNow(list, i); });
}

function pumpAddCondition(list) {
    if (pumpEdit[list].length >= 4) { alert('Up to 4 conditions'); return; }
    var devices = (pumpData.devices || []).filter(function(d) { return pumpType(d.t); });
    // the same device as the row above, else the first heard
    var prev = pumpEdit[list].length ? pumpEdit[list][pumpEdit[list].length - 1] : (pumpEdit.start[0] || null);
    var c;
    if (prev) {
        c = {t: prev.t, d: prev.d, f: prev.f, op: list === 'stop' ? '>=' : '<', v: prev.v};
    } else if (devices.length) {
        var t = pumpType(devices[0].t);
        c = {t: devices[0].t, d: devices[0].name, f: t.fields[0].id, op: '<', v: 0};
    } else {
        alert('No devices heard yet. A device appears once Gloria has received its data with this farm\'s authenticator code.');
        return;
    }
    pumpEdit[list].push(c);
    pumpDrawRows(list);
}

function pumpMessage(text, ok) {
    $('#pump-message').text(text).css('color', ok ? '#2e7d32' : '#c62828');
}

function pumpCode() {
    var code = $('#pump-code').val().trim();
    if (!/^\d{6,8}$/.test(code)) { pumpMessage('Enter the authenticator code (6 digits)', false); return null; }
    return code;
}

function pumpSave() {
    var rule = {
        stale: Math.round(parseFloat($('#pump-stale').val()) * 60),
        dryRunSec: parseInt($('#pump-dryrun').val(), 10),
        manualMin: parseInt($('#pump-manualmin').val(), 10),
        manualMinV: parseFloat($('#pump-manualminv').val()),
        qfactor: parseFloat($('#pump-qfactor').val()),
        autoWhenLow: $('#pump-autowhenlow').val() === '1',
        start: pumpEdit.start, stop: pumpEdit.stop
    };
    if (isNaN(rule.stale) || rule.stale < 60) { pumpMessage('Stale: at least 1 minute', false); return; }
    if (isNaN(rule.dryRunSec) || rule.dryRunSec < 2 || rule.dryRunSec > 255) { pumpMessage('No-flow time: 2 to 255 s', false); return; }
    if (isNaN(rule.manualMin) || rule.manualMin < 1) { pumpMessage('MANUAL time: at least 1 minute', false); return; }
    if (isNaN(rule.manualMinV) || rule.manualMinV < 9 || rule.manualMinV > 15) { pumpMessage('MANUAL battery: 9 to 15 V', false); return; }
    if (isNaN(rule.qfactor) || rule.qfactor <= 0) { pumpMessage('Flow meter factor must be above 0', false); return; }
    var bad = rule.start.concat(rule.stop).find(function(c) { return typeof c.v !== 'number' || isNaN(c.v); });
    if (bad) { pumpMessage('Every condition needs a value', false); return; }
    if (!rule.start.length && !confirm('No start conditions: in AUTO the pump will never run. Save anyway?')) return;
    var code = pumpCode();
    if (!code) return;
    pumpMessage('Saving...', true);
    $.ajax({
        type: 'POST', url: '/GloriaServlet', data: {formName: 'SetPumpRule', rule: JSON.stringify(rule), code: code},
        success: function() {
            $('#pump-code').val('');
            pumpMessage('Saved', true);
            // applied on Gloria's next loop pass - read it back
            setTimeout(function() { loadPumpData(function() { pumpEdit = JSON.parse(JSON.stringify(pumpData.rule)); pumpDrawRows('start'); pumpDrawRows('stop'); pumpEditorRefresh(); }); }, 1500);
        },
        error: function(e) { pumpMessage(e.responseText || 'Not saved', false); }
    });
}

function pumpClearFault() {
    var code = pumpCode();
    if (!code) return;
    $.ajax({
        type: 'POST', url: '/GloriaServlet', data: {formName: 'ClearPumpFault', code: code},
        success: function() { $('#pump-code').val(''); pumpMessage('Fault cleared', true); setTimeout(function() { loadPumpData(pumpEditorRefresh); }, 1500); },
        error: function(e) { pumpMessage(e.responseText || 'Not cleared', false); }
    });
}

$(document).on('click', '#pump-info-icon', pumpOpenEditor);
$(document).on('click', '#gloria-pump', pumpOpenEditor);
$(document).on('click', '#pump-start-add', function() { pumpAddCondition('start'); });
$(document).on('click', '#pump-stop-add', function() { pumpAddCondition('stop'); });
$(document).on('click', '#pump-save', pumpSave);
$(document).on('click', '#pump-clear-fault', pumpClearFault);
$(document).on('click', '#pump-info-close', function() { pumpEdit = null; refreshScreen(); });
