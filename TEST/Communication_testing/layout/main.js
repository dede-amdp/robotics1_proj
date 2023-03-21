'use strict';

var input_element = document.getElementById("send-text");
var send_button = document.getElementById("send-button");
var output_paragraph = document.getElementById("output");

send_button.addEventListener("click", send_message);

eel.expose(jslog);
function jslog(msg) {
    console.log(msg);
}

function send_message() {
    var msg = input_element.value.toString();
    update_output_paragraph(`SENT:<br>${msg}<br>RECEIVED:`);
    input_element.value = '';
    eel.write_serial(msg)();
    eel.read_serial()(update_output_paragraph);
}

function update_output_paragraph(msg) {
    console.log(msg);
    output_paragraph.innerHTML += msg + "<br>";
}

function main() {
    eel.pylog("JS Started");
    //eel.write_serial("Serial Test")();
    //eel.read_serial()(console.log);
}

main();