const SerialPort = require('serialport');

'use strict';

const sendButton = document.getElementById("send-button");
const sendText = document.getElementById("send-text");
const outputText = document.getElementById("output");

sendButton.addEventListener("click", sendData);



const parser = new SerialPort.ReadlineParser({
    delimiter: '\r\n'
});

var port = new SerialPort.SerialPort({
    path: "COM3",
    baudRate: 115200,
    dataBits: 8,
    parity: 'none',
    stopBits: 1,
    flowControl: false
});

port.pipe(parser);

parser.on("data", handleData);

function handleData(data) {
    showData(data);
}

function showData(data) {
    outputText.innerHTML += data + "<br>";
}

function sendData(_) {
    showData("SENT:\t" + sendText.value + "<br>");
    port.write(sendText.value + "<br>");
    sendText.value = "";
}