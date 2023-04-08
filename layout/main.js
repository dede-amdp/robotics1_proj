'use strict';

const input_canvas = document.getElementById('input_canvas');
const ctx = input_canvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
const online_status = document.getElementById('status-dot');
var points = []; // list of points -> end effector coordinates

settings = {
    'origin': { 'x': 0, 'y': input_canvas.height / 2 },
    'm_p': 0.5 / input_canvas.width, // m/p 
    'l1': 0.25,
    'l2': 0.25
};

input_canvas.addEventListener('click', handle_input);
send_btn.addEventListener('click', handle_data);

main();

function main() {
    eel.py_serial_online()(serial_online);
    draw_background();
}

eel.expose(js_log);
function js_log(msg) {
    console.log(msg);
}

eel.expose(js_get_data);
function js_get_data() {
    var temp = [];
    //console.log("SENT DATA:");
    for (var p of points) {
        temp.push(p['actual']);
        //console.log(p);
    }
    points = []; //empty the array
    draw_background();
    return temp;
}

function serial_online(is_online) {
    online_status.classList.remove('online');
    online_status.classList.remove('offline');
    if (is_online)
        online_status.classList.add('online');
    else
        online_status.classList.add('offline');
}

function draw_background(color = '#EEEEEE', line = '#000000', limit = '#FF0000') {
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, input_canvas.clientWidth, input_canvas.clientHeight);
    ctx.beginPath();
    ctx.strokeStyle = line;
    ctx.fillStyle = line;
    ctx.moveTo(0, input_canvas.height / 2);
    ctx.lineTo(input_canvas.width, input_canvas.height / 2);
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.fillStyle = limit;
    ctx.arc(0, input_canvas.height / 2, input_canvas.height / 4, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.fillStyle = limit;
    ctx.arc(0, input_canvas.height / 2, input_canvas.height / 2, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
}

function draw_point(x, y) {
    ctx.beginPath();
    ctx.fillStyle = '#000000';
    ctx.strokeStyle = '#000000';
    ctx.arc(x, y, 7, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fill();
    ctx.closePath();
}

function handle_input(e) {
    var boundary = e.target.getBoundingClientRect();
    var x = e.clientX - boundary.left;
    var y = e.clientY - boundary.top;
    var rx, ry;
    [rx, ry] = rel2abs(x, y, settings);
    // add points to the list
    points.push({ 'actual': { 'x': rx, 'y': ry }, 'relative': { x, y } });
    draw_point(x, y);
}

function handle_data() {
    eel.py_get_data();
}