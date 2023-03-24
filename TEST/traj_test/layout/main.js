'use strict';


const incanvas = document.getElementById('input_canvas');
const ctx = incanvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
var points = [];

settings = {
    'origin': { 'x': 0, 'y': incanvas.height / 2 },
    'm_p': 0.5 / incanvas.width, /* m/p */
    'l1': 0.25,
    'l2': 0.25
};

incanvas.addEventListener('click', handle_input);
send_btn.addEventListener('click', handle_data);
draw_background();



// === FUNCTION DEFINITIONS === //

eel.expose(jslog);
function jslog(msg) {
    console.log(msg);
}


eel.expose(jsget_points);
function jsget_points() {
    var temp = [];
    console.log("SENT DATA:");
    for (var p of points) {
        temp.push(p['actual']);
        console.log(p);
    }
    points = []; //empty the array
    canvas_update();
    return temp;
}

function draw_background(color = '#EEEEEE', line = '#000000', limit = '#FF0000') {
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, incanvas.clientWidth, incanvas.clientHeight);
    ctx.beginPath();
    ctx.strokeStyle = line;
    ctx.fillStyle = line;
    ctx.moveTo(0, incanvas.height / 2);
    ctx.lineTo(incanvas.width, incanvas.height / 2);
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.fillStyle = limit;
    ctx.arc(0, incanvas.height / 2, incanvas.height / 4, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.fillStyle = limit;
    ctx.arc(0, incanvas.height / 2, incanvas.height / 2, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
}

function handle_input(e) {
    var boundary = e.target.getBoundingClientRect();
    //DEBUG: console.log(boundary);
    var x = e.clientX - boundary.left;
    var y = e.clientY - boundary.top;
    //DEBUG: console.log(x, y);
    var rx, ry;
    [rx, ry] = rel2abs(x, y, settings);
    //DEBUG: console.log(rx, ry);
    // add points to the list
    points.push({ 'actual': { 'x': rx, 'y': ry }, 'relative': { x, y } });
    // update the canvas
    canvas_update();
}

function canvas_update() {
    draw_background();
    for (var p of points) {
        ctx.beginPath();
        ctx.fillStyle = '#000000';
        ctx.strokeStyle = '#000000';
        ctx.arc(p['relative']['x'], p['relative']['y'], 7, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.fill();
        ctx.closePath();
    }
}

function handle_data() {
    eel.pyget_data();
}