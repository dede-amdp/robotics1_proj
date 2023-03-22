'use strict';

const settings = {
    'origin': { 'x': 0, 'y': 250 },
    'rwidth': 0.25,
    'rheight': 0.25
};


const incanvas = document.getElementById('input_canvas');
const ctx = incanvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
var points = [];

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
    for (var p of points) {
        temp.push(p['actual']);
    }
    points = []; //empty the array
    console.log("SENT DATA:", temp);
    canvas_update();
    return temp;
}

function draw_background(color = '#EEEEEE') {
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, incanvas.clientWidth, incanvas.clientHeight);
}

function handle_input(e) {
    var boundary = e.target.getBoundingClientRect();
    //DEBUG: console.log(boundary);
    var x = e.clientX - boundary.left;
    var y = e.clientY - boundary.top;
    //DEBUG: console.log(x, y);
    var px = x - settings['origin']['x'];
    var py = (incanvas.height - y) - settings['origin']['y'];
    var rx = (px / incanvas.width) * settings['rwidth'];
    var ry = (py / incanvas.height) * settings['rheight'];
    //DEBUG: console.log(rx, ry);
    // add points to the list
    points.push({ 'actual': { 'x': rx, 'y': ry }, 'relative': { x, y } });
    // update the canvas
    canvas_update();
}

function canvas_update() {
    draw_background();
    for (var p of points) {
        ctx.fillStyle = '#000000';
        ctx.beginPath();
        ctx.arc(p['relative']['x'], p['relative']['y'], 7, 0, 2 * Math.PI);
        ctx.stroke();
        ctx.fill();
        ctx.closePath();
    }
}

function handle_data() {
    console.log("Handling data");
    eel.pyget_data();
}