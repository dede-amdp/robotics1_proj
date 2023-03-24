'use strict';


const incanvas = document.getElementById('input_canvas');
const ctx = incanvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
var points = [];

const settings = {
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

eel.expose(jsdraw_pose);
function jsdraw_pose(q) {
    ctx.beginPath();
    ctx.strokeStyle = '#000000';
    ctx.moveTo(settings['origin']['x'], settings['origin']['y']);
    var p1 = [settings['l1'] * Math.cos(q[0]), settings['l1'] * Math.sin(q[0])]
    var p2 = [p1[0] + settings['l2'] * Math.cos(q[0] + q[1]), p1[1] + settings['l2'] * Math.sin(q[0] + q[1])];
    var p1rel = abs2rel(p1[0], p1[1]);
    console.log(p1, p1rel);
    ctx.lineTo(p1rel[0], p1rel[1]);
    ctx.moveTo(p1rel[0], p1rel[1]);
    var p2rel = abs2rel(p2[0], p2[1]);
    console.log(p2, p2rel);
    ctx.lineTo(p2rel[0], p2rel[1]);
    ctx.stroke();
    ctx.closePath();
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
    [rx, ry] = rel2abs(x, y);
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

function rel2abs(x, y) {
    var x_a = (x - settings['origin']['x']) * settings['m_p'];
    var y_a = -(y - settings['origin']['y']) * settings['m_p'];
    return [x_a, y_a];
}

function abs2rel(x, y) {
    var x_p = x / settings['m_p'] + settings['origin']['x'];
    var y_p = -y / settings['m_p'] + settings['origin']['y'];
    return [x_p, y_p];
}