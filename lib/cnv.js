'use strict';

/*

const incanvas = document.getElementById('input_canvas');
const ctx = incanvas.getContext('2d');

const settings = {
    'origin': { 'x': 0, 'y': incanvas.height / 2 },
    'm_p': 0.5 / incanvas.width, // m/p 
    'l1': 0.25,
    'l2': 0.25
};

*/


function rel2abs(x, y, settings) {
    var x_a = (x - settings['origin']['x']) * settings['m_p'];
    var y_a = -(y - settings['origin']['y']) * settings['m_p'];
    return [x_a, y_a];
}

function abs2rel(x, y, settings) {
    var x_p = x / settings['m_p'] + settings['origin']['x'];
    var y_p = -y / settings['m_p'] + settings['origin']['y'];
    return [x_p, y_p];
}

eel.expose(jsdraw_pose);
function jsdraw_pose(q) {
    if (!('settings' in window)) {
        console.log('This method requires the global variable `settings` that has to contain:\n' +
            '- `origin` field (js object) containing an x and y field,\n' +
            '- `m_p` field containing the conversion ration between meters and pixes (m/p),\n' +
            '- `l1` and `l2` field, containing the lengths of the two links of the planar manipulator.');
        return
    }
    if (!('ctx' in window)) {
        console.log('This method requires the canvas context to be a global variable called `ctx`');
        return
    }
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

eel.expose(jslog);
function jslog(msg) {
    console.log(msg);
}