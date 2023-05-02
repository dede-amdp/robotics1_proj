'use strict';

/*

const incanvas = document.getElementById('input_canvas');
const ctx = incanvas.getContext('2d');
var traces = []; // list of drawable elements
var poses = []; // list of drawable elements

const settings = {
    'origin': { 'x': 0, 'y': incanvas.height / 2 },
    'm_p': 0.5 / incanvas.width, // m/p 
    'l1': 0.25,
    'l2': 0.25
};

*/

/* this is an initialized settings variable that SHOULD be overwritten with the correct values */
var settings = {
    'origin': { 'x': 0, 'y': 500 / 2 },
    'm_p': 0.5 / 500, // m/p 
    'l1': 0.25,
    'l2': 0.25
};


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

eel.expose(js_draw_pose);
function js_draw_pose(q) {
    /*
        This method requires the existence of a global variable called `settings`, that is for this reason initialized at the start of this script.
        That object can be modified with the correct values in the main.js file if necessary
     */
    var draw_method = function(q){
        ctx.beginPath();
        ctx.strokeStyle = '#000000';
        ctx.moveTo(settings['origin']['x'], settings['origin']['y']);
        var p1 = [settings['l1'] * Math.cos(q[0]), settings['l1'] * Math.sin(q[0])]
        var p2 = [p1[0] + settings['l2'] * Math.cos(q[0] + q[1]), p1[1] + settings['l2'] * Math.sin(q[0] + q[1])];
        var p1rel = abs2rel(p1[0], p1[1], settings);
        //console.log(p1, p1rel);
        ctx.lineTo(p1rel[0], p1rel[1]);
        ctx.moveTo(p1rel[0], p1rel[1]);
        var p2rel = abs2rel(p2[0], p2[1], settings);
        //console.log(p2, p2rel);
        ctx.lineTo(p2rel[0], p2rel[1]);
        ctx.stroke();
        ctx.closePath();
    }
    var data = [q];

    poses.push(new drawable(draw_method, data));
}

eel.expose(js_draw_traces);
function js_draw_traces(points, color = "#FF0000") {
    var draw_method = function(points, color = "#FF0000") {
        var trace = points.slice(0);
        for (var i = 0; i < trace.length; i++) {
            [trace[i]['x'], trace[i]['y']] = abs2rel(trace[i]['x'], trace[i]['y'], settings);
        }
        ctx.beginPath();
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        var p = trace.shift();
        ctx.moveTo(p['x'], p['y']);
        while (trace.length > 0) {
            p = trace.shift();
            ctx.lineTo(p['x'], p['y']);
            ctx.arc(p['x'], p['y'], 5, 0, 2 * Math.PI);
            ctx.moveTo(p['x'], p['y']);
        }
        ctx.stroke();
        ctx.closePath();
    }
    var data = [points, color];
    traces.push(new drawable(draw_method,data));
}


class drawable{
    constructor(draw_method, data){
        this.draw = draw_method;
        this.data = data;
    }
}