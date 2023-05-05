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

/* The following piece of code uses the global variable "man" which is an instance of the class "Manipulator" */
eel.expose(js_draw_pose);
function js_draw_pose(q) {
    man.q = q;
}

eel.expose(js_draw_traces);
function js_draw_traces(points) {
    for(var i = 0; i < points[0].length; i++){
        man.add2trace([points[0][i], points[1][i]]);
    }
}