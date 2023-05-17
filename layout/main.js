'use strict';

const input_canvas = document.getElementById('input_canvas');
const ctx = input_canvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
const online_status = document.getElementById('status-dot');
const serial_com_btn = document.getElementById('start-serial-btn');
const line_btn = document.getElementById('line-btn');
const circle_btn = document.getElementById('circle-btn');
const penup_btn = document.getElementById('penup-btn');
var points = []; // list of points -> end effector coordinates
var circle_definition = [];
var man, traj;
var tool = line_tool;
var penup = false;
var dom_mouseX, dom_mouseY;


settings = {
    'origin': { 'x': input_canvas.width / 2, 'y': input_canvas.height / 2 },
    'm_p': 1 / input_canvas.width, // m/p -> meters per pixel conversion factor
    'l1': 0.25, // length of the first arm
    'l2': 0.25, // length of the second arm
    's_step': 1/50, // slicing step size
    'framerate': 5 // animation framerate //TODO: make it dynamic
};


// get GUI elements to assign event handlers
input_canvas.addEventListener('click', handle_input);
send_btn.addEventListener('click', handle_data);
serial_com_btn.addEventListener('click', handle_serial);
line_btn.addEventListener('click', () => {tool = line_tool;});
circle_btn.addEventListener('click', () => {tool = circle_tool;});
penup_btn.addEventListener('click', () => {penup = !penup;})

// get continuous updates on mouse position
input_canvas.addEventListener('mousemove', (e) => {
    var boundary = e.target.getBoundingClientRect();
    dom_mouseX = e.clientX - boundary.left;
    dom_mouseY = e.clientY - boundary.top;
});

main();

function main() {
    eel.py_serial_online()(serial_online); // check if the serial is online
    man = new Manipulator([-Math.PI/2, -Math.PI/2], settings); // create a new manipulator instance
    traj = new Trajectory(); // create a new trajectory object
    draw_loop(); // start the draw loop
}

eel.expose(js_log);
function js_log(msg) {
    console.log(msg);
}

eel.expose(js_get_data);
function js_get_data() {
    var temp = [];
    // add to temp 
    for(var t of traj.data){
        if(t.type == 'line'){
            const linef = function(s, data) {
                // p = pi+(pf-pi)*s, s in [0,...,1]
                var a,b,l;
                a = data[0]; // first point
                b = data[1]; // final point
                l = b.sub(a); // vector from a to b
                return a.add(l.scale(s)); // move from a to b but just by a distance of s*|l|
            };
            if(t.data[2]){
                // the pen is up
                if(temp.length == 0){
                    // if this is the first trajectory, add also the first point
                    // otherwise it will ignore the first point of the trajectory if the pen is up
                    var a = t.data[0].actual;
                    a.z = 1;
                    temp.push(a);
                }
                // don't do any slicing
                var b = t.data[1].actual;
                b.z = 1;
                temp.push(b);
                continue;
            }
            // slice the motion primitive with a step size of `s_step`
            for(var s = temp.length == 0 ? 0 : settings['s_step'] ; s <= 1; s+=settings['s_step']){
                temp.push(linef(s, t.data).actual);
            }
        }
        else if(t.type == 'circle'){
            const circf = function(s, data){
                var c, r, theta_0, theta_1, d, A, B, ccw;
                c = data[0]; // center
                r = data[1]; // radius
                theta_0 = data[2]; // start angle
                theta_1 = data[3]; // end angle
                d = -Math.abs(theta_1-theta_0); // delta angle
                A = theta_0 >theta_1; // first condition
                B = Math.abs(theta_1-theta_0)<Math.PI; // second condition
                ccw = (!A&&!B)||(A&&B); // counter clockwise motion
                if (!ccw) d *= -1; // clockwise motion

                var sd = theta_0+d*s;
                sd = sd >= 0 ? sd : 2*Math.PI+sd; // maintain sd within the domain
                sd = sd <= 2*Math.PI ? sd : sd-2*Math.PI;
                var p = new Point(
                    r*Math.cos(sd),
                    r*Math.sin(sd),
                    settings
                );
                return c.add(p); // rotate from the starting by an angle of `sd` radians
            };
            if(t.data[4]){
                // the pen is up
                if(temp.length == 0){
                    // if this is the first trajectory, add also the first point
                    // otherwise it will ignore the first point of the trajectory if the pen is up
                    var a = t.data[5].actual;
                    a.z = 1;
                    temp.push(a);
                }
                // don't do any slicing 
                var p = t.data[6].actual;
                p.z = 1;
                temp.push(p);
                continue;
            }
            // slice the motion primitive with a step of `s_step`
            for(var s = temp.length == 0 ? 0 : settings['s_step']; s <= 1; s+=settings['s_step']){
                temp.push(circf(s, t.data).actual);
            }
        }
    }
    points = []; //empty the array
    traj.reset(); // reset the trajectory
    draw_background();
    return temp; // return to python the sliced trajectory data
}

function serial_online(is_online) {
    // change the class of the status "badge" to make it show the status of the serial connection
    online_status.classList.remove('online');
    online_status.classList.remove('offline');
    if (is_online)
        online_status.classList.add('online');
    else
        online_status.classList.add('offline');
}

function draw_background(color = '#EEEEEE', line = '#000000', limit = '#FF0000') {
    // draw white background
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, input_canvas.clientWidth, input_canvas.clientHeight);
    ctx.closePath();
    // draw transparent red background (for off limits areas)
    ctx.beginPath();
    ctx.fillStyle = limit+"22";
    ctx.lineStyle = "#00000000";
    ctx.fillRect(0,0, input_canvas.width, input_canvas.height);
    ctx.stroke();
    ctx.closePath();
    // draw white half-circle
    ctx.beginPath();
    ctx.fillStyle = color;
    ctx.lineStyle = "#00000000";
    ctx.arc(settings['origin']['x'], settings['origin']['y'], input_canvas.width/2, -Math.PI/2, +Math.PI/2, false);
    ctx.fill();
    ctx.closePath();
    // draw x axis
    ctx.beginPath();
    ctx.strokeStyle = line;
    ctx.lineWidth = 1;
    ctx.fillStyle = line;
    ctx.moveTo(0, input_canvas.height / 2);
    ctx.lineTo(input_canvas.width, input_canvas.height / 2);
    ctx.stroke();
    ctx.closePath();
    // draw inner circumference
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.lineWidth = 1;
    ctx.fillStyle = limit;
    ctx.arc(settings['origin']['x'], settings['origin']['y'], input_canvas.height / 4, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
    // draw outer circumference
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.lineWidth = 1;
    ctx.fillStyle = limit;
    ctx.arc(settings['origin']['x'], settings['origin']['y'], input_canvas.height / 2, 0, 2 * Math.PI);
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

function find_circ(){
    /*
    circle_definition = [
        - point defining the diameter of the circumference
        - point defining the arc of the circumference
    ]
    the starting point of the arc is the previous point in the trajectory
    the ending point of the arc is defined by the second circle_definition point
    */
    var n, a, b, k, p
    n = points.length;
    a = points[n-1]; // starting point of the circumference
    b = circle_definition[1]; // point needed to define the diameter
    k = circle_definition[0]; // point needed to define end point of the arc

    // use a and k to define the center and the radius
    var r = k.sub(a).mag()/2; // radius
    var c = a.add(k.sub(a).scale(0.5)) // center

    p = c.add(b.sub(c).set(r)); // final point of the arc

    var v1 = c.sub(a);
    var v2 = c.sub(p);

    var theta_0 = v1.angle()+Math.PI; // problems with how js handles angles
    var theta_1 = v2.angle()+Math.PI;

    return [c, a, p, r, theta_0, theta_1];
}

function handle_input(e) {
    var boundary = e.target.getBoundingClientRect();
    var x = e.clientX - boundary.left;
    var y = e.clientY - boundary.top;
    // if the point is on the left side of the canvas, ignore it
    if(x <input_canvas.width/2) return;
    // if the point is outside the outer circumference, ignore it
    if(Math.pow(x-settings.origin.x, 2)+Math.pow(y-settings.origin.y, 2) > Math.pow(input_canvas.height/2,2)) return;
    var n = points.length;
    if(tool == line_tool){
        circle_definition = []; // empty the circle_definition array to avoid having problems when the circle tool is selected afterwards
        points.push(new Point(x,y, settings));
        n +=1;
        if(n > 1) traj.add_line(points[n-2], points[n-1], penup);
    }else
    if(tool == circle_tool){
        if(n == 0){
            points.push(new Point(x,y, settings));
        }
        else{
            circle_definition.push(new Point(x,y, settings))
            if(circle_definition.length == 2){
                var c, a, p, r, theta_0, theta_1;
                [c, a, p, r, theta_0, theta_1] = find_circ();
                traj.add_circle(c, r, theta_0, theta_1, penup, a, p);
                points.push(p);
                circle_definition = [];
            }
        }
    }

    man.reset_trace();
}

function handle_data() {
    eel.py_get_data();
}

function handle_serial() {
    eel.py_serial_startup()();
    eel.py_serial_online()(serial_online);
}

function line_tool(){
    // use the mouse position (dom_mouseX, dom_mouseY) to show the correct ui
    if(points.length > 0){
        var last_point = points[points.length-1].relative;
        ctx.beginPath();
        ctx.strokeStyle = "#00000077";
        ctx.moveTo(last_point['x'], last_point['y']);
        ctx.lineTo(dom_mouseX, dom_mouseY);
        ctx.stroke();
        ctx.closePath();
    }
}

function circle_tool(){
    // use the mouse position (dom_mouseX, dom_mouseY) to show the correct ui
    if(circle_definition.length == 0 && points.length > 0){
        // draw a circumference with variable size
        var m = new Point(dom_mouseX, dom_mouseY, settings);
        var a = points[points.length-1];
        var c = a.add(m.sub(a).scale(0.5)); // center
        var r = m.sub(a).mag()/2; // radius

        ctx.beginPath();
        ctx.strokeStyle = "#00000077";
        ctx.lineWidth = 5;
        ctx.arc(c.relX, c.relY, r, 0, 2*Math.PI);
        ctx.stroke();
        ctx.closePath();
    }else{
        if(circle_definition.length == 1 && points.length > 0){
            // the circumference was set, the arc is to be defined

            var b = new Point(dom_mouseX, dom_mouseY, settings);
            var m = circle_definition[0];
            var a = points[points.length-1];
            var c = a.add(m.sub(a).scale(0.5)); // center
            var r = m.sub(a).mag()/2; // radius


            ctx.beginPath();
            ctx.strokeStyle = "#00000077";
            ctx.lineWidth = 5;
            ctx.arc(c.relX, c.relY, r, 0, 2*Math.PI);
            ctx.stroke();
            ctx.closePath();
            ctx.beginPath();
            ctx.strokeStyle = "#00000077";
            ctx.lineWidth = 3;
            ctx.moveTo(c.relX, c.relY);
            ctx.lineTo(b.relX, b.relY);
            ctx.stroke();
            ctx.closePath();
        }
    }
}


/* === ANIMATION === */


function draw_loop(){
    draw_background();
    for(var p of points) draw_point(p.relative['x'], p.relative['y']);
    traj.draw(ctx);
    man.draw_pose(ctx);
    man.draw_traces(ctx); // -> slows down the animation a lot
    // draw the ui
    // these ui elements will be draw on top of everything else as long as the tool is used
    tool();
    // window.requestAnimationFrame(draw_loop);
    setTimeout(draw_loop, 1000/settings['framerate']);
}