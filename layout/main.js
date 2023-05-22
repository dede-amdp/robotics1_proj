'use strict';

const input_canvas = document.getElementById('input_canvas'); // canvas element
const ctx = input_canvas.getContext('2d'); // canvas context
const send_btn = document.getElementById('send_data_btn'); // send button element
const online_status = document.getElementById('status-dot'); // circle showing serial communication status element
const serial_com_btn = document.getElementById('start-serial-btn'); // button to retry serial communication element
const line_btn = document.getElementById('line-btn'); // line tool button
const circle_btn = document.getElementById('circle-btn'); // circle tool button
const penup_btn = document.getElementById('penup-btn'); // pen-up button
var points = []; // list of points -> end effector coordinates
var circle_definition = []; // points needed to define a circle (radius and arc)
var man, traj; // manipulator and trajectory instance
var tool = line_tool; // currently used tool
var penup = false; // pen-up boolean
var dom_mouseX, dom_mouseY; // mouse position on canvas


settings = {
    'origin': { 'x': input_canvas.width / 2, 'y': input_canvas.height / 2 }, // origin of the manipulator
    'm_p': 1 / input_canvas.width, // m/p -> meters per pixel conversion factor
    'l1': 0.25, // length of the first arm
    'l2': 0.25, // length of the second arm
    's_step': 1/50, // slicing step size
    'framerate': 60 // animation framerate
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

/*
#@
@name: (eel) js_get_data
@brief: gives trajectory data to the python backend
@outputs: 
- list: list of json objs containing trajectory data
@#
*/
eel.expose(js_get_data);
function js_get_data() {
    var temp = [];
    // add to temp 
    for(var t of traj.data){
        var a,b, penup, c, r;
        if(t.type == 'line'){
            a = t.data[0];
            b = t.data[1];
            penup = t.data[2];
        }else{
            a = t.data[5];
            b = t.data[6];
            penup = t.data[4];
            c = t.data[0];
            r = t.data[1]
        }
        // trajectory = {'type', 'points', 'data'}
        // example:
        // line_t = {'type':'line', 'points': [p0, p1], 'data':[penup]}
        // circle_t = {'type':'circle', 'points': [a, b], 'data':[center, radius, penup, ...]}
        var to_send = {
            'type':t.type,
            'points':[[a.actX, a.actY], [b.actX, b.actY]],
            'data':t.type == 'line' ? {'penup':penup} : {'penup':penup, 'center':[c.actX, c.actY], 'radius':r}
        };
        temp.push(to_send);
    }
    points = []; //empty the array
    traj.reset(); // reset the trajectory
    draw_background();
    return temp; // return to python the sliced trajectory data
}

/*
#@
@name: serial_online
@brief: shows a red or green circle depending on the serial com. status
@inputs: 
- bool is_online: boolean stating if the serial communication is online
@#
*/
function serial_online(is_online) {
    // change the class of the status "badge" to make it show the status of the serial connection
    online_status.classList.remove('online');
    online_status.classList.remove('offline');
    if (is_online)
        online_status.classList.add('online');
    else
        online_status.classList.add('offline');
}

/*
#@
@name: draw_background
@brief: draws the background on the canvas
@notes: the background shows two circumferences with radii equal to the lengths of the two links of the manipulator, with red areas showing where the end effector should avoid going to avoid problems with cable intertwining, etc...
@inputs: 
- optional string color: background color;
- optional string line: line color;
- optional string limit: limited zones color;
@#
*/
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

/*
#@
@name: draw_point
@brief: draws a point on the canvas
@inputs: 
- float x: x coordinate of the point;
- float y: y coordinate of the point;
@#
*/
function draw_point(x, y) {
    ctx.beginPath();
    ctx.fillStyle = '#000000';
    ctx.strokeStyle = '#000000';
    ctx.arc(x, y, 7, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.fill();
    ctx.closePath();
}

/*
#@
@name: find_circ
@brief: find the circumference arc given 3 points
@notes: the circumference arc can be defined with 3 points:
* the first point determines where the arc should start;
* the second point determines the diameter of the circumference;
* the third point determines the angle of the arc, which is the angle between the vector from the center 
of the circumference to the first point and the vector from the center to the third point; 
@outputs:
- Point center: center of the circumference;
- Point a: starting point of the arc;
- Point p: ending point of the arc;
- float r: radius of the circumference;
- theta_0: angle of the vector a-c;
- theta_1: angle of the vector p-c;
@#
*/
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

/*
#@
@name: handle_input
@brief: handles the click event on the canvas
@inputs: 
- mouse event e;
@#
*/
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

/*
#@
@name: line_tool
@brief: method that shows the line tool gui
@#
*/
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

/*
#@
@name: circle_tool
@brief: method that shows the circle tool gui
@#
*/
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
    man.draw_traces(ctx) // -> slows down the animation a lot
    // draw the ui
    // these ui elements will be draw on top of everything else as long as the tool is used
    tool();
    // window.requestAnimationFrame(draw_loop);
    setTimeout(draw_loop, 1000/settings['framerate']);
}