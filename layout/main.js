'use strict';

const input_canvas = document.getElementById('input_canvas');
const ctx = input_canvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
const online_status = document.getElementById('status-dot');
const serial_com_btn = document.getElementById('start-serial-btn');
const line_btn = document.getElementById('line-btn');
const circle_btn = document.getElementById('circle-btn');
var points = []; // list of points -> end effector coordinates
var circle_definition = [];
var man, traj;
var tool = line_tool;
var dom_mouseX, dom_mouseY;


settings = {
    'origin': { 'x': input_canvas.width / 2, 'y': input_canvas.height / 2 },
    'm_p': 1 / input_canvas.width, // m/p -> meters per pixel conversion factor
    'l1': 0.25,
    'l2': 0.25
};

input_canvas.addEventListener('click', handle_input);
send_btn.addEventListener('click', handle_data);
serial_com_btn.addEventListener('click', handle_serial);
line_btn.addEventListener('click', () => {tool = line_tool;});
circle_btn.addEventListener('click', () => {tool = circle_tool;});

input_canvas.addEventListener('mousemove', (e) => {
    var boundary = e.target.getBoundingClientRect();
    dom_mouseX = e.clientX - boundary.left;
    dom_mouseY = e.clientY - boundary.top;
});

main();

function main() {
    eel.py_serial_online()(serial_online);
    man = new Manipulator([-Math.PI/2, -Math.PI/2], settings);
    traj = new Trajectory();
    draw_loop();
}

eel.expose(js_log);
function js_log(msg) {
    console.log(msg);
}

eel.expose(js_get_data);
function js_get_data() {
    var temp = [];
    for (var p of points) {
        temp.push(p.actual);
    }
    points = []; //empty the array
    draw_background(); // REMOVED FOR DEBUG PURPOSES
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
    ctx.beginPath();
    ctx.strokeStyle = color;
    ctx.lineWidth = 1;
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, input_canvas.clientWidth, input_canvas.clientHeight);
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = line;
    ctx.lineWidth = 1;
    ctx.fillStyle = line;
    ctx.moveTo(0, input_canvas.height / 2);
    ctx.lineTo(input_canvas.width, input_canvas.height / 2);
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = limit;
    ctx.lineWidth = 1;
    ctx.fillStyle = limit;
    ctx.arc(settings['origin']['x'], settings['origin']['y'], input_canvas.height / 4, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
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
    the ending point of the arc is defined by the arc parameter
    */
    var n = points.length;
    var a = points[n-1];
    var b = circle_definition[1];
    var k = circle_definition[0];
    var p, arc;

    // use a and k to define the center and the radius
    var r = k.sub(a).mag()/2;
    var c = a.add(k.sub(a).scale(0.5))

    p = c.add(b.sub(c).set(r));

    var v1 = c.sub(a);
    var v2 = c.sub(p);
    //arc = Math.acos((v1.relX*v2.relX+v1.relY*v2.relY)/(v1.mag()*v2.mag()));

    var theta_0 = v1.angle()+Math.PI;
    var theta_1 = v2.angle()+Math.PI;

    return [c, p, r, theta_0, theta_1];
}

function handle_input(e) {
    var boundary = e.target.getBoundingClientRect();
    var x = e.clientX - boundary.left;
    var y = e.clientY - boundary.top;
    //var rx, ry;
    //[rx, ry] = rel2abs(x, y, settings);
    // add points to the list
    //points.push({ 'actual': { 'x': rx, 'y': ry }, 'relative': { x, y } });
    //draw_point(x, y); // REMOVED FOR DEBUG PURPOSES
    var n = points.length;
    // TODO: ADD TRAJECTORY TO traj
    if(tool == line_tool){
        circle_definition = []; // empty the circle_definition array to avoid having problems when the circle tool is selected afterwards
        points.push(new Point(x,y, settings));
        n +=1;
        if(n > 1) traj.add_line(points[n-2], points[n-1], 0);
        //if(n == 0) traj.add_line(null, points[n-1], 0);
    }else
    if(tool == circle_tool){
        if(n == 0){
            points.push(new Point(x,y, settings));
        }
        else{
            circle_definition.push(new Point(x,y, settings))
            if(circle_definition.length == 2){
                var c, p, r, theta_0, theta_1;
                [c, p, r, theta_0, theta_1] = find_circ();
                traj.add_circle(c, r, theta_0, theta_1, 0);
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
    man.draw_traces(ctx);
    // draw the ui
    // these ui elements will be draw on top of everything else as long as the tool is used
    tool();
    window.requestAnimationFrame(draw_loop);
}