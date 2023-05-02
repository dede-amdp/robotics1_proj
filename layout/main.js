'use strict';

const input_canvas = document.getElementById('input_canvas');
const ctx = input_canvas.getContext('2d');
const send_btn = document.getElementById('send_data_btn');
const online_status = document.getElementById('status-dot');
const serial_com_btn = document.getElementById('start-serial-btn');
const line_btn = document.getElementById('line-btn');
const circle_btn = document.getElementById('circle-btn');
var points = []; // list of points -> end effector coordinates
//var to_draw = []; // list of drawable elements
var traces = []; // list of drawable traces
var poses = []; // list of drawable poses
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

document.addEventListener('mousemove', (e) => {dom_mouseX=e.pageX; dom_mouseY=e.pageY; });

main();

function main() {
    eel.py_serial_online()(serial_online);
    draw_loop();
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
    ctx.fillStyle = color;
    ctx.fillRect(0, 0, input_canvas.clientWidth, input_canvas.clientHeight);
    ctx.closePath();
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
    ctx.arc(settings['origin']['x'], settings['origin']['y'], input_canvas.height / 4, 0, 2 * Math.PI);
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    ctx.strokeStyle = limit;
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

function handle_input(e) {
    var boundary = e.target.getBoundingClientRect();
    var x = e.clientX - boundary.left;
    var y = e.clientY - boundary.top;
    var rx, ry;
    [rx, ry] = rel2abs(x, y, settings);
    // add points to the list
    points.push({ 'actual': { 'x': rx, 'y': ry }, 'relative': { x, y } });
    //draw_point(x, y); // REMOVED FOR DEBUG PURPOSES
    // clear drawable elements
    traces = [];
    poses = [];
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
        var last_point = points[points.length-1]['relative'];
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
}


/* === ANIMATION === */


function draw_loop(){
    draw_background();
    for(var p of points) draw_point(p['relative']['x'], p['relative']['y']);
    for(var element of traces) element.draw(...element.data);
    for(var element of poses) element.draw(...element.data);
    // draw the ui
    // these ui elements will be draw on top of everything else as long as the tool is used
    tool();
    window.requestAnimationFrame(draw_loop);
}