/*
#@
@name: class Manipulator
@brief: class that represents a 2Dof manipulator and its trajectory;
@inputs: 
- list q: generalized coordinate values;
@#
*/
class Manipulator{
    constructor(q, settings){
        this.q_coords = q;
        this.settings = settings;
        [this.p, this.end_eff] = this.dk(q);
        this.traces = {'x1':[],'x2':[]};
    }

    // SETTERS AND GETTERS
    set q(q){
        this.q_coords = q;
        [this.p, this.end_eff] = this.dk(this.q_coords); // set also the end effector position
    }

    set eef(pos){
        return // it cannot be modified directly
    }

    add2trace(q){
        var x1, x2;
        [x1, x2] = this.dk(q);
        this.traces['x1'].push(x1);
        this.traces['x2'].push(x2);
    }

    add_trace(points){
        this.traces = {'x1':[],'x2':[]};
        var p1, p2;
        for( var point of points){
            [p1, p2] = abs2rel(point['x'], point['y'], settings);
            this.traces['x1'].push(p1);
            this.traces['x2'].push(p2);
        }
    }

    reset_trace(){
        this.traces = {'x1':[],'x2':[]};
    }

/*
#@
@name: Manipulator.draw_pose    
@brief: draws the two links of the manipulator
@inputs: 
- canvas context 2D ctx;
@#
*/
    draw_pose(ctx){
        settings = this.settings;
        ctx.beginPath();
        ctx.lineWidth = 3;
        ctx.strokeStyle = "#000000";
        ctx.moveTo(settings['origin']['x'], settings['origin']['y']);
        ctx.lineTo(this.p[0], this.p[1]);
        ctx.moveTo(this.p[0], this.p[1]);
        ctx.lineTo(this.end_eff[0], this.end_eff[1]);
        ctx.stroke();
        ctx.closePath();
    }

/*
#@
@name: async Manipulator.draw_traces
@brief: draws the "traces" of the manipulator (the trajectory that the links should have followed)
@notes: the method was made async so that it may not increase too much the page reaction time;
@inputs: 
- canvas context 2D ctx;
- optional list[string] colors: list of 2 colors to use for the links traces;
@#
*/
    async draw_traces(ctx, colors = ['#0000FF','#00FF00']){ // made async so that the animation loop does not slow down
        ctx.beginPath();
        ctx.lineWidth = 5;
        ctx.strokeStyle = colors[0];
        if(this.traces['x1'].length < 1 || this.traces['x2'].length < 1) return false;
        ctx.moveTo(this.traces['x1'][0][0], this.traces['x1'][0][1]);
        for(var p of this.traces['x1']){
            ctx.lineTo(p[0], p[1]);
            ctx.moveTo(p[0], p[1]);
        }
        ctx.stroke();
        ctx.closePath();
        ctx.beginPath();
        ctx.lineWidth = 5;
        ctx.strokeStyle = colors[1];
        ctx.moveTo(this.traces['x2'][0][0], this.traces['x2'][0][1]);
        for(var p of this.traces['x2']){
            ctx.lineTo(p[0], p[1]);
            ctx.moveTo(p[0], p[1]);
        }
        ctx.stroke();
        ctx.closePath();
        return true;
    }

/*
#@
@name: Manipulator.dk
@brief: computes the direct kinematics of the manipulator
@inputs: 
- list[float] q: configuration of the robot;
@outputs: 
- list[float]: list containing the positions of the ending points (list of coordinates) of the links
@#
*/
    dk(q){
        settings = this.settings;
        var p1 = [settings['l1'] * Math.cos(q[0]), settings['l1'] * Math.sin(q[0])]
        var p2 = [p1[0] + settings['l2'] * Math.cos(q[0] + q[1]), p1[1] + settings['l2'] * Math.sin(q[0] + q[1])];
        var p1rel = abs2rel(p1[0], p1[1], settings); // intermediate point position
        var p2rel = abs2rel(p2[0], p2[1], settings); // end effector position
        return [p1rel, p2rel];
    }
}

/*
#@
@name: class Point
@brief: class modeling a point in the operational space
@inputs: 
- float x: x coordinate of the point;
- float y: y coordinate of the point;
- dict settings: dictionary containing data for coordinate conversion (from canvas space to operational space)
@#
*/
class Point{
    constructor(x, y, settings){
        this.settings = settings;
        this.relative = {x, y};
        var rx, ry;
        [rx, ry] = rel2abs(x, y, settings);
        this.actual = {'x':rx, 'y': ry, 'z': 0};
    }

    // SETTERS AND GETTERS
    set relX(x){
        this.relative.x = x;
        var rx, ry;
        [rx, ry] = rel2abs(this.relX, this.relY, this.settings);
        this.actual = {'x':rx, 'y': ry, 'z': 0};
    }

    get relX(){
        return this.relative.x;
    }

    set relY(y){
        this.relative.y = y;
        var rx, ry;
        [rx, ry] = rel2abs(this.relX, this.relY, this.settings);
        this.actual = {'x':rx, 'y': ry, 'z': 0};
    }

    get actX(){
        return this.actual.x;
    }

    get actY(){
        return this.actual.y;
    }

    get actZ(){
        return this.actual.z;
    }

    set actX(x){
        this.actual.x = x;
        var rx, ry;
        [rx, ry] = abs2rel(this.actX, this.actY, this.settings);
        this.relative = {'x':rx, 'y': ry};
    }

    set actY(y){
        this.actual.y = y;
        var rx, ry;
        [rx, ry] = abs2rel(this.actX, this.actY, this.settings);
        this.relative = {'x':rx, 'y': ry};
    }

    set actZ(z){
        this.actual.z = z;
    }

    get relY(){
        return this.relative.y;
    }

    // OPERATIONS 
/*
#@
@name: Point.add
@brief: adds to point vectors together
@inputs: 
- Point other: Point to add;
@outputs: 
- Point: result
@#
*/
    add(other){
        var result = new Point(this.relX, this.relY, this.settings);
        result.relX = result.relX+other.relX;
        result.relY = result.relY+other.relY;
        return result;
    }

/*
#@
@name: Point.sub
@brief: subtracts two point vectors together
@inputs: 
- Point other: point to subtract;
@outputs: 
- Point: result
@#
*/
    sub(other){
        var result = new Point(this.relX, this.relY, this.settings);
        result.relX = result.relX-other.relX;
        result.relY = result.relY-other.relY;
        return result;
    }

/*
#@
@name: Point.mag
@brief: computes the length of the point vector
@outputs: 
- float: length of the point vector
@#
*/
    mag(){
        return Math.sqrt(this.relX*this.relX+this.relY*this.relY);
    }

/*
#@
@name: Point.scale
@brief: scales the point vector length with the specified scalar
@inputs: 
- float scalar: value used to scale the length;
@outputs: 
- Point: scaled point vector
@#
*/
    scale(scalar){
        var result = new Point(0, 0, this.settings);
        //result.relX = scalar*result.relX;
        //result.relY = scalar*result.relY;
        var rho = scalar*this.mag();
        var theta = Math.atan2(this.relY, this.relX);
        result.relX = rho*Math.cos(theta);
        result.relY = rho*Math.sin(theta);
        return result;
    }

/*
#@
@name: Point.rot    
@brief: rotates (of the specified angle) the point vector around its origin
@inputs: 
float delta: angle (in radians) used for the rotation (a positive rotation is counterclockwise);
@outputs: 
- Point: rotated point vector
@#
*/
    rot(delta){
        var result = new Point(0, 0, this.settings);
        var rho = this.mag();
        var theta = Math.atan2(this.relY, this.relX) + delta;
        result.relX = rho*Math.cos(theta);
        result.relY = rho*Math.sin(theta);
        return result;
    }

/*
#@
@name: Point.set
@brief: sets the length of the point vector to the specified value
@inputs: 
- float scalar: length of the new vector;
@outputs: 
- Point: scaled point vector;
@#
*/
    set(scalar){
        var result = new Point(0, 0, this.settings);
        var rho = scalar;
        var theta = Math.atan2(this.relY, this.relX);
        result.relX = rho*Math.cos(theta);
        result.relY = rho*Math.sin(theta);
        return result;
    }

/*
#@
@name: Point.angle
@brief: computes the angle of the point vector (in radians)
@outputs: 
- float: angle of the point vector
@#
*/
    angle(){
        return Math.atan2(this.relY, this.relX);
    }
}

/*
#@
@name: class Trajectory
@brief: class that models a trajectory made fo multiple patches
@notes: the trajectory modelled by this class can be made up of two different kind of patches:
* line: a linear movement from point A to point B;
* circle: a curvilinear movement between two points (A and P) by following a circumference with given center and radius; 
@#
*/
class Trajectory{
    constructor(){
        this.data = [];
    }

/*
#@
@name: Trajectory.add_line
@brief: adds a line patch to the trajectory
@inputs: 
- Point p0: starting point of the patch;
- Point p1: ending point of the patch;
- bool raised: boolean that states wether the pen on the end effector of the manipulator should be raised or not;
@#
*/
    add_line(p0, p1, raised){
        this.data.push({'type':'line', 'data':[p0, p1, raised]}) // start and end point
    }

/*
#@
@name: Trajectory.add_circle
@brief: adds a circular patch to the trajectory
@inputs: 
- Point c: center of the circumference;
- float r: radius of the circumference;
- float theta_0: angle of the starting point;
- float theta_1: angle of the ending point;
- bool raised: states wether the pen on the end effector of the manipulator should be up or not;
- Point a: starting point of the circumference;
- Point p: ending point of the circumference;
@#
*/
    add_circle(c, r, theta_0, theta_1, raised, a, p){
        this.data.push({'type':'circle', 'data': [c, r, theta_0, theta_1, raised, a, p]}) // point and diameter
    }

/*
#@
@name: Trajectory.reset
@brief: resets the trajectory data
@#
*/
    reset(){
        this.data = [];
    }

/*
#@
@name: Trajectory.draw
@brief: draws the trajectory
@notes: each patch of the trajectory will be drawn on the canvas specified by the user
@inputs: 
- 2D canvas context ctx: context of the canvas that will be used to draw on;
@#
*/
    draw(ctx){
        for(var traj of this.data){
            if(traj.type == 'line'){
                if(traj.data[2]) continue
                ctx.beginPath();
                ctx.lineWidth = 3;
                ctx.strokeStyle = "#000000";
                ctx.moveTo(traj.data[0].relX, traj.data[0].relY);
                ctx.lineTo(traj.data[1].relX, traj.data[1].relY);
                ctx.stroke();
                ctx.closePath();
            }else if(traj.type == 'circle'){
                if(traj.data[4]) continue
                var c, r, theta_0, theta_1;
                c = traj.data[0];
                r = traj.data[1];
                theta_0 = traj.data[2];
                theta_1 = traj.data[3];
                var A, B, ccw;
                A = theta_0>theta_1;
                B = Math.abs(theta_1-theta_0) < Math.PI;
                ccw = (!A&&!B)||(A&&B); // xand 
                ctx.beginPath();
                ctx.lineWidth = 3;
                ctx.strokeStyle = "#000000";
                ctx.arc(c.relX, c.relY, r, theta_0, theta_1, ccw);

                ctx.stroke();
                ctx.closePath();
            }
        }
    }
}