class Manipulator{
    constructor(q, settings){
        this.q_coords = q;
        this.settings = settings;
        [this.p, this.end_eff] = this.dk(q);
        this.traces = {'x1':[],'x2':[]};
    }

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

    draw_traces(ctx, colors = ['#0000FF','#00FF00']){
        ctx.beginPath();
        ctx.lineWidth = 5;
        ctx.strokeStyle = colors[0];
        if(this.traces['x1'].length < 1 || this.traces['x2'].length < 1) return ;
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
    }

    dk(q){
        settings = this.settings;
        var p1 = [settings['l1'] * Math.cos(q[0]), settings['l1'] * Math.sin(q[0])]
        var p2 = [p1[0] + settings['l2'] * Math.cos(q[0] + q[1]), p1[1] + settings['l2'] * Math.sin(q[0] + q[1])];
        var p1rel = abs2rel(p1[0], p1[1], settings); // intermediate point position
        var p2rel = abs2rel(p2[0], p2[1], settings); // end effector position
        return [p1rel, p2rel];
    }
}

class Point{
    constructor(x, y, settings){
        this.settings = settings;
        this.relative = {x, y};
        var rx, ry;
        [rx, ry] = rel2abs(x, y, settings);
        this.actual = {'x':rx, 'y': ry};
    }

    set relX(x){
        this.relative.x = x;
        var rx, ry;
        [rx, ry] = rel2abs(this.relX, this.relY, this.settings);
        this.actual = {'x':rx, 'y': ry};
    }

    get relX(){
        return this.relative.x;
    }

    set relY(y){
        this.relative.y = y;
        var rx, ry;
        [rx, ry] = rel2abs(this.relX, this.relY, this.settings);
        this.actual = {'x':rx, 'y': ry};
    }

    get actX(){
        return this.actual.x;
    }

    get actY(){
        return this.actual.y;
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

    get relY(){
        return this.relative.y;
    }

    add(other){
        var result = new Point(this.relX, this.relY, this.settings);
        result.relX = result.relX+other.relX;
        result.relY = result.relY+other.relY;
        return result;
    }

    sub(other){
        var result = new Point(this.relX, this.relY, this.settings);
        result.relX = result.relX-other.relX;
        result.relY = result.relY-other.relY;
        return result;
    }

    mag(){
        return Math.sqrt(this.relX*this.relX+this.relY*this.relY);
    }

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

    rot(delta){
        var result = new Point(0, 0, this.settings);
        var rho = this.mag();
        var theta = Math.atan2(this.relY, this.relX) + delta;
        result.relX = rho*Math.cos(theta);
        result.relY = rho*Math.sin(theta);
        return result;
    }

    set(scalar){
        var result = new Point(0, 0, this.settings);
        var rho = scalar;
        var theta = Math.atan2(this.relY, this.relX);
        result.relX = rho*Math.cos(theta);
        result.relY = rho*Math.sin(theta);
        return result;
    }

    angle(){
        return Math.atan2(this.relY, this.relX);
    }
}

class Trajectory{
    constructor(){
        this.data = [];
    }

    add_line(p0, p1, raised){
        this.data.push({'type':'line', 'data':[p0, p1, raised]}) // start and end point
    }

    add_circle(c, r, theta_0, theta_1, raised){
        this.data.push({'type':'circle', 'data': [c, r, theta_0, theta_1, raised]}) // point and diameter
    }

    reset(){
        this.data = [];
    }

    draw(ctx){
        for(var traj of this.data){
            if(traj.type == 'line'){
                ctx.beginPath();
                ctx.lineWidth = 3;
                ctx.strokeStyle = "#000000";
                ctx.moveTo(traj.data[0].relX, traj.data[0].relY);
                ctx.lineTo(traj.data[1].relX, traj.data[1].relY);
                ctx.stroke();
                ctx.closePath();
            }else if(traj.type == 'circle'){
                var c, r, theta_0, theta_1;
                c = traj.data[0];
                r = traj.data[1];
                theta_0 = traj.data[2];
                theta_1 = traj.data[3];
                ctx.beginPath();
                ctx.lineWidth = 3;
                ctx.strokeStyle = "#000000";
                var cw = Math.abs(theta_1-theta_0) < Math.PI && (theta_1 > theta_0); // clockwise
                ctx.arc(c.relX, c.relY, r, theta_0, theta_1, !cw)

                ctx.stroke();
                ctx.closePath();
            }
        }
    }
}