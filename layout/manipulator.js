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

    /*get q(){
        return this.q_coords;
    }

    get eef(){
        return this.end_eff;//{'x': this.end_eff[0], 'y': this.end_eff[1]};
    }*/

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