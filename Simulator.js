function Simulator(w2hRatio) {
    this.GRAVITY = .4;
    this.MASS_OF_POINT = 100;
    this.RADIUS_OF_POINT = 1;
    this.RESTITUTION = .95;
    this.TOLERANCE = .01;
    this.COLLISIONS = true;
    this.DOWN_GRAVITY = true;
    this.WALL_COLLISIONS = true;
    this.LIMIT_PATHS = true;
    this.DRAW_PATHS = true;
    this.DRAW_HELP = true;
    this.DRAW_FPS = true;
    this.ACCEL_GRAV = false;

    this.delay = 5;
    this.fps = 0;

    this.balls = [];
    this.gravityPoints =[];
    this.paths = [];
    this.gravity_dir = new Vector(0,-1);
    this.GRID_WIDTH = Math.round(100*w2hRatio);
    this.GRID_HEIGHT = 100;
}

Simulator.prototype.changeGravity = function(y, z) {
    if (this.ACCEL_GRAV) {
        if (y < 0) {
            y = Math.max(-45, y); 
        } else {
            y = Math.min(45, y);
        }
        if (z < 0) {
            z = Math.max(-45, z);
        } else {
            z = Math.min(45, z);
        }

        this.gravity_dir = new Vector(z/45, -y/45);
        console.log(this.gravity_dir);
    }
}

Simulator.prototype.addBall = function(r, pos, theta, mag, mass, c) {
    this.balls.push(new Ball(r, pos, theta, mag, mass, c));
    var newPath = [];
    this.paths.push(newPath);
}

Simulator.prototype.update = function(timestep) {
    this.moveBalls(timestep);
    //make sure paths don't get too long
    if (this.LIMIT_PATHS) {
        for(var i = 0; i < this.paths.length; i++) {
            if (this.paths[i].length > 100) {
                this.paths[i].splice(0, this.paths[i].length-100);
            }
        }
    }
}

Simulator.prototype.moveBalls = function(timestep) {
    for(var i = 0; i < this.balls.length; i++) {
        b = this.balls[i];
        //TODO check for collisions with gravity points
        //Check for collisions with all the other balls. 
        for (var j = i+1; j < this.balls.length; j++) {
            if (this.COLLISIONS && this.areColliding(this.balls[i], this.balls[j])) {
                //Glancing collision code
                b1 = b;
                b2 = this.balls[j];
                
                i2j = new Vector(b2.pos.x - b1.pos.x, b2.pos.y - b1.pos.y);
                i2j.makeMag(b1.radius + b2.radius + this.TOLERANCE);
                b2.pos = new Point(b1.pos.x + i2j.a, b1.pos.y+i2j.b);

                m1 = b1.mass;
                m2 = b2.mass;

                v_cm = new Vector((b1.velocity.a*m1+b2.velocity.a*m2)/(m1+m2),
                                        (b1.velocity.b*m1+b2.velocity.b*m2)/(m1+m2));
                v_cm.reverse();
                v1 = b1.velocity;
                v2 = b2.velocity;
                
                v1.addVector(v_cm);
                v2.addVector(v_cm);

                i2j.normalize();
                temp = v1.proj(i2j);
                temp.reverse();
                v1.addVector(temp);
                v1.addVector(temp);

                i2j.reverse();

                temp = v2.proj(i2j);
                temp.reverse();

                v2.addVector(temp);
                v2.addVector(temp);

                v_cm.reverse();

                v1.addVector(v_cm);
                v2.addVector(v_cm);
            }
        }
    }
    for (var i = 0; i < this.balls.length; i++) {
        if (this.DRAW_PATHS) {
            this.paths[i].push(new Point(this.balls[i].pos.x, this.balls[i].pos.y));
        } else {
            this.paths[i] = [];
        }
        this.moveBall(this.balls[i], timestep);
    }
}

Simulator.prototype.moveBall = function(b, timestep) {
    var dt = timestep/1000.0;
	b.pos.x = (b.pos.x + b.velocity.a*dt);
	b.pos.y = (b.pos.y + b.velocity.b*dt);
	var none = true;
	var mass = b.mass;
	/*for(nPoint p : gravityPoints) {
		none = false;
		PVector grav = new PVector(p.x-b.pos.x,p.y-b.pos.y);
		double mag = grav.getMag();
		grav.makeMag(-(this.GRAVITY*mass*this.MASS_OF_POINT)/(mag*mag));
	    b.velocity.addVector(grav);	
	}  */ 
    if(none && this.DOWN_GRAVITY){
        var grav = this.gravity_dir.scale(this.GRAVITY);
        b.velocity.addVector(grav);
	}
	if(this.WALL_COLLISIONS) {
		this.checkWalls(b);
    }

}

Simulator.prototype.checkWalls = function(b) {
    var collided = false;
	if(b.pos.x-b.radius <= 0){
		collided = true;
		b.velocity.a = -b.velocity.a;
		b.pos.x = 2*b.radius-b.pos.x;
	}
	else if(b.pos.x+b.radius >= this.GRID_WIDTH){
		collided = true;
		b.velocity.a = -b.velocity.a;
		b.pos.x = (2*this.GRID_WIDTH-b.pos.x-2*b.radius);
	}
	if(b.pos.y-b.radius <= 0){
		collided = true;
		b.velocity.b = -b.velocity.b;
		b.pos.y = 2*b.radius-b.pos.y;
	}
	else if(b.pos.y+b.radius >= this.GRID_HEIGHT){
		collided = true;
		b.velocity.b = -b.velocity.b;
		b.pos.y = 2*this.GRID_HEIGHT-2*b.radius-b.pos.y;
	}
	if(collided){
		b.velocity.mult(this.RESTITUTION);
	}
}

Simulator.prototype.distance = function(b1, b2) {
    return Math.sqrt((b1.pos.x-b2.pos.x)*(b1.pos.x-b2.pos.x) + (b1.pos.y-b2.pos.y)*(b1.pos.y-b2.pos.y));
}

Simulator.prototype.areColliding = function(b1, b2) {
    d = this.distance(b1,b2);
    return d <= b1.radius + b2.radius;
}
