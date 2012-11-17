var WIDTH = 0;
var HEIGHT = 0;
var w2gw = 0;
var h2gh = 0;
var timestep = 5;

function init() {

    var canvas = document.getElementById('canvas');
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    ctx = canvas.getContext("2d");
    
    WIDTH = canvas.clientWidth;
    HEIGHT = canvas.clientHeight;
    w2gw = HEIGHT/100.0;
    h2gh = HEIGHT/100.0;
    PI2 = 2*Math.PI;
    console.log(WIDTH);
    console.log(HEIGHT);
    sim = new Simulator(WIDTH/HEIGHT);
    addInitialBalls();
    
    var intervalId = setInterval(draw, timestep);
    return intervalId;
}

function addInitialBalls() {
    sim.addBall(5,new Point(10,50),PI2*Math.random(),20,5,'#0000FF');
	sim.addBall(5,new Point(30,50),PI2*Math.random(),20,5,'#FF0000');
	sim.addBall(5,new Point(50,25),PI2*Math.random(),20,5,'#00FF00');
	sim.addBall(5,new Point(70,25),PI2*Math.random(),20,5,'#FFFF00');
	sim.addBall(5,new Point(90,25),PI2*Math.random(),20,5,'#FF00FF');
    sim.addBall(5,new Point(10,75),PI2*Math.random(),20,5,'#000000');
}

function clear() {
    ctx.clearRect(0,0,WIDTH, HEIGHT);
}

function draw() {
    clear();
    sim.update(timestep);
    drawBalls();
}

function drawBalls() {
    //Add path drawing next
    for (var i = 0; i < sim.balls.length; i++) {
        drawBall(sim.balls[i]);        
    }
}

 

function drawBall(b) {
    ctx.beginPath();
    ctx.fillStyle = b.color;
	var r_h = (Math.round(b.radius))*h2gh;
	var x = Math.round((b.pos.x*w2gw)-r_h);
	var y = Math.round((HEIGHT-(b.pos.y*h2gh))-r_h);
	
    //This will be used for velocity line drawing later
    //var mag = b.velocity.getMag();
    ctx.arc(x+r_h, y+r_h, r_h,  0, PI2, false);
    ctx.fill();
    ctx.lineWidth = 1;
    ctx.strokeStyle = "#000";
    ctx.stroke();
    ctx.closePath();

/*ctx.beginPath();
 ctx.arc(centerX, centerY, radius, 0, 2 * Math.PI, false);
 ctx.fillStyle = 'green';
 ctx.fill();
 ctx.lineWidth = 5;
 ctx.strokeStyle = '#003300';
 ctx.stroke();*/  
}
