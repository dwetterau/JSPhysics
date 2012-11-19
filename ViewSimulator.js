function sim_viewer() {

var WIDTH = 0;
var HEIGHT = 0;
var w2gw = 0;
var h2gh = 0;
var timestep = 15;
var PI2 = 2*Math.PI;
var sim;
var canvas;
var ctx;

function init() {
    canvas = document.getElementById('main_canvas');
    canvas.width = canvas.clientWidth;
    canvas.height = canvas.clientHeight;
    ctx = canvas.getContext("2d");
   
    WIDTH = Math.max(canvas.clientWidth, 800);
    HEIGHT = Math.max(canvas.clientHeight, 600);
    canvas.width = WIDTH;
    canvas.height = HEIGHT;
    w2gw = HEIGHT/100.0;
    h2gh = HEIGHT/100.0;
    sim = new Simulator(WIDTH/HEIGHT);
    addInitialBalls();
    initializeSliders(); 
    var intervalId = setInterval(draw, timestep);
    return intervalId;
}

function initializeSliders() {
    slider = $('#GravityToggle');
    slider[0].selectedIndex = 1;
    slider.slider('refresh');
    slider.on('slidestop', function(event) {
        sim.DOWN_GRAVITY = slider[0].selectedIndex == 0 ? false : true;
    });


}

function initializeSlider(name, callback) {


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
        drawPath(sim.paths[i], sim.balls[i].color);
    }
    for (var i = 0; i < sim.balls.length; i++) {
        drawBall(sim.balls[i]); 
    }
}

function drawPath(path, color) {
    ctx.beginPath();
    ctx.strokeStyle = color;
    
    for (var i = 0; i < path.length; i++) {
        var x = Math.round(path[i].x*w2gw);
        var y = Math.round(HEIGHT-(path[i].y*h2gh));
        ctx.lineTo(x, y);
    }
    
    ctx.lineWidth = 2;
    ctx.stroke();
    ctx.closePath();
}

 

function drawBall(b) {
    ctx.beginPath();
    ctx.fillStyle = b.color;
	var r_h = (Math.round(b.radius))*h2gh;
	var x = Math.round((b.pos.x*w2gw)-r_h);
	var y = Math.round((HEIGHT-(b.pos.y*h2gh))-r_h);
    r_h = Math.round(r_h);

    //This will be used for velocity line drawing later
    //var mag = b.velocity.getMag();
    ctx.arc(x+r_h, y+r_h, r_h,  0, PI2, false);
    ctx.fill();
    ctx.lineWidth = 2;
    ctx.strokeStyle = "#000";
    ctx.stroke();
    ctx.closePath();
}

init();
}
