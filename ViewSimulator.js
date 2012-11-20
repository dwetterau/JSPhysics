function sim_viewer() {

var WIDTH = 0;
var HEIGHT = 0;
var w2gw = 0;
var h2gh = 0;
var timestep = 5;
var PI2 = 2*Math.PI;
var sim;
var canvas;
var ctx;
var mobile = false;

function init() {
    canvas = $('#main_canvas')[0];
    var body = $('#body')[0];
    
    canvas.width = Math.min(800, Math.round(body.clientWidth * .6));
    canvas.height = Math.min(600, Math.round(body.clientHeight * .75));
    ctx = canvas.getContext("2d");
  
    WIDTH = canvas.width;
    HEIGHT = canvas.height;
    canvas.style.width = WIDTH+'px';
    canvas.style.height = HEIGHT+'px';
    w2gw = HEIGHT/100.0;
    h2gh = HEIGHT/100.0;
    sim = new Simulator(WIDTH/HEIGHT);
    addInitialBalls();
    initializeSliders(); 
    initializeAccelerometer();
    var intervalId = setInterval(draw, timestep);
}

function initializeSliders() {
    var slider = $('select#GravityToggle');
    slider.val('on').slider('refresh');
    slider.change(function(event) {
       sim.DOWN_GRAVITY = slider.val() == 'on' ?  true : false; //!sim.DOWN_GRAVITY;
    });

    var slider2 = $('select#PathToggle');
    slider2.val('on').slider('refresh');
    slider2.change(function(event) {
        sim.DRAW_PATHS = slider2.val() == 'on' ? true : false;
    });

    var slider3 = $('select#LimitToggle');
    slider3.val('on').slider('refresh');
    slider3.change(function(event) {
        sim.LIMIT_PATHS = slider3.val() == 'on' ? true : false;
    });
}

function initializeAccelerometer() {
    $(window).bind("deviceorientation", function(e) {
        var orig_event = e.originalEvent;
        if (orig_event.beta == null) {
            mobile = false;
        } else {
            mobile = true;
            sim.ACCEL_GRAV = true;
            sim.changeGravity(orig_event.beta, orig_event.gamma);
        }
    });
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
    //var start = -(new Date().getTime());
    clear();
    //console.log('time to clear: '+((new Date().getTime())+start));
    //start = -(new Date().getTime());
    sim.update(timestep); 
    //console.log('time to update: '+((new Date().getTime())+start));
    //start = -(new Date().getTime());
    drawBalls();
    //console.log('drawing ball time: '+((new Date().getTime())+start));
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

function drawGrav() {
    $('#grav_test').html(
        'a: '+sim.gravity_dir.a +'<br>'+
        'b: '+sim.gravity_dir.b +'<br>'
    );
}

init();
}
