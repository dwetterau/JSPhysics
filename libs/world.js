function World() {
  // Physics attributes
  this.bodyIds = [];
  this.bodies = {};
  this.bodyCounter = 0;
  this.forceGenerators = [];

  // Rendering attributes
  this.scene = new THREE.Scene();
}

World.prototype.addBody = function(body) {
  this.bodyCounter += 1;
  this.bodyIds.push(this.bodyCounter);
  this.bodies[this.bodyCounter] = body;
  return this.bodyCounter;
};

World.prototype.updateAllForces = function(dt) {
  for(var i = 0; i < this.forceGenerators.length; i++) {
    var bodiesToApply = this.forceGenerators[i].getBodyIds();
    for (var j = 0; j < bodiesToApply.length; j++) {
      this.forceGenerators[i].updateForce(this.bodies[bodiesToApply[j]], dt);
    }
  }
};

World.prototype.runPhysics = function(dt) {
  this.updateAllForces(dt);
  for (var i = 0; i < this.bodyIds.length; i++) {
    this.bodies[this.bodyIds[i]].integrate(dt);
  }
};

World.prototype.addCube = function(w, h, d, pos) {
  var cube = new THREE.Mesh(new THREE.CubeGeometry(w, h, d), new THREE.MeshLambertMaterial({
    color: 'blue'
  }));
  cube.matrixAutoUpdate = false;
  cube.overdraw = true;
  cube.dynamic = true;
  var body = (new BodyBuilder())
    .setPosition(pos)
    .setGeometry({
      type: "cube",
      dx: w,
      dy: h,
      dz: d
    })
    .build();

  // set up inertia
  var m = body.getMass();
  body.setInertiaTensor(new Matrix3([
    1/12.0 * m * (h * h + d * d), 0, 0,
    0, 1/12.0 * m * (w * w + d * d), 0,
    0, 0, 1/12.0 * m * (w * w + h * h)
  ]));
  body.setObject(cube);
  body.calculateDerivedData();
  this.addBody(body);
  this.scene.add(cube);
};

World.prototype.render = function() {
  renderer.render(this.scene, this.camera);
};

