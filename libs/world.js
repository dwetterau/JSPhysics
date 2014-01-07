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

World.prototype.addBox = function(w, h, d, pos) {
  var cube = new THREE.Mesh(new THREE.CubeGeometry(w, h, d), new THREE.MeshLambertMaterial({
    color: 'blue'
  }));
  cube.matrixAutoUpdate = false;
  cube.overdraw = true;
  cube.dynamic = true;
  var body = (new BodyBuilder())
    .setPosition(pos)
    .setGeometry({
      type: "box",
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

World.prototype.addSphere = function(r, pos) {
  var sphere = new THREE.Mesh(new THREE.SphereGeometry(r, 100, 100), new THREE.MeshLambertMaterial({
    color: 'red'
  }));
  sphere.matrixAutoUpdate = false;
  sphere.overdraw = true;
  sphere.dynamic = true;
  var body = (new BodyBuilder())
    .setPosition(pos)
    .setGeometry({
      type: "sphere",
      r: r
    })
    .build();
  // set up inertia
  var m = body.getMass();
  var mr2 = 2/5.0 * m * r * r;
  body.setInertiaTensor(new Matrix3([
    mr2, 0, 0,
    0, mr2, 0,
    0, 0, mr2
  ]));
  body.setObject(sphere);
  body.calculateDerivedData();
  this.addBody(body);
  this.scene.add(sphere);
};

World.prototype.addPlane = function(w, h, normal, offset, isHalfSpace) {
  var plane = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshNormalMaterial());
  plane.matrixAutoUpdate = false;
  plane.overdraw = true;
  plane.dynamic = true;

  // Set the normal correctly
  plane.lookAt(new THREE.Vector3(normal.x, normal.y, normal.z));
  var q = new THREE.Quaternion();
  q.setFromEuler(plane.rotation);

  var pos = normal.scale(offset);

  var body = (new BodyBuilder())
    .setPosition(pos)
    .setGeometry({
      type: "plane",
      normal: normal,
      offset: offset,
      isHalfSpace: isHalfSpace
    })
    .setOrientation(new Quaternion(q.w, q.x, q.y, q.z))
    .build();

  // set up inertia
  var m = body.getMass();
  body.setInertiaTensor(new Matrix3([
    1/12.0 * m * (h * h), 0, 0,
    0, 1/12.0 * m * (w * w), 0,
    0, 0, 1/12.0 * m * (w * w + h * h)
  ]));
  body.setObject(plane);
  body.calculateDerivedData();
  this.addBody(body);
  this.scene.add(plane);
};

World.prototype.render = function() {
  renderer.render(this.scene, this.camera);
};

