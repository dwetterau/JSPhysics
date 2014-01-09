function World() {
  // Physics attributes
  this.bodyIds = [];
  this.bodies = {};
  this.bodyCounter = 0;
  this.forceGenerators = {};

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
  for(var key in this.forceGenerators) {
    if (!this.forceGenerators.hasOwnProperty(key)) {
      continue;
    }
    var bodiesToApply = this.forceGenerators[key].getBodyIds();
    for (var bodyId in bodiesToApply) {
      if (!bodiesToApply.hasOwnProperty(bodyId)) {
        continue;
      }
      this.forceGenerators[key].updateForce(this.bodies[bodyId], dt);
    }
  }
};

/**
 * The main method for the physics engine
 * @param dt The amount of time to integrate over in seconds
 */
World.prototype.runPhysics = function(dt) {
  this.updateAllForces(dt);
  var i, j;
  for (i = 0; i < this.bodyIds.length; i++) {
    this.bodies[this.bodyIds[i]].integrate(dt);
  }
  // Check for collisions
  if (this.collisionDetector) {
    var collisionsToResolve = [];

    for (i = 0; i < this.bodyIds.length; i++) {
      for (j = i + 1; j < this.bodyIds.length; j++) {
        var collisionList = this.collisionDetector.getCollisions(
          this.bodies[this.bodyIds[i]],
          this.bodies[this.bodyIds[j]]);
        // TODO: Actually process the collisions
        if (collisionList.length > 0) {
          for (var k = 0; k < collisionList.length; k++) {
            collisionList[k].setBodyIds(this.bodyIds[i], this.bodyIds[j]);
          }
          collisionsToResolve.push.apply(collisionsToResolve, collisionList);
        }
      }
    }
    // resolve the collisions

  }
};

World.prototype.addCollisionDetector = function(threshold) {
  this.collisionDetector = new CollisionDetector(threshold);
};

World.prototype.addGravity = function(g) {
  this.forceGenerators.gravity = new Gravity(g);
};

World.prototype.addGravityToBody = function(bodyId) {
  if (!this.forceGenerators.gravity) {
    throw "Gravity force doesn't exist!";
  }
  this.forceGenerators.gravity.addBodyId(bodyId);
};

World.prototype.addBox = function(w, h, d, body) {
  var cube = new THREE.Mesh(new THREE.CubeGeometry(w, h, d), new THREE.MeshLambertMaterial({
    color: 'blue'
  }));
  cube.matrixAutoUpdate = false;
  cube.overdraw = true;
  cube.dynamic = true;
  body.setGeometry({
    type: "box",
    dx: w,
    dy: h,
    dz: d
  });
  // set up inertia
  var m = body.getMass();
  body.setInertiaTensor(new Matrix3([
    1/12.0 * m * (h * h + d * d), 0, 0,
    0, 1/12.0 * m * (w * w + d * d), 0,
    0, 0, 1/12.0 * m * (w * w + h * h)
  ]));
  body.setObject(cube);
  body.calculateDerivedData();
  this.scene.add(cube);
  return this.addBody(body);
};

World.prototype.addSphere = function(r, body) {
  var sphere = new THREE.Mesh(new THREE.SphereGeometry(r, 100, 100), new THREE.MeshLambertMaterial({
    color: 'red'
  }));
  sphere.matrixAutoUpdate = false;
  sphere.overdraw = true;
  sphere.dynamic = true;
  body.setGeometry({
    type: "sphere",
    r: r
  });
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
  this.scene.add(sphere);
  return this.addBody(body);
};

World.prototype.addPlane = function(w, h, normal, offset, isHalfSpace, body) {
  var plane = new THREE.Mesh(new THREE.PlaneGeometry(w, h), new THREE.MeshNormalMaterial());
  plane.matrixAutoUpdate = false;
  plane.overdraw = true;
  plane.dynamic = true;

  // Set the normal correctly
  plane.lookAt(new THREE.Vector3(normal.x, normal.y, normal.z));
  var q = new THREE.Quaternion();
  q.setFromEuler(plane.rotation);

  var pos = normal.scale(offset);

  body.setPosition(pos);
  body.setGeometry({
      type: "plane",
      normal: normal,
      offset: offset,
      isHalfSpace: isHalfSpace
  });
  body.setOrientation(new Quaternion(q.w, q.x, q.y, q.z));

  // set up inertia
  var m = body.getMass();
  body.setInertiaTensor(new Matrix3([
    1/12.0 * m * (h * h), 0, 0,
    0, 1/12.0 * m * (w * w), 0,
    0, 0, 1/12.0 * m * (w * w + h * h)
  ]));
  body.setObject(plane);
  body.calculateDerivedData();
  this.scene.add(plane);
  return this.addBody(body);
};

World.prototype.render = function() {
  renderer.render(this.scene, this.camera);
};

