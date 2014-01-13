function BodyBuilder() {
  this.inverseMass = 1;
  this.linearDamping = .999999;
  this.angularDamping = .999999;
  this.position = new Vector3(0, 0, 0);
  this.orientation = new Quaternion(1, 0, 0, 0);
  this.velocity = new Vector3(0, 0, 0);
  this.rotation = new Vector3(0, 0, 0);
  this.geometry = {type: 'UNKNOWN'};
}

BodyBuilder.prototype.setInverseMass = function(inverseMass) {
  this.inverseMass = inverseMass;
  return this;
};

BodyBuilder.prototype.setLinearDamping = function(linearDamping) {
  this.linearDamping = linearDamping;
  return this;
};

BodyBuilder.prototype.setAngularDamping = function(angularDamping) {
  this.angularDamping = angularDamping;
  return this;
};

BodyBuilder.prototype.setPosition = function(position) {
  this.position = position;
  return this;
};

BodyBuilder.prototype.setOrientation = function(orientation) {
  this.orientation = orientation;
  return this;
};

BodyBuilder.prototype.setVelocity = function(velocity) {
  this.velocity = velocity;
  return this;
};

BodyBuilder.prototype.setRotation = function(rotation) {
  this.rotation = rotation;
  return this;
};

BodyBuilder.prototype.setGeometry = function(geometry) {
  this.geometry = geometry;
  return this;
};

BodyBuilder.prototype.build = function() {
  return new Body(
    this.inverseMass,
    this.linearDamping,
    this.angularDamping,
    this.position,
    this.orientation,
    this.velocity,
    this.rotation,
    this.geometry
  );
};

function Body(inverseMass, linearDamping, angularDamping, position, orientation, velocity, rotation, geometry) {
  this.inverseMass = inverseMass;
  this.linearDamping = linearDamping;
  this.angularDamping = angularDamping;
  this.position = position;
  this.orientation = orientation;
  this.velocity = velocity;
  this.rotation = rotation;
  this.geometry = geometry;
  this.forceAccumulator = new Vector3(0, 0, 0);
  this.torqueAccumulator = new Vector3(0, 0, 0);

  // Set up derived values
  this.transformMatrix = new Matrix4(new Array(12));
  this.inverseInertiaTensor = new Matrix3(new Array(9));
  this.inverseInertiaTensorWorld = new Matrix3(new Array(9));
}

Body.prototype.setInertiaTensor = function(mat) {
  this.inverseInertiaTensor.setInverse(mat);
};

Body.prototype.calculateDerivedData = function() {
  this.orientation.normalize();

  this.calculateTransformMatrix();
  this.transformInertiaTensor();
};

Body.prototype.calculateTransformMatrix = function() {
  this.transformMatrix.data[0] = 1 - 2 * this.orientation.j * this.orientation.j - 2 * this.orientation.k * this.orientation.k;
  this.transformMatrix.data[1] = 2 * this.orientation.i * this.orientation.j - 2 * this.orientation.r * this.orientation.k;
  this.transformMatrix.data[2] = 2 * this.orientation.i * this.orientation.k + 2 * this.orientation.r * this.orientation.j;
  this.transformMatrix.data[3] = this.position.x;

  this.transformMatrix.data[4] = 2 * this.orientation.i * this.orientation.j + 2 * this.orientation.r * this.orientation.k;
  this.transformMatrix.data[5] = 1 - 2 * this.orientation.i * this.orientation.i - 2 * this.orientation.k * this.orientation.k;
  this.transformMatrix.data[6] = 2 * this.orientation.j * this.orientation.k - 2 * this.orientation.r * this.orientation.i;
  this.transformMatrix.data[7] = this.position.y;

  this.transformMatrix.data[8] = 2 * this.orientation.i * this.orientation.k - 2 * this.orientation.r * this.orientation.j;
  this.transformMatrix.data[9] = 2 * this.orientation.j * this.orientation.k + 2 * this.orientation.r * this.orientation.i;
  this.transformMatrix.data[10] = 1 - 2 * this.orientation.i * this.orientation.i - 2 * this.orientation.j * this.orientation.j;
  this.transformMatrix.data[11] = this.position.z;
};

Body.prototype.transformInertiaTensor = function() {
  if (!this.hasFiniteMass()) {
    return;
  }
  var t4 = this.transformMatrix.data[0] * this.inverseInertiaTensor.data[0] +
      this.transformMatrix.data[1] * this.inverseInertiaTensor.data[3] +
      this.transformMatrix.data[2] * this.inverseInertiaTensor.data[6];
  var t9 = this.transformMatrix.data[0] * this.inverseInertiaTensor.data[1] +
    this.transformMatrix.data[1] * this.inverseInertiaTensor.data[4] +
    this.transformMatrix.data[2] * this.inverseInertiaTensor.data[7];
  var t14 = this.transformMatrix.data[0] * this.inverseInertiaTensor.data[2] +
    this.transformMatrix.data[1] * this.inverseInertiaTensor.data[5] +
    this.transformMatrix.data[2] * this.inverseInertiaTensor.data[8];

  var t28 = this.transformMatrix.data[4] * this.inverseInertiaTensor.data[0] +
    this.transformMatrix.data[5] * this.inverseInertiaTensor.data[3] +
    this.transformMatrix.data[6] * this.inverseInertiaTensor.data[6];
  var t33 = this.transformMatrix.data[4] * this.inverseInertiaTensor.data[1] +
    this.transformMatrix.data[5] * this.inverseInertiaTensor.data[4] +
    this.transformMatrix.data[6] * this.inverseInertiaTensor.data[7];
  var t38 = this.transformMatrix.data[4] * this.inverseInertiaTensor.data[2] +
    this.transformMatrix.data[5] * this.inverseInertiaTensor.data[5] +
    this.transformMatrix.data[6] * this.inverseInertiaTensor.data[8];

  var t52 = this.transformMatrix.data[8] * this.inverseInertiaTensor.data[0] +
    this.transformMatrix.data[9] * this.inverseInertiaTensor.data[3] +
    this.transformMatrix.data[10] * this.inverseInertiaTensor.data[6];
  var t57 = this.transformMatrix.data[8] * this.inverseInertiaTensor.data[1] +
    this.transformMatrix.data[9] * this.inverseInertiaTensor.data[4] +
    this.transformMatrix.data[10] * this.inverseInertiaTensor.data[7];
  var t62 = this.transformMatrix.data[8] * this.inverseInertiaTensor.data[2] +
    this.transformMatrix.data[9] * this.inverseInertiaTensor.data[5] +
    this.transformMatrix.data[10] * this.inverseInertiaTensor.data[8];

  this.inverseInertiaTensorWorld.data[0] = t4 * this.transformMatrix.data[0] +
      t9 * this.transformMatrix.data[1] +
      t14 * this.transformMatrix.data[2];
  this.inverseInertiaTensorWorld.data[1] = t4 * this.transformMatrix.data[4] +
    t9 * this.transformMatrix.data[5] +
    t14 * this.transformMatrix.data[6];
  this.inverseInertiaTensorWorld.data[2] = t4 * this.transformMatrix.data[8] +
    t9 * this.transformMatrix.data[9] +
    t14 * this.transformMatrix.data[10];

  this.inverseInertiaTensorWorld.data[3] = t28 * this.transformMatrix.data[0] +
    t33 * this.transformMatrix.data[1] +
    t38 * this.transformMatrix.data[2];
  this.inverseInertiaTensorWorld.data[4] = t28 * this.transformMatrix.data[4] +
    t33 * this.transformMatrix.data[5] +
    t38 * this.transformMatrix.data[6];
  this.inverseInertiaTensorWorld.data[5] = t28 * this.transformMatrix.data[8] +
    t33 * this.transformMatrix.data[9] +
    t38 * this.transformMatrix.data[10];

  this.inverseInertiaTensorWorld.data[6] = t52 * this.transformMatrix.data[0] +
    t57 * this.transformMatrix.data[1] +
    t62 * this.transformMatrix.data[2];
  this.inverseInertiaTensorWorld.data[7] = t52 * this.transformMatrix.data[4] +
    t57 * this.transformMatrix.data[5] +
    t62 * this.transformMatrix.data[6];
  this.inverseInertiaTensorWorld.data[8] = t52 * this.transformMatrix.data[8] +
    t57 * this.transformMatrix.data[9] +
    t62 * this.transformMatrix.data[10];
};

/**
 * Converts a point from local to world space
 * @param p the point the convert
 */
Body.prototype.getPointInWorldSpace = function(p) {
  return this.transformMatrix.multiplyVector(p);
};

/**
 * Converts a point in world space to local space
 * @param p the point in world space
 */
Body.prototype.getPointInBodySpace = function(p) {
  var inverseTransform = new Matrix4(new Array(12));
  inverseTransform.setInverse(this.transformMatrix);
  return inverseTransform.multiplyVector(p);
};

/**
 * Add a force to the body through the center of mass
 * @param v
 */
Body.prototype.addForce = function(v) {
  this.forceAccumulator.addInPlace(v);
};

/**
 * Add a force to the body through a point in body coordinate space
 * @param v vector in world space
 * @param p point in body space
 */
Body.prototype.addForceAtBodyPoint = function(v, p) {
  var pointInWorld = this.getPointInWorldSpace(p);
  this.addForceAtPoint(v, pointInWorld);
};

/**
 * Add a force at a point in world space on the body
 * @param v vector in world space
 * @param p point in world space
 */
Body.prototype.addForceAtPoint = function(v, p) {
  var pt = new Vector3(p.x, p.y, p.z);
  pt.subInPlace(this.position);

  this.forceAccumulator.add(v);
  this.torqueAccumulator.add(pt.cross(v));
};

/**
 * Zeros out the torques and forces after each integration step
 */
Body.prototype.clearAccumulators = function() {
  this.forceAccumulator.clear();
  this.torqueAccumulator.clear();
};

/**
 * Function that moves the body according to the forces accumulated
 * @param dt the change in time to integrate over
 */
Body.prototype.integrate = function(dt) {

  var acceleration = new Vector3(0, 0, 0);
  acceleration.addScaledVector(this.forceAccumulator, this.inverseMass);

  this.velocity.scaleInPlace(Math.pow(this.linearDamping, dt));
  this.velocity.addScaledVector(acceleration, dt);
  this.position.addScaledVector(this.velocity, dt);

  if (this.hasFiniteMass()) {
    var angularAcceleration = this.inverseInertiaTensorWorld.multiplyVector(this.torqueAccumulator);
    this.rotation.scaleInPlace(Math.pow(this.angularDamping, dt));
    this.rotation.addScaledVector(angularAcceleration, dt);
    this.orientation.addScaledVector(this.rotation, dt);
  }

  this.calculateDerivedData();
  this.updateSceneObject();
  this.clearAccumulators();
};

Body.prototype.hasFiniteMass = function() {
  return this.inverseMass > 0;
};

Body.prototype.updateSceneObject = function() {
  this.sceneObject.updateMatrix();
  this.sceneObject.matrix.copy(new THREE.Matrix4(
    this.transformMatrix.data[0], this.transformMatrix.data[1], this.transformMatrix.data[2], this.transformMatrix.data[3],
    this.transformMatrix.data[4], this.transformMatrix.data[5], this.transformMatrix.data[6], this.transformMatrix.data[7],
    this.transformMatrix.data[8], this.transformMatrix.data[9], this.transformMatrix.data[10], this.transformMatrix.data[11],
    0, 0, 0, 1));
};

Body.prototype.addVelocity = function(deltaVelocity) {
  this.velocity.addInPlace(deltaVelocity);
};

Body.prototype.addRotation = function(deltaRotation) {
  this.rotation.addInPlace(deltaRotation);
};

// Needed Setters
Body.prototype.setGeometry = function(geometry) {
  this.geometry = geometry;
};

Body.prototype.setObject = function(sceneObject) {
  this.sceneObject = sceneObject;
};

Body.prototype.setOrientation = function(orientation) {
  this.orientation = orientation;
};

Body.prototype.setPosition = function(position) {
  this.position = position;
};

// Needed Getters
Body.prototype.getGeometry = function() {
  return this.geometry;
};

Body.prototype.getInverseMass = function() {
  return this.inverseMass;
};

Body.prototype.getMass = function() {
  return 1 / this.inverseMass;
};

Body.prototype.getPosition = function() {
   return this.position;
};

Body.prototype.getRotation = function() {
  return this.rotation;
};

Body.prototype.getVelocity = function() {
  return this.velocity;
};

// Body type checks
Body.prototype.isBox = function() {
  return this.geometry.type == 'box';
};

Body.prototype.isSphere = function() {
  return this.geometry.type == 'sphere';
};

Body.prototype.isPlane = function() {
  return this.geometry.type == 'plane';
};
