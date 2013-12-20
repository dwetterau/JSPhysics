function Body(inverseMass, linearDamping, position, orientation, velocity, rotation) {
  this.inverseMass = inverseMass;
  this.linearDamping = linearDamping;
  this.position = position;
  this.orientation = orientation;
  this.velocity = velocity;
  this.rotation = rotation;

  // Set up derived values
  this.transformMatrix = new Matrix4(new Array(12));
  this.inverseInertiaTensor = new Matrix3(new Array(9));
  this.inverseInertiaTensorWorld = new Matrix3(new Array(9));
  this.calculateDerivedData();
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