function Vector3(x, y, z) {
  this.x = x;
  this.y = y;
  this.z = z;
}

Vector3.prototype.invert = function() {
  this.x = -this.x;
  this.y = -this.y;
  this.z = -this.z;
};

Vector3.prototype.magnitudeSq = function() {
  return this.x * this.x + this.y * this.y + this.z * this.z;
};

Vector3.prototype.magnitude = function() {
  return Math.sqrt(this.magnitudeSq());
};

Vector3.prototype.normalize = function() {
  var l = this.magnitude();
  if (l > 0) {
    this.scale(1 / l);
  }
};

Vector3.prototype.scale = function(scale) {
  return new Vector3(this.x * scale, this.y * scale, this.z * scale);
};

Vector3.prototype.scaleInPlace = function(scale) {
  this.x *= scale;
  this.y *= scale;
  this.z *= scale;
};

Vector3.prototype.add = function(v) {
  return new Vector3(this.x + v.x, this.y + v.y, this.z + v.z);
};

Vector3.prototype.addInPlace = function(v) {
  this.x += v.x;
  this.y += v.y;
  this.z += v.z;
};

Vector3.prototype.sub = function(v) {
  return new Vector3(this.x - v.x, this.y - v.y, this.z - v.z);
};

Vector3.prototype.subInPlace = function(v) {
  this.x -= v.x;
  this.y -= v.y;
  this.z -= v.z;
};

Vector3.prototype.addScaledVector = function(v, scale) {
  this.x += v.x * scale;
  this.y += v.y * scale;
  this.z += v.z * scale;
};

Vector3.prototype.componentProduct = function(v) {
  return new Vector3(this.x * v.x, this.y * v.y, this.z * v.z);
};

Vector3.prototype.componentProductUpdate = function(v) {
  this.x *= v.x;
  this.y *= v.y;
  this.z *= v.z;
};

Vector3.prototype.dot = function(v) {
  return this.x * v.x + this.y * v.y + this.z * v.z;
};

Vector3.prototype.cross = function(v) {
  return new Vector3(
      this.y * v.z - this.z * v.y,
      this.z * v.x - this.x * v.z,
      this.x * v.y - this.y * v.x);
};
