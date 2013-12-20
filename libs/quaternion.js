function Quaternion(r, i, j, k) {
  this.r = r;
  this.i = i;
  this.j = j;
  this.k = k;
}

Quaternion.prototype.normalize = function() {
  var d = this.r * this.r + this.i * this.i + this.j * this.j + this.k * this.k;
  if (d == 0) {
    this.r = 1;
    return;
  }
  d = 1 / Math.sqrt(d);
  this.r *= d;
  this.i *= d;
  this.j *= d;
  this.k *= d;
};

Quaternion.prototype.multiply = function(m) {
  var r = this.r * m.r - this.i * m.i - this.j * m.j - this.k * m.k;
  var i = this.r * m.i + this.i * m.r + this.j * m.k - this.k * m.j;
  var j = this.r * m.j + this.j * m.r + this.k * m.i - this.i * m.k;
  var k = this.r * m.k + this.k * m.r + this.i * m.j - this.j * m.i;
  this.r = r;
  this.i = i;
  this.j = j;
  this.k = k;
};

Quaternion.prototype.rotateByVector = function(v) {
  var q = new Quaternion(0, v.x, v.y, v.z);
  this.multiply(q);
};

Quaternion.prototype.addScaledVector = function(v, scale) {
  var q = new Quaternion(0, v.x * scale, v.y * scale, v.z * scale);
  q.multiply(this);
  this.r += q.r * .5;
  this.i += q.i * .5;
  this.j += q.j * .5;
  this.k += q.k * .5;
};
