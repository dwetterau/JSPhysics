function Matrix3(data) {
  // Copy input matrix data array (3 x 4)
  this.data = data.slice(0);
}

Matrix3.prototype.makeIdentity = function() {
  this.data[0] = 1;
  this.data[1] = 0;
  this.data[2] = 0;
  this.data[3] = 0;
  this.data[4] = 1;
  this.data[5] = 0;
  this.data[6] = 0;
  this.data[7] = 0;
  this.data[8] = 1;
};

Matrix3.prototype.multiplyVector = function(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2],
      v.x * this.data[3] + v.y * this.data[4] + v.z * this.data[5],
      v.x * this.data[6] + v.y * this.data[7] + v.z * this.data[8]);
};

Matrix3.prototype.multiplyVectorTranspose = function(v) {
  return new Vector3(
    v.x * this.data[0] + v.y * this.data[3] + v.z * this.data[6],
    v.x * this.data[1] + v.y * this.data[4] + v.z * this.data[7],
    v.x * this.data[2] + v.y * this.data[5] + v.z * this.data[8]);
};

Matrix3.prototype.multiplyVectorInverse = function(v) {
  var tmp = new Vector3(v.x, v.y, v.z);
  tmp.x -= this.data[3];
  tmp.y -= this.data[7];
  tmp.z -= this.data[11];
  return new Vector3(
      tmp.x * this.data[0] + tmp.y * this.data[4] + tmp.z * this.data[8],
      tmp.x * this.data[1] + tmp.y * this.data[5] + tmp.z * this.data[9],
      tmp.x * this.data[2] + tmp.y * this.data[6] + tmp.z * this.data[10]);
};

Matrix3.prototype.multiplyMatrix = function(mat) {
  return new Matrix3([
      this.data[0] * mat.data[0] + this.data[1] * mat.data[3] + this.data[2] * mat.data[6],
      this.data[0] * mat.data[1] + this.data[1] * mat.data[4] + this.data[2] * mat.data[7],
      this.data[0] * mat.data[2] + this.data[1] * mat.data[5] + this.data[2] * mat.data[8],
      
      this.data[3] * mat.data[0] + this.data[4] * mat.data[3] + this.data[5] * mat.data[6],
      this.data[3] * mat.data[1] + this.data[4] * mat.data[4] + this.data[5] * mat.data[7],
      this.data[3] * mat.data[2] + this.data[4] * mat.data[5] + this.data[5] * mat.data[8],
      
      this.data[6] * mat.data[0] + this.data[7] * mat.data[3] + this.data[8] * mat.data[6],
      this.data[6] * mat.data[1] + this.data[7] * mat.data[4] + this.data[8] * mat.data[7],
      this.data[6] * mat.data[2] + this.data[7] * mat.data[5] + this.data[8] * mat.data[8]]);
};

Matrix3.prototype.multiplyMatrixInPlace = function(mat) {
  var t1 = this.data[0] * mat.data[0] + this.data[1] * mat.data[3] + this.data[2] * mat.data[6];
  var t2 = this.data[0] * mat.data[1] + this.data[1] * mat.data[4] + this.data[2] * mat.data[7];
  var t3 = this.data[0] * mat.data[2] + this.data[1] * mat.data[5] + this.data[2] * mat.data[8];
  this.data[0] = t1;
  this.data[1] = t2;
  this.data[2] = t3;
  
  t1 = this.data[3] * mat.data[0] + this.data[4] * mat.data[3] + this.data[5] * mat.data[6];
  t2 = this.data[3] * mat.data[1] + this.data[4] * mat.data[4] + this.data[5] * mat.data[7];
  t3 = this.data[3] * mat.data[2] + this.data[4] * mat.data[5] + this.data[5] * mat.data[8];
  this.data[3] = t1;
  this.data[4] = t2;
  this.data[5] = t3;

  t1 = this.data[6] * mat.data[0] + this.data[7] * mat.data[3] + this.data[8] * mat.data[6];
  t2 = this.data[6] * mat.data[1] + this.data[7] * mat.data[4] + this.data[8] * mat.data[7];
  t3 = this.data[6] * mat.data[2] + this.data[7] * mat.data[5] + this.data[8] * mat.data[8];
  this.data[6] = t1;
  this.data[7] = t2;
  this.data[8] = t3;
};

/**
 * Sets the data of this matrix to the inverse of the input
 */
Matrix3.prototype.setInverse = function(mat) {
  var t1 = mat.data[0] * mat.data[4];
  var t2 = mat.data[0] * mat.data[5];
  var t3 = mat.data[1] * mat.data[3];
  var t4 = mat.data[2] * mat.data[3];
  var t5 = mat.data[1] * mat.data[6];
  var t6 = mat.data[2] * mat.data[6];

  var det = (
    t1 * mat.data[8] - t2 * mat.data[7] - t3 * mat.data[8] +
    t4 * mat.data[7] + t5 * mat.data[5] - t6 * mat.data[4]);
  if (det == 0) {
    return; // Can't invert
  }
  var invDet = 1 / det;

  this.data[0] = (mat.data[4] * mat.data[8] - mat.data[5] * mat.data[7]) * invDet;
  this.data[1] = -(mat.data[1] * mat.data[8] - mat.data[2] * mat.data[7]) * invDet;
  this.data[2] = (mat.data[1] * mat.data[5] - mat.data[2] * mat.data[4]) * invDet;
  this.data[3] = -(mat.data[3] * mat.data[8] - mat.data[5] * mat.data[6]) * invDet;
  this.data[4] = (mat.data[0] * mat.data[8] - t6) * invDet;
  this.data[5] = -(t2 - t4) * invDet;
  this.data[6] = (mat.data[3] * mat.data[7] - mat.data[4] * mat.data[6]) * invDet;
  this.data[7] = -(mat.data[0] * mat.data[7] - t5) * invDet;
  this.data[8] = (t1 - t3) * invDet;
};

Matrix3.prototype.getInverse = function() {
  var mat = new Matrix3(new Array(9));
  mat.setInverse(this.data);
  return mat;
};

Matrix3.prototype.invert = function() {
  this.setInverse(this.data.splice(0));
};

Matrix3.prototype.setTranspose = function(mat) {
  this.data[0] = mat.data[0];
  this.data[1] = mat.data[3];
  this.data[2] = mat.data[6];
  this.data[3] = mat.data[1];
  this.data[4] = mat.data[4];
  this.data[5] = mat.data[7];
  this.data[6] = mat.data[2];
  this.data[7] = mat.data[5];
  this.data[8] = mat.data[8];
};

Matrix3.prototype.setComponents = function(v1, v2, v3) {
  this.data[0] = v1.x;
  this.data[1] = v2.x;
  this.data[2] = v3.x;
  this.data[3] = v1.y;
  this.data[4] = v2.y;
  this.data[5] = v3.y;
  this.data[6] = v1.z;
  this.data[7] = v2.z;
  this.data[8] = v3.z;
};

Matrix3.prototype.transpose = function() {
  var mat = new Matrix3(new Array(9));
  mat.setTranspose(this.data);
  return mat;
};

Matrix3.prototype.setOrientation = function(q) {
  this.data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
  this.data[1] = (2 * q.i * q.j + 2 * q.k * q.r);
  this.data[2] = (2 * q.i * q.k - 2 * q.j * q.r);
  this.data[3] = (2 * q.i * q.j - 2 * q.k * q.r);
  this.data[4] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
  this.data[5] = (2 * q.j * q.k + 2 * q.i * q.r);
  this.data[6] = (2 * q.i * q.k + 2 * q.j * q.r);
  this.data[7] = (2 * q.j * q.k - 2 * q.i * q.r);
  this.data[8] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
};

function Matrix4(data) {
  // Copy input matrix data array (3 x 3)
  this.data = data.slice(0);
}

Matrix4.prototype.makeIdentity = function() {
  this.data[0] = 1;
  this.data[1] = 0;
  this.data[2] = 0;
  this.data[3] = 0;
  this.data[4] = 0;
  this.data[5] = 1;
  this.data[6] = 0;
  this.data[7] = 0;
  this.data[8] = 0;
  this.data[9] = 0;
  this.data[10] = 1;
  this.data[11] = 0;
};

Matrix4.prototype.multiplyVector = function(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2] + this.data[3],
      v.x * this.data[4] + v.y * this.data[5] + v.z * this.data[6] + this.data[7],
      v.x * this.data[8] + v.y * this.data[9] + v.z * this.data[10] + this.data[11]);
};

Matrix4.prototype.multiplyVectorDirection = function(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2],
      v.x * this.data[4] + v.y * this.data[5] + v.z * this.data[6],
      v.x * this.data[8] + v.y * this.data[9] + v.z * this.data[10]);
};

Matrix4.prototype.multiplyVectorInverseDirection = function(v) {
  return new Vector3(
    v.x * this.data[0] + v.y * this.data[4] + v.z * this.data[8],
    v.x * this.data[1] + v.y * this.data[5] + v.z * this.data[9],
    v.x * this.data[2] + v.y * this.data[6] + v.z * this.data[10]);
};

Matrix4.prototype.multiplyMatrix = function(mat) {
  var data = new Array(12);
  data[0] = mat.data[0] * this.data[0] + mat.data[4] * this.data[1] + mat.data[8] * this.data[2];
  data[4] = mat.data[0] * this.data[4] + mat.data[4] * this.data[5] + mat.data[8] * this.data[6];
  data[8] = mat.data[0] * this.data[8] + mat.data[4] * this.data[9] + mat.data[8] * this.data[10];

  data[1] = mat.data[1] * this.data[0] + mat.data[5] * this.data[1] + mat.data[9] * this.data[2];
  data[5] = mat.data[1] * this.data[4] + mat.data[5] * this.data[5] + mat.data[9] * this.data[6];
  data[9] = mat.data[1] * this.data[8] + mat.data[5] * this.data[9] + mat.data[9] * this.data[10];

  data[2] = mat.data[2] * this.data[0] + mat.data[6] * this.data[1] + mat.data[10] * this.data[2];
  data[6] = mat.data[2] * this.data[4] + mat.data[6] * this.data[5] + mat.data[10] * this.data[6];
  data[10] = mat.data[2] * this.data[8] + mat.data[6] * this.data[9] + mat.data[10] * this.data[10];

  data[3] = mat.data[3] * this.data[0] + mat.data[7] * this.data[1] + mat.data[11] * this.data[2];
  data[7] = mat.data[3] * this.data[4] + mat.data[7] * this.data[5] + mat.data[11] * this.data[6];
  data[11] = mat.data[3] * this.data[8] + mat.data[7] * this.data[9] + mat.data[11] * this.data[10];
  return new Matrix4(data);
};

Matrix4.prototype.getDeterminant = function() {
  return this.data[8] * this.data[5] * this.data[2] +
      this.data[4] * this.data[9] * this.data[2] +
      this.data[8] * this.data[1] * this.data[6] -
      this.data[0] * this.data[9] * this.data[6] -
      this.data[4] * this.data[1] * this.data[10] +
      this.data[0] * this.data[5] * this.data[10];
};

Matrix4.prototype.setInverse = function(mat) {
  var det = mat.getDeterminant();
  if (det == 0) {
    return; // can't invert
  }
  var invDet = 1 / det;
  this.data[0] = (-mat.data[9] * mat.data[6] + mat.data[5] * mat.data[10]) * invDet;
  this.data[4] = (mat.data[8] * mat.data[6] - mat.data[4] * mat.data[10]) * invDet;
  this.data[8] = (-mat.data[8] * mat.data[5] + mat.data[4] * mat.data[9] * mat.data[15]) * invDet;

  this.data[1] = (mat.data[9] * mat.data[2] - mat.data[1] * mat.data[10]) * invDet;
  this.data[5] = (-mat.data[8] * mat.data[2] + mat.data[0] * mat.data[10]) * invDet;
  this.data[9] = (mat.data[8] * mat.data[1] - mat.data[0] * mat.data[9] * mat.data[15]) * invDet;

  this.data[2] = (-mat.data[5] * mat.data[2] + mat.data[1] * mat.data[6] * mat.data[15]) * invDet;
  this.data[6] = (mat.data[4] * mat.data[2] - mat.data[0] * mat.data[6] * mat.data[15]) * invDet;
  this.data[10] = (-mat.data[4] * mat.data[1] + mat.data[0] * mat.data[5] * mat.data[15]) * invDet;

  this.data[3] = (
      mat.data[9] * mat.data[6] * mat.data[3] -
      mat.data[5] * mat.data[10] * mat.data[3] -
      mat.data[9] * mat.data[2] * mat.data[7] +
      mat.data[1] * mat.data[10] * mat.data[7] +
      mat.data[5] * mat.data[2] * mat.data[11] -
      mat.data[1] * mat.data[6] * mat.data[11]) * invDet;
  this.data[7] = (
      -mat.data[8] * mat.data[6] * mat.data[3] +
      mat.data[4] * mat.data[10] * mat.data[3] +
      mat.data[8] * mat.data[2] * mat.data[7] -
      mat.data[0] * mat.data[10] * mat.data[7] -
      mat.data[4] * mat.data[2] * mat.data[11] +
      mat.data[0] * mat.data[6] * mat.data[11]) * invDet;
  this.data[11] = (
      mat.data[8] * mat.data[5] * mat.data[3] -
      mat.data[4] * mat.data[9] * mat.data[3] -
      mat.data[8] * mat.data[1] * mat.data[7] +
      mat.data[0] * mat.data[9] * mat.data[7] +
      mat.data[4] * mat.data[1] * mat.data[11] -
      mat.data[0] * mat.data[5] * mat.data[11]) * invDet;
};

Matrix4.prototype.getInverse = function() {
  var mat = new Matrix4(new Array(12));
  mat.setInverse(this.data);
  return mat;
};

Matrix4.prototype.invert = function() {
  this.setInverse(this.data.splice(0));
};

Matrix4.prototype.setOrientationAndPos = function(q, v) {
  this.data[0] = 1 - (2 * q.j * q.j + 2 * q.k * q.k);
  this.data[1] = (2 * q.i * q.j + 2 * q.k * q.r);
  this.data[2] = (2 * q.i * q.k - 2 * q.j * q.r);
  this.data[3] = v.x;

  this.data[4] = (2 * q.i * q.j - 2 * q.k * q.r);
  this.data[5] = 1 - (2 * q.i * q.i + 2 * q.k * q.k);
  this.data[6] = (2 * q.j * q.k + 2 * q.i * q.r);
  this.data[7] = v.y;

  this.data[8] = (2 * q.i * q.k + 2 * q.j * q.r);
  this.data[9] = (2 * q.j * q.k - 2 * q.i * q.r);
  this.data[10] = 1 - (2 * q.i * q.i + 2 * q.j * q.j);
  this.data[11] = v.z;
};