function Matrix3(data) {
  // Copy input matrix data array (3 x 4)
  this.data = data.slice(0);
}

Matrix3.prototype.multiplyVector = function(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2],
      v.x * this.data[3] + v.y * this.data[4] + v.z * this.data[5],
      v.x * this.data[6] + v.y * this.data[7] + v.z * this.data[8]);
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

Matrix3.prototype.transpose = function() {
  var mat = new Matrix3(new Array(9));
  mat.setTranspose(this.data);
  return mat;
};

function Matrix4(data) {
  // Copy input matrix data array (3 x 3)
  this.data = data.slice(0);
}

Matrix4.prototype.multiplyVector = function(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2] + this.data[3],
      v.x * this.data[4] + v.y * this.data[5] + v.z * this.data[6] + this.data[7],
      v.x * this.data[8] + v.y * this.data[9] + v.z * this.data[10] + this.data[11]);
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

