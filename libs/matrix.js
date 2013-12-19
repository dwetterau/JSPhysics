function Matrix3(data) {
  // Copy input matrix data array (3 x 4)
  this.data = data.slice(0);
}

Matrix3.prototype.multiplyVector(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2],
      v.x * this.data[3] + v.y * this.data[4] + v.z * this.data[5],
      v.x * this.data[6] + v.y * this.data[7] + v.z * this.data[8]);
}

Matrix3.prototype.multiplyMatrix(mat) {
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
}

Matrix3.prototype.multiplyMatrixInPlace(mat) {
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
}

function Matrix4(data) {
  // Copy input matrix data array (3 x 3)
  this.data = data.slice(0);
}

Matrix4.prototype.multiplyVector(v) {
  return new Vector3(
      v.x * this.data[0] + v.y * this.data[1] + v.z * this.data[2] + this.data[3],
      v.x * this.data[4] + v.y * this.data[5] + v.z * this.data[6] + this.data[7],
      v.x * this.data[8] + v.y * this.data[9] + v.z * this.data[10] + this.data[11]);
}

Matrix4.prototype.multiplyMatrix(mat) {
  data = new Array(12);
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
}
