function Collision(point, normal, penetration) {
  this.point = point;
  this.normal = normal;
  this.penetration = penetration;
  this.contactToWorld = new Matrix3(new Array(9));
}

Collision.prototype.setBodyIds = function(firstId, secondId) {
  this.firstId = firstId;
  this.secondId = secondId;
};

Collision.prototype.calculateContactBasis = function(worldBodies) {
  var body1 = worldBodies[this.firstId];
  var body2 = worldBodies[this.secondId];

  var s, firstNormal, secondNormal;
  if (Math.abs(this.normal.x) > Math.abs(this.normal.y)) {
    s = 1.0 / Math.sqrt(this.normal.z * this.normal.z + this.normal.x * this.normal.x);
    firstNormal = new Vector3(
      this.normal.z * s,
      0,
      -this.normal.x * s
    );
    secondNormal = new Vector3(
      this.normal.y * firstNormal.x,
      this.normal.z * firstNormal.x - this.normal.x * firstNormal.z,
      -this.normal.y * firstNormal.x
    );
  } else {
    s = 1.0 / Math.sqrt(this.normal.z * this.normal.z + this.normal.y * this.normal.y);
    firstNormal = new Vector3(
      0,
      -this.normal.z * s,
      this.normal.y * s
    );
    secondNormal = new Vector3(
      this.normal.y * firstNormal.z - this.normal.z * firstNormal.y,
      -this.normal.x * firstNormal.z,
      this.normal.x * firstNormal.y
    );
  }
  this.contactToWorld.setComponents(this.normal, firstNormal, secondNormal);
};

function CollisionDetector(threshold) {
  this.threshold = threshold;
}

CollisionDetector.prototype.getCollisions = function(body_1, body_2) {
  if (body_1.isSphere() && body_2.isSphere()) {
    return this.getSphereSphereCollisions(body_1, body_2);
  }
  if (body_1.isSphere() && body_2.isPlane()) {
    return this.getSpherePlaneCollisions(body_1, body_2, false);
  }
  if (body_1.isPlane() && body_2.isSphere()) {
    return this.getSpherePlaneCollisions(body_2, body_1, true);
  }
  throw Error("Unknown Geometry / Collision Type");
};

CollisionDetector.prototype.getSphereSphereCollisions = function(body_1, body_2) {
  var normalLine = body_1.getPosition().copy();
  normalLine.subInPlace(body_2.getPosition());

  var radius_1 = body_1.getGeometry().r;
  var radius_2 = body_2.getGeometry().r;

  var size = normalLine.magnitude();
  if (size <= this.threshold || size >= radius_1 + radius_2) {
    return [];
  }
  var normal = normalLine.scale(1.0 / size);
  return [
    new Collision(
        body_1.getPosition().add(normalLine.scale(.5)),
        normal,
        radius_1 + radius_2 - size
    )
  ];
};

CollisionDetector.prototype.getSpherePlaneCollisions = function(sphere, plane, flipped) {
  var centerDistance = plane.getGeometry().normal.dot(sphere.getPosition()) - plane.getGeometry().offset;

  if (plane.getGeometry().isHalfSpace) {
    var sphereDistance = centerDistance - sphere.getGeometry().r;
    if (sphereDistance >= this.threshold) {
      return [];
    }
    return [
      new Collision(
          sphere.getPosition().sub(plane.getGeometry().normal.dot(sphereDistance * sphere.getGeometry().r)),
          (flipped ? plane.getGeometry().normal.scale(-1) : plane.getGeometry().copy()),
          -sphereDistance
      )
    ];
  } else {
    var r = sphere.getGeometry().r;
    if (centerDistance * centerDistance > r * r) {
      return [];
    }
    var normal = plane.getGeometry().normal.copy();
    var penetration = -centerDistance;
    if (centerDistance < 0) {
      centerDistance = -centerDistance;
      normal.scaleInPlace(-1);
    }
    penetration += r;
    return [
      new Collision(
          sphere.getPosition().sub(plane.getGeometry().normal.dot(centerDistance)),
          (flipped ? normal.scaleInPlace(-1) : normal),
          penetration
      )
    ]
  }
};