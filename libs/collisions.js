function Collision(point, normal, penetration) {
  this.point = point;
  this.normal = normal;
  this.penetration = penetration;
}

function CollisionDetector(threshold) {
  this.threshold = threshold;
}

CollisionDetector.prototype.getCollisions = function(body_1, body_2) {
  if (body_1.isSphere() && body_2.isSphere()) {
    return this.getSphereSphereCollisions(body_1, body_2);
  }
  throw Error("Unknown Geometry Type");
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